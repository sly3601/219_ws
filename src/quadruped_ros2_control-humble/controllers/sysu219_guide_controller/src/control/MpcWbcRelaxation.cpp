#include "sysu219_guide_controller/control/MpcWbcRelaxation.h"
#include "quadProgpp/QuadProg++.hh"

// 已有Array.hh定义了solve宏；这里取消它，避免改写本类的solve函数名。
#ifdef solve
#undef solve
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace sysu219::wbc {
namespace {

// 条件不满足就抛异常；solve统一捕获并返回失败结果。
void require(bool condition, const char* message)
{
  if (!condition) throw std::invalid_argument(message);
}

// 按传入矩阵类型直接分解，避免复制到动态堆内存矩阵。
template <typename MatrixType>
bool positiveDefinite(const MatrixType& matrix)
{
  if (!matrix.allFinite() ||
      (matrix - matrix.transpose()).norm() > 1e-9 * std::max(1.0, matrix.norm())) return false;
  Eigen::LLT<MatrixType> llt(matrix);
  return llt.info() == Eigen::Success;
}

// 六条基座动力学等式中的最大绝对分量，用于判断最坏误差。
double infNorm(const Vec6& vector)
{
  return vector.lpNorm<Eigen::Infinity>();
}

}  // namespace


MpcWbcRelaxation::MpcWbcRelaxation(RelaxationSettings settings)
    : settings_(std::move(settings))
{
  // Q_WBC、Q_MPC保持正定，使QuadProg++得到严格凸的二次目标。
  require(positiveDefinite(settings_.Q_WBC) && positiveDefinite(settings_.Q_MPC), // Q_WBC是WBC的权重，Q_MPC是MPC的权重
          "松弛优化权重必须对称正定");
  require(std::isfinite(settings_.absolute_feasibility_tolerance) &&
              settings_.absolute_feasibility_tolerance > 0.0 &&
              std::isfinite(settings_.relative_feasibility_tolerance) &&
              settings_.relative_feasibility_tolerance >= 0.0 &&
              settings_.relative_feasibility_tolerance < 1.0,
          "松弛优化残差容差无效");
}


void MpcWbcRelaxation::validate(const RelaxationInput& in) const
{
  // M、C、qddot_cmd必须来自同一浮动基模型和同一周期。
  require(positiveDefinite(in.M), "浮动基质量矩阵必须对称正定");
  require(in.C.allFinite() && in.qddot_cmd.allFinite() && in.tau_PD_j.allFinite(),
          "动力学偏置、WBC加速度或关节PD力矩无效");

  for (int leg = 0; leg < 4; ++leg)
  {
    require(in.contact[leg] == 0 || in.contact[leg] == 1, "足端接触状态只能为0或1");
    if (in.contact[leg] == 0) continue; // 摆动足不产生接触力变量，不读取其模型和MPC力。

    require(in.f_MPC_O.col(leg).allFinite() && in.J_f_O[leg].allFinite() &&
                in.Jdot_f_qdot_O.col(leg).allFinite(),
            "支撑足力、雅可比或加速度偏置无效");

    const auto& limits = in.force_limits[leg];
    require(std::isfinite(limits.mu) && limits.mu > 0.0 &&
                std::isfinite(limits.f_z_min) && limits.f_z_min >= 0.0 &&
                std::isfinite(limits.f_z_max) && limits.f_z_max >= limits.f_z_min,
            "接触摩擦系数或法向力限值无效");
    require(limits.R_OC.allFinite() &&
                (limits.R_OC.transpose() * limits.R_OC - Eigen::Matrix3d::Identity()).norm() < 1e-7 &&
                std::abs(limits.R_OC.determinant() - 1.0) < 1e-7,
            "接触面坐标矩阵不是有效旋转");
  }

  if (settings_.enforce_torque_limits)
  {
    // 无穷上下界不会传给求解器，而是在构造时省略对应的不等式。
    for (int joint = 0; joint < 12; ++joint)
    {
      require(!std::isnan(in.tau_min_j(joint)) && !std::isnan(in.tau_max_j(joint)) &&
                  in.tau_min_j(joint) <= in.tau_max_j(joint) &&
                  in.tau_min_j(joint) != std::numeric_limits<double>::infinity() &&
                  in.tau_max_j(joint) != -std::numeric_limits<double>::infinity(),
              "关节力矩上下界无效");
    }
  }
}


const RelaxationQp& MpcWbcRelaxation::buildQp(const RelaxationInput& in) const
{
  validate(in);
  auto& qp = qp_;

  // 1. 按腿序挑出支撑足。论文优化向量只含这些脚的力修正，摆动足不占位置。
  qp.n_c = 0;
  qp.stance_legs.fill(-1);
  for (int leg = 0; leg < 4; ++leg)
  {
    if (in.contact[leg] == 1) qp.stance_legs[qp.n_c++] = leg;
  }

  const int n_f = 3 * qp.n_c;       // 每只支撑足的xyz三个力分量。
  const int n_X = 6 + n_f;          // 前6项修正基座加速度，其余修正支撑足力。
  qp.J_c.setZero(n_f, 18);
  qp.f_MPC_O.setZero(n_f);
  qp.C_A.setZero(5 * qp.n_c, n_f);
  qp.c_A_lower.setZero(5 * qp.n_c);
  qp.c_A_upper.setConstant(5 * qp.n_c, std::numeric_limits<double>::infinity());

  for (int i = 0; i < qp.n_c; ++i)
  {
    const int leg = qp.stance_legs[i];
    qp.J_c.middleRows(3 * i, 3) = in.J_f_O[leg];
    qp.f_MPC_O.segment(3 * i, 3) = in.f_MPC_O.col(leg);

    // 论文式(4.44)、(4.52)：每只脚的C_i与上下界。
    // 具体实现沿用仓库MPC的四棱面摩擦限制，填入论文通用的力约束结构。
    // 前4行限制切向力，最后1行限制法向力；摩擦侧面只需零下界。
    const auto& limits = in.force_limits[leg];
    Eigen::Matrix<double, 5, 3> C_i;
    C_i << -1.0, 0.0, limits.mu,
            1.0, 0.0, limits.mu,
            0.0, -1.0, limits.mu,
            0.0, 1.0, limits.mu,
            0.0, 0.0, 1.0;

    // [英文补充] 把接触面方向转换到世界系；R_OC为单位阵时就是水平地面。
    qp.C_A.block(5 * i, 3 * i, 5, 3).noalias() = C_i * limits.R_OC.transpose();
    qp.c_A_lower(5 * i + 4) = limits.f_z_min;
    qp.c_A_upper(5 * i + 4) = limits.f_z_max;
  }


  // 2. 论文式(4.44)～(4.46)：惩罚两类松弛量，默认Q_WBC为I，Q_MPC为0.005I。
  // 论文和QuadProg++的标准目标都有半系数，因此G的两个对角块均取两倍权重。
  qp.G.setZero(n_X, n_X);
  qp.G.topLeftCorner<6, 6>() = 2.0 * settings_.Q_WBC;
  for (int i = 0; i < qp.n_c; ++i)
  {
    for (int j = 0; j < qp.n_c; ++j)
    {
      // Q_MPC按四足保存，按当前支撑编号选块；非对角块也保留，支持力修正之间的耦合。
      qp.G.block(6 + 3 * i, 6 + 3 * j, 3, 3) =
          2.0 * settings_.Q_MPC.block(3 * qp.stance_legs[i], 3 * qp.stance_legs[j], 3, 3);
    }
  }


  // 3. 论文式(4.47)～(4.50)：基座受力与修正后的运动必须满足动力学。
  // 实现说明：Sf只有前6行非零，直接取这6行，避免把12条恒零等式交给QP库。
  const auto M_f = in.M.topRows<6>();                // 6×18，含基座与关节的惯性耦合。
  const Vec6 C_f = in.C.head<6>();
  const auto J_cf_T = qp.J_c.leftCols(6).transpose(); // 接触力对基座的作用，6×n_f。

  // 式(4.48)的缩写存在维度歧义，这里从式(4.47)展开：
  // 常量部分保留完整18维qddot_cmd；只有前6维加速度允许修正。
  // C_E按列存储，前6行对应加速度修正，其余行对应力修正。
  qp.C_E.setZero(n_X, 6);
  qp.C_E.topRows<6>() = M_f.leftCols<6>().transpose();
  qp.C_E.bottomRows(n_f) = -qp.J_c.leftCols(6);
  qp.c_e.noalias() = M_f * in.qddot_cmd + C_f - J_cf_T * qp.f_MPC_O;


  // 4. 论文式(4.52)～(4.57)：将足底力上下界改写为对松弛量的不等式。
  // 下界：系数保持正号，常量为原MPC力的贡献减下界。
  // 上界：系数取负号，常量为上界减原MPC力的贡献。
  // 无穷上界表示没有该约束，省略它能减少QP规模，也避免传入无穷常量。
  int n_I = 6 * qp.n_c; // 每足4个摩擦侧面、法向下界和法向上界。
  if (settings_.enforce_torque_limits)
  {
    for (int joint = 0; joint < 12; ++joint)
    {
      n_I += std::isfinite(in.tau_min_j(joint));
      n_I += std::isfinite(in.tau_max_j(joint));
    }
  }
  qp.C_I.setZero(n_X, n_I);
  qp.c_i.setZero(n_I);

  int constraint = 0;
  for (int row = 0; row < qp.C_A.rows(); ++row)
  {
    const double f_MPC_contribution = qp.C_A.row(row).dot(qp.f_MPC_O);

    qp.C_I.col(constraint).tail(n_f) = qp.C_A.row(row).transpose();
    qp.c_i(constraint++) = f_MPC_contribution - qp.c_A_lower(row);

    if (std::isfinite(qp.c_A_upper(row)))
    {
      qp.C_I.col(constraint).tail(n_f) = -qp.C_A.row(row).transpose();
      qp.c_i(constraint++) = qp.c_A_upper(row) - f_MPC_contribution;
    }
  }


  // [英文补充] 可选总力矩约束，追加在论文足底力约束之后；默认不执行。
  // 系数矩阵描述两类松弛量如何改变力矩，常量为松弛前力矩加外部PD。
  if (settings_.enforce_torque_limits)
  {
    WbcTaskMatrix tau_j_X;
    tau_j_X.setZero(12, n_X);
    tau_j_X.leftCols<6>() = in.M.bottomRows<12>().leftCols<6>();
    tau_j_X.rightCols(n_f) = -qp.J_c.rightCols(12).transpose();
    const Vec12 tau_j_0 = in.M.bottomRows<12>() * in.qddot_cmd + in.C.tail<12>() -
        qp.J_c.rightCols(12).transpose() * qp.f_MPC_O + in.tau_PD_j;

    for (int joint = 0; joint < 12; ++joint)
    {
      if (std::isfinite(in.tau_min_j(joint)))
      {
        qp.C_I.col(constraint) = tau_j_X.row(joint).transpose();
        qp.c_i(constraint++) = tau_j_0(joint) - in.tau_min_j(joint);
      }
      if (std::isfinite(in.tau_max_j(joint)))
      {
        qp.C_I.col(constraint) = -tau_j_X.row(joint).transpose();
        qp.c_i(constraint++) = in.tau_max_j(joint) - tau_j_0(joint);
      }
    }
  }

  require(qp.G.allFinite() && qp.C_E.allFinite() && qp.c_e.allFinite() &&
              qp.C_I.allFinite() && qp.c_i.allFinite(), "松弛QP矩阵含无效数值");
  return qp;
}


RelaxationOutput MpcWbcRelaxation::solve(const RelaxationInput& in) const
{
  RelaxationOutput out;
  try
  {
    const auto& qp = buildQp(in);
    const int n_X = static_cast<int>(qp.G.rows());
    const int n_I = static_cast<int>(qp.C_I.cols());

    // 5. 论文式(4.58)：调用仓库已有QuadProg++。
    // 接口说明：Eigen矩阵要复制到库的Matrix/Vector类型，约束已按列存好，无需再转置。
    // G会被求解器用于原地分解，必须传副本；g0是库要求的一次项，本问题取零。
    // 内存说明：这些库容器与求解器内部仍分配堆内存，未改动原库。
    quadprogpp::Matrix<double> G(n_X, n_X), C_E(n_X, 6), C_I(n_X, n_I);
    quadprogpp::Vector<double> g0(n_X), c_e(6), c_i(n_I), X(n_X);

    // 数值适配：按G的对角线缩放求解变量，使两类变量的数值尺度接近。
    // 权重、目标和约束均不改变；求解后还原为论文中的原变量。
    // 这样避免Q_MPC很大时，QuadProg++内部停止阈值随矩阵尺度放大。
    WbcWorkVector variable_scale(n_X);
    for (int row = 0; row < n_X; ++row)
      variable_scale(row) = 1.0 / std::sqrt(qp.G(row, row));

    // 每条约束再按缩放后的系数长度归一化，保持约束含义与方向不变。
    Vec6 equality_scale;
    RelaxationInequalityVector inequality_scale(n_I);
    for (int col = 0; col < 6; ++col)
    {
      const double norm = qp.C_E.col(col).cwiseProduct(variable_scale).norm();
      equality_scale(col) = norm > 0.0 ? 1.0 / norm : 1.0;
    }
    for (int col = 0; col < n_I; ++col)
    {
      const double norm = qp.C_I.col(col).cwiseProduct(variable_scale).norm();
      inequality_scale(col) = norm > 0.0 ? 1.0 / norm : 1.0;
    }
    require(variable_scale.allFinite() && equality_scale.allFinite() && inequality_scale.allFinite(),
            "松弛QP缩放系数无效");

    for (int row = 0; row < n_X; ++row)
    {
      g0[row] = 0.0;
      X[row] = 0.0;
      for (int col = 0; col < n_X; ++col)
        G[row][col] = variable_scale(row) * qp.G(row, col) * variable_scale(col);
      for (int col = 0; col < 6; ++col)
        C_E[row][col] = variable_scale(row) * qp.C_E(row, col) * equality_scale(col);
      for (int col = 0; col < n_I; ++col)
        C_I[row][col] = variable_scale(row) * qp.C_I(row, col) * inequality_scale(col);
    }
    for (int row = 0; row < 6; ++row) c_e[row] = qp.c_e(row) * equality_scale(row);
    for (int row = 0; row < n_I; ++row) c_i[row] = qp.c_i(row) * inequality_scale(row);

    const double objective = quadprogpp::solve_quadprog(G, g0, C_E, c_e, C_I, c_i, X);
    require(std::isfinite(objective), "松弛QP不可行或求解器失败");

    // 把库输出放回固定容量Eigen向量，便于后续分段和残差检查。
    WbcWorkVector X_solution(n_X);
    for (int row = 0; row < n_X; ++row) X_solution(row) = variable_scale(row) * X[row];
    require(X_solution.allFinite(), "松弛QP返回无效解");


    // 6. 检查解是否满足等式和不等式，误差容差只用于判定结果，不改变优化问题。
    const Vec6 equality_value = qp.C_E.transpose() * X_solution;
    const Vec6 equality_error = equality_value + qp.c_e;
    const double equality_tolerance = settings_.absolute_feasibility_tolerance +
        settings_.relative_feasibility_tolerance * std::max(infNorm(equality_value), infNorm(qp.c_e));
    if (!equality_error.allFinite() || infNorm(equality_error) > equality_tolerance)
    {
      char message[192];
      std::snprintf(message, sizeof(message), "松弛QP动力学等式误差%.3e，超过容差%.3e",
                    infNorm(equality_error), equality_tolerance);
      require(false, message);
    }

    RelaxationInequalityVector inequality_margin(n_I);
    inequality_margin.noalias() = qp.C_I.transpose() * X_solution;
    for (int row = 0; row < n_I; ++row)
    {
      const double tolerance = settings_.absolute_feasibility_tolerance +
          settings_.relative_feasibility_tolerance *
          std::max(std::abs(inequality_margin(row)), std::abs(qp.c_i(row)));
      inequality_margin(row) += qp.c_i(row);
      if (!std::isfinite(inequality_margin(row)) || inequality_margin(row) < -tolerance)
      {
        char message[192];
        std::snprintf(message, sizeof(message), "松弛QP第%d条不等式违反量%.3e，超过容差%.3e",
                      row + 1, -inequality_margin(row), tolerance);
        require(false, message);
      }
    }


    // 7. 论文式(4.43)：取出松弛量，再加回WBC加速度和MPC足底力。
    out.delta_qddot = X_solution.head<6>();
    out.qddot = in.qddot_cmd;
    out.qddot.head<6>() += out.delta_qddot; // 后12维实体关节加速度不修正。
    const WbcTaskVector f_c_O = qp.f_MPC_O + X_solution.tail(n_X - 6);

    for (int i = 0; i < qp.n_c; ++i)
    {
      const int leg = qp.stance_legs[i];
      out.delta_f_O.col(leg) = X_solution.segment(6 + 3 * i, 3);
      out.f_c_O.col(leg) = f_c_O.segment(3 * i, 3); // 未写入的摆动足保持默认零。

      // 中文两阶段方法没有重新约束支撑足加速度；基座松弛可能改变原WBC任务结果。
      // 该残差用于诊断，不参与本节QP的约束。
      out.stance_acceleration_residual_O.col(leg) =
          in.J_f_O[leg] * out.qddot + in.Jdot_f_qdot_O.col(leg);
    }


    // 8. 论文式(4.59)～(4.61)：用修正后的加速度和足底力求关节力矩。
    const Vec18 tau = in.M * out.qddot + in.C - qp.J_c.transpose() * f_c_O;
    out.floating_base_dynamics_residual = tau.head<6>(); // 虚拟基座不能驱动，此处应接近零。
    out.tau_j = tau.tail<12>(); // Sj的选择作用直接用取后12维实现，避免显式构造选择矩阵。
    out.tau_cmd_j = out.tau_j + in.tau_PD_j; // 外部PD属于控制接口，和论文逆动力学力矩分开。

    require(out.qddot.allFinite() && out.f_c_O.allFinite() && tau.allFinite() &&
                out.stance_acceleration_residual_O.allFinite() && out.tau_cmd_j.allFinite(),
            "松弛后的加速度、力或力矩含无效数值");
    out.equality_residual_inf = infNorm(equality_error);
    out.minimum_inequality_margin = n_I ? inequality_margin.minCoeff() :
        std::numeric_limits<double>::infinity();
    out.objective = objective;
    out.success = true;
    out.message = "松弛优化完成，动力学和支撑足残差可用于诊断";
  }
  catch (const std::exception& error)
  {
    out = RelaxationOutput{};
    out.message = error.what(); // 失败时返回清零结果，外部决定如何保底。
  }
  return out;
}

}  // namespace sysu219::wbc
