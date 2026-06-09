//
// Created by biao on 24-9-18.
//


#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H
#include <memory>
#include <sysu219_guide_controller/common/mathTypes.h>

#include "FeetEndCalc.h"


class Estimator;
class WaveGenerator;
struct CtrlComponent;
class StateTrotting; // 【新增】前向声明

class GaitGenerator {
public:
// 【修改】构造函数新增参数
    explicit GaitGenerator(CtrlComponent &ctrl_component, StateTrotting* trotting_ptr = nullptr);
    // explicit GaitGenerator(CtrlComponent &ctrl_component);

    ~GaitGenerator() = default;

    void setGait(Vec2 vxy_goal_global, double d_yaw_goal, double gait_height);

    void generate(Vec34 &feet_pos, Vec34 &feet_vel);

    void restart();

    [[nodiscard]] const Vec34& getEndFeetPos() const { return end_p_; } // 获得预期落足点的位置

private:
    Vec3 getFootPos(int i);

    Vec3 getFootVel(int i);

    /**
     * Calculate the position of the foot in the XY plane
     * @param startXY
     * @param endXY
     * @param phase
     * @return
     */
    static double cycloidXYPosition(double startXY, double endXY, double phase);

    /**
     * Calculate the position of the foot in the Z direction
     * @param startZ
     * @param height
     * @param phase
     * @return
     */
    static double cycloidZPosition(double startZ, double height, double phase);

    /**
     * Calculate the velocity of the foot in the XY plane
     * @param startXY
     * @param endXY
     * @param phase
     * @return
     */
    [[nodiscard]] double cycloidXYVelocity(double startXY, double endXY, double phase) const;

    /**
     * Calculate the velocity of the foot in the Z direction
     * @param height
     * @param phase
     * @return
     */
    [[nodiscard]] double cycloidZVelocity(double height, double phase) const;

    std::shared_ptr<WaveGenerator> &wave_generator_;
    std::shared_ptr<Estimator> &estimator_;
    FeetEndCalc feet_end_calc_; // 足端终点位置规划器

    double gait_height_{};
    Vec2 vxy_goal_;
    double d_yaw_goal_{};
    Vec34 start_p_, end_p_, ideal_p_, past_p_;
    bool first_run_;

    StateTrotting* trotting_ptr_; // 【新增】保存 StateTrotting 的指针
    CtrlComponent& ctrl_component_; // 新增：引用控制组件，获取robot_model_

    Vec34 nominal_feet_body_; // 新增：足底在身体坐标系下的名义位置（站立时位置）

};


#endif //GAITGENERATOR_H
