//
// Created by biao on 24-9-18.
//

#include "unitree_guide_controller/gait/WaveGenerator.h"

#include <iostream>

WaveGenerator::WaveGenerator(const double period, const double st_ratio, const Vec4 &bias) {

    phase_past_ << 0.5, 0.5, 0.5, 0.5;
    contact_past_.setZero();
    status_past_ = WaveStatus::SWING_ALL;
    status_ = WaveStatus::SWING_ALL;

    period_ = period;
    st_ratio_ = st_ratio;
    bias_ = bias;

    if (st_ratio_ >= 1 || st_ratio_ <= 0) {
        std::cerr << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)"
                << std::endl;
        exit(-1);
    }

    for (int i(0); i < bias_.rows(); i++) {
        if (bias_(i) > 1 || bias_(i) < 0) {
            std::cerr << "[ERROR] The bias of WaveGenerator should between [0, 1]"
                    << std::endl;
            exit(-1);
        }
    }
    start_t_ = getSystemTime();
}

auto WaveGenerator::update() -> void {
    // 【第1步】用【当前状态status_】计算【当前相位phase_】和【当前接触状态contact_】
    // 注意：这一步算出来的contact_，在WAVE_ALL模式下，应该是"对角腿1，对角腿0"
    calcWave(phase_, contact_, status_);

    // 【第2步】检测状态是否发生了切换（当前状态 != 上一帧状态）
    if (status_ != status_past_) {
        // 【2.1】如果是刚开始切换（switch_status_全是0），就把切换标志全设为1
        // switch_status_(i)=1 代表第i条腿还没完成状态切换
        if (switch_status_.sum() == 0) {
            switch_status_.setOnes();
        }
        
        // 【2.2】用【上一帧的旧状态status_past_】计算【上一帧的旧相位phase_past_】和【上一帧的旧接触状态contact_past_】
        // 注意：如果上一帧是SWING_ALL，这里算出来的contact_past_就是全0
        calcWave(phase_past_, contact_past_, status_past_);

        // 【2.3】特殊情况处理：修正contact_past_，避免极端切换
        // 情况A：从SWING_ALL（全抬腿）切到STANCE_ALL（全支撑）
        if (status_ == WaveStatus::STANCE_ALL && status_past_ == WaveStatus::SWING_ALL) {
            contact_past_.setOnes(); // 强制把旧接触状态设为全1，方便切换
        } 
        // 情况B：从STANCE_ALL（全支撑）切到SWING_ALL（全抬腿）
        else if (status_ == WaveStatus::SWING_ALL && status_past_ == WaveStatus::STANCE_ALL) {
            contact_past_.setZero(); // 强制把旧接触状态设为全0，方便切换
        }
        // 注意：这里没有处理【SWING_ALL -> WAVE_ALL】的情况！这是死锁的伏笔之一
    }

    // 【第3步】状态切换平滑逻辑（死锁的核心！！！）
    // 如果switch_status_不全为0，说明还有腿没完成切换
    if (switch_status_.sum() != 0) {
        // 遍历4条腿，逐条处理切换
        for (int i(0); i < 4; ++i) {
            // 【3.1】如果这条腿的【新contact】和【旧contact_past_】一样
            if (contact_(i) == contact_past_(i)) {
                switch_status_(i) = 0; // 这条腿切换完成，标志设为0
            } 
            // 【3.2】如果这条腿的【新contact】和【旧contact_past_】不一样！！！
            else {
                // 【死锁核心操作】强制把【新contact】改回【旧contact_past_】！！！
                contact_(i) = contact_past_(i);
                // 同时强制把【新phase】也改回【旧phase_past_】！！！
                phase_(i) = phase_past_(i);
            }
        }
        
        // 【第4步】只有当所有腿都切换完成（switch_status_全为0），才更新status_past_
        if (switch_status_.sum() == 0) {
            status_past_ = status_;
        }
    }
}

void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, const WaveStatus status) {
    switch (status) {
        case WaveStatus::WAVE_ALL: {
            const double past_t = static_cast<double>(getSystemTime() - start_t_) * 1e-6;
            for (int i(0); i < 4; ++i) {
                normal_t_(i) =
                        fmod(past_t + period_ - period_ * bias_(i), period_) / period_;
                if (normal_t_(i) < st_ratio_) {
                    contact(i) = 1;
                    phase(i) = normal_t_(i) / st_ratio_;
                } else {
                    contact(i) = 0;
                    phase(i) = (normal_t_(i) - st_ratio_) / (1 - st_ratio_);
                }
            }
            break;
        }
        case WaveStatus::SWING_ALL: {
            contact.setZero();
            phase << 0.5, 0.5, 0.5, 0.5;
            break;
        }
        case WaveStatus::STANCE_ALL: {
            contact.setOnes();
            phase << 0.5, 0.5, 0.5, 0.5;
            break;
        }
    }
}
