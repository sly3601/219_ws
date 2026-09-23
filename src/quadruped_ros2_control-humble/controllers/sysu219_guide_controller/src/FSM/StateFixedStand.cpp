//
// Created by biao on 24-9-10.
//

#include "sysu219_guide_controller/FSM/StateFixedStand.h"

StateFixedStand::StateFixedStand(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &target_pos,
                                 const double kp,
                                 const double kd)
    : BaseFixedStand(ctrl_interfaces, target_pos, kp, kd) {
}

FSMStateName StateFixedStand::checkChange() {
    // 失能优先：1 键不受姿态过渡锁定期限制，随时可切
    if (ctrl_interfaces_.control_inputs_.command == 1) {
        return FSMStateName::PASSIVE;
    }
    if (percent_ < 1.5) {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 2:
            return FSMStateName::FIXEDDOWN;
        case 3:
            return FSMStateName::FREESTAND;
        case 4:
            return FSMStateName::TROTTING;
        case 5:
            return FSMStateName::SWINGTEST;
        case 6:
            return FSMStateName::BALANCETEST;
        case 7:
            return FSMStateName::RLWALK;
        case 8:
            // 8 键：先降到半趴，等半趴过渡完成后再自动继续到全趴
            // （见 StateFixedDown::checkChange）
            ctrl_interfaces_.go_prone_after_down = true;
            return FSMStateName::FIXEDDOWN;
        default:
            return FSMStateName::FIXEDSTAND;
    }
}
