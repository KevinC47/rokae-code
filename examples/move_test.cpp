/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * MOVE指令示例
 */

#include <cmath>
#include <iostream>
#include <functional>

#include <unistd.h>
#include "ini.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"
#include "control_tools.h"

using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    sleep(1);
    robot.setMotorPower(1);

    const double PI = 3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);

    RCI::robot::RobotState robot_state = robot.receiveRobotState();
    cart_pos pos_start,pos_end,pos_middle;
    pos_start.pos = robot_state.toolTobase_pos_m;

    Eigen::Matrix3d rot_start,rot_change,rot_end;
    Eigen::Vector3d trans_start,trans_change,trans_end;
    ArrayTo(pos_start.pos,rot_start,trans_start);
    Eigen::Vector3d euler(-0.1,-0.1,-0.1);
    EulerToMatrix(euler,rot_change);
    rot_end = rot_start;
    trans_end = trans_start;
    trans_end[1] -= 0.2;
    ToArray(rot_end,trans_end,pos_end.pos);
    pos_middle = pos_start;
    pos_middle.pos[7] -= 0.1;
    pos_middle.pos[3] -= 0.1;
    MOVEL(0.2, pos_start, pos_end,robot);
    MOVEL(0.2, pos_end, pos_start,robot);
    MOVEC(0.2,pos_start,pos_middle,pos_end,robot);
    MOVEC(0.2,pos_end,pos_middle,pos_start,robot);

    return 0;
}