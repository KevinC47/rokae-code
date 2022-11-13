/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 */

/**
 * MOVE指令示例
 */

#include <cmath>
#include <functional>
#include <iostream>
#include <thread>

#include <unistd.h>
#include "control_tools.h"
#include "ini.h"
#include "move.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "model.h"
#include "xmate_exception.h"
#include "follow_pose.h"

using namespace xmate;
using JointControl =
    std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl =
    std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

void camera_update(FollowPoseSixJoints* fp){
  sleep(5);

  double count = 0.0;
  while(true){
    count+=1.0;
    sleep(1);
    Eigen::Transform<double, 3, Eigen::Isometry> bMe_desire = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
    bMe_desire.rotate(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0));
    bMe_desire.pretranslate(Eigen::Vector3d(0.563, 0.2*sin(M_PI/2*count), 0.2));
    fp->update(bMe_desire);
  }

}

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
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // sleep(1);
  robot.setMotorPower(1);

  xmate::XmateModel model(&robot, xmate::XmateType::XMATE7_PRO);
  try {
    const double PI = 3.14159;
    std::array<double, 7> q_init;
    //注意，6轴xMate机器人同样传入7个角度，最后一个数默认是0
    std::array<double, 7> q_drag = {{0,PI/6,0,PI/3,0,PI/3,0}};
    q_init = robot.receiveRobotState().q;
    //回到拖拽位姿
    MOVEJ(0.2, q_init, q_drag, robot);
    //设置TCP
    robot.setCoor({{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}});
    robot.setLoad(0.0, {{0.0, 0.0, 0.0}}, {{0, 0, 0, 0, 0, 0, 0, 0, 0}});

    RCI::robot::RobotState robot_state = robot.receiveRobotState();
    cart_pos pos_start, pos_end, pos_middle, pos_end_x;
    pos_start.pos = robot_state.toolTobase_pos_m;

    Eigen::Matrix3d rot_start, rot_change, rot_end;
    Eigen::Vector3d trans_start, trans_change, trans_end;
    ArrayTo(pos_start.pos, rot_start, trans_start);
    Eigen::Vector3d euler(-0.1, -0.1, -0.1);
    EulerToMatrix(euler, rot_change);
    rot_end = rot_start;
    // rot_end = rot_start*rot_change;
    trans_end = trans_start;
    trans_end[1] -= 0.2;
    ToArray(rot_end, trans_end, pos_end.pos);
    pos_middle = pos_start;
    pos_middle.pos[7] -= 0.1;
    pos_middle.pos[3] -= 0.1;
    robot.setCollisionBehavior({{15,30,20, 8,10, 10 ,6}});
    MOVEL(0.2, pos_start, pos_end, robot);
    robot.setCollisionBehavior({{15,30,20, 8,10, 10,6 }});
    MOVEL(0.2, pos_end, pos_start, robot);
    MOVEC(0.2, pos_start, pos_middle, pos_end, robot);
    MOVEC(0.2, pos_end, pos_middle, pos_start, robot);

    double time = 0;
    if(robot.GetRobotRunningState()!=RobotRunningState::DRAGING){
        robot.startDrag(RCI::robot::DragSpace::CARTESIAN_DRAG, RCI::robot::DragType::FREELY);
        while(time<0.01){
            time+=0.001;
            std::array<double,16> tool_base = robot.receiveRobotState().toolTobase_pos_m;
            usleep(10000);
        }
        robot.stopDrag();
        std::cout<<"停止拖动"<<std::endl;
    }
  } catch (xmate::ControlException &e) {
    std::cout << e.what() << std::endl;
    return 0;
  }

  sleep(1);

  // 动态跟踪示例
  // xmate::XmateModel model(&robot, xmate::XmateType::XMATE3);
  std::cout<<"开始视觉伺服"<<std::endl;
  robot.setMotorPower(1);
  FollowPoseSixJoints fp(robot, model);
  Eigen::Transform<double, 3, Eigen::Isometry> bMe_desire = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  // bMe_desire.rotate(Eigen::Quaterniond(0.18, -0.04, 0.96, -0.2));
  // bMe_desire.rotate(Eigen::Quaterniond(0.0039, -0.0024, 0.9697, -0.2444));
  bMe_desire.rotate(Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0));
  // bMe_desire.pretranslate(Eigen::Vector3d(0.66, -0.208, 0.337));
  bMe_desire.pretranslate(Eigen::Vector3d(0.563, 0, 0.2));
  std::thread thread_camera(camera_update,&fp);
  thread_camera.detach();
  fp.start(bMe_desire);

  std::this_thread::sleep_for(std::chrono::seconds(5));
  fp.update(bMe_desire);
  // bMe_desire = ;
  std::this_thread::sleep_for(std::chrono::seconds(5));
  fp.update(bMe_desire);
  std::this_thread::sleep_for(std::chrono::seconds(5));
  fp.stop();
  return 0;
}
