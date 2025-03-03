/*
This is part of OpenLoong Dynamics Control, an open project for the control of
biped robot, Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under
Apache 2.0. Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control
in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "data_logger.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include <mujoco/mujoco.h>

// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/go2/scene.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

//************************
// main function
int main(int argc, const char **argv) {
  // ini classes
  UIctr uiController(mj_model, mj_data);        // UI control for Mujoco
  MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
  Pin_KinDyn kinDynSolver(
      "../models/go2/go2_description.urdf"); // kinematics and dynamics solver
  DataBus RobotState(kinDynSolver.model_nv); // data bus
  PVT_Ctr pvtCtr(mj_model->opt.timestep,
                 "../common/joint_ctrl_config.json"); // PVT joint control
  DataLogger logger("../record/datalog.log");         // data logger

  // variables ini
  int model_nv = kinDynSolver.model_nv;

  // ini position and posture for foot-end
  std::vector<double> motors_pos_des(model_nv - 6, 0);
  std::vector<double> motors_pos_cur(model_nv - 6, 0);
  std::vector<double> motors_vel_des(model_nv - 6, 0);
  std::vector<double> motors_vel_cur(model_nv - 6, 0);
  std::vector<double> motors_tau_des(model_nv - 6, 0);
  std::vector<double> motors_tau_cur(model_nv - 6, 0);
  /// ----------------- sim Loop ---------------
  double simEndTime = 1e6;
  mjtNum simstart = mj_data->time;
  double simTime = mj_data->time;
  double startSteppingTime = 3;
  double startWalkingTime = 5;


  // init UI: GLFW
  uiController.iniGLFW();
  uiController.enableTracking();
  uiController.createWindow("Demo", false);

  // 这里尝试匀速直线运动的对角步态
  double P = 5.0; // gait T
  double r = 0.6; // duty factor
  double c = r * P - 0.5 * P;
  double start_at_swing_time = 0.0;
  double end_at_stance_time = P;

  // some const param for go2
  double body_radius = 0.24;
  double body_theta = 0.633; // rad
  const double max_foot_height = 0.15;

  while (!glfwWindowShouldClose(uiController.window)) {
    simstart = mj_data->time;
    while (mj_data->time - simstart < 1.0 / 60.0) {
      mj_step(mj_model, mj_data);

      simTime = mj_data->time;
      double time = simTime - simstart;
      printf("-------------%.3f s------------\n", simTime);
      // read sensor
      mj_interface.updateSensorValues();
      mj_interface.dataBusWrite(RobotState);

      // update kinematics and dynamics info
      kinDynSolver.dataBusRead(RobotState);
      kinDynSolver.computeJ_dJ();
      kinDynSolver.computeDyn();
      kinDynSolver.dataBusWrite(RobotState);

      // set base velocuty des
      Eigen::VectorXd base_vel_des = Eigen::VectorXd::Zero(6);
      base_vel_des[0] = 0.5;
      


      Eigen::Vector3d FL_foot_pos_L_des;
      Eigen::Vector3d RR_foot_pos_L_des;
      Eigen::Vector3d FR_foot_pos_L_des;
      Eigen::Vector3d RL_foot_pos_L_des;
      Eigen::Vector4d FL_foot_quat_L_des = {0, -0.296, 0, 0.955};
      Eigen::Vector4d FR_foot_quat_L_des = {0, -0.296, 0, 0.955};
      Eigen::Vector4d RL_foot_quat_L_des = {0, -0.296, 0, 0.955};
      Eigen::Vector4d RR_foot_quat_L_des = {0, -0.296, 0, 0.955};
      Eigen::Matrix3d FL_foot_rot_L_des = Quat2rot(FL_foot_quat_L_des);
      Eigen::Matrix3d FR_foot_rot_L_des = Quat2rot(FR_foot_quat_L_des);
      Eigen::Matrix3d RL_foot_rot_L_des = Quat2rot(RL_foot_quat_L_des);
      Eigen::Matrix3d RR_foot_rot_L_des = Quat2rot(RR_foot_quat_L_des);

      auto resQ = kinDynSolver.computeInK_Leg_pos_only(
          FL_foot_rot_L_des, FL_foot_pos_L_des, FR_foot_rot_L_des,
          FR_foot_pos_L_des, RL_foot_rot_L_des, RL_foot_pos_L_des,
          RR_foot_rot_L_des, RR_foot_pos_L_des);

      RobotState.motors_pos_des = eigen2std(resQ.jointPosRes);
      RobotState.motors_vel_des = motors_vel_des;
      RobotState.motors_tor_des = motors_tau_des;

      pvtCtr.dataBusRead(RobotState);
      pvtCtr.calMotorsPVT();

      pvtCtr.dataBusWrite(RobotState);
      mj_interface.setMotorsTorque(RobotState.motors_tor_out);
    }

    if (mj_data->time >= simEndTime) {
      break;
    }

    uiController.updateScene();
  }

  //    // free visualization storage
  uiController.Close();

  // free MuJoCo model and data, deactivate
  mj_deleteData(mj_data);
  mj_deleteModel(mj_model);

  return 0;
}