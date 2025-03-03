/*
This is part of OpenLoong Dynamics Control, an open project for the control of
biped robot, Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under
Apache 2.0. Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control
in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
/////////////////////////////////
//  这是一个足端阻抗控制的demo     //
/////////////////////////////////
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "data_logger.h"
#include "pino_kin_dyn.h"
#include "tools.h"
#include "useful_math.h"
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include <mujoco/mujoco.h>

// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/go2/scene_fixed.xml", 0, error, 1000);
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
  double xv_des = 0.7; // desired velocity in x direction
  int model_nv = kinDynSolver.model_nv;

  Eigen::Matrix3d K_p;
  K_p.diagonal() << 500, 500, 500;
  Eigen::Matrix3d K_d;
  K_d.diagonal() << 0, 0, 0;

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

  while (!glfwWindowShouldClose(uiController.window)) {
    simstart = mj_data->time;
    while (mj_data->time - simstart < 1.0 / 60.0) {
      mj_step(mj_model, mj_data);

      simTime = mj_data->time;
      double time = simTime - simstart;
      printf("-------------%.3f s------------\n", simTime);
      mj_interface.updateSensorValues();
      mj_interface.dataBusWrite(RobotState);

      // update kinematics and dynamics info
      kinDynSolver.dataBusRead(RobotState);
      kinDynSolver.computeJ_dJ();
      kinDynSolver.computeDyn();
      kinDynSolver.dataBusWrite(RobotState);

      Eigen::Vector3d FL_foot_pos_L_des;
      Eigen::Vector3d RR_foot_pos_L_des;
      Eigen::Vector3d FR_foot_pos_L_des;
      Eigen::Vector3d RL_foot_pos_L_des;
      Eigen::Vector3d FL_foot_vel_L_des;
      Eigen::Vector3d RR_foot_vel_L_des;
      Eigen::Vector3d FR_foot_vel_L_des;
      Eigen::Vector3d RL_foot_vel_L_des;
      double T = 2.0;
      double omg = 2 * 3.14 / T;
      double simple_phase = fmod(simTime, T);
      std::cout << "simple_phase = " << simple_phase << std::endl;
      bool simple_swing = simple_phase < (T / 2) ? 1 : 0;
      double stand_legLength = 0.35;
      double foot_step = 0.1;
      double foot_height = 0.1;
      double temp1 = std::clamp(sin(simTime * omg), 0.0, 1.0);
      double temp2 = std::clamp(-sin(simTime * omg), 0.0, 1.0);
      if (simple_swing) {
        FL_foot_pos_L_des = {0.231 + simple_phase * foot_step, 0.16,
                             -stand_legLength + foot_height * temp1};
        FL_foot_vel_L_des = {foot_step, 0.0,
                             foot_height * omg * cos(simTime * omg)};
        RR_foot_pos_L_des = {-0.153 + simple_phase * foot_step, -0.16,
                             -stand_legLength + foot_height * temp1};
        RR_foot_vel_L_des = {foot_step, 0.0,
                             foot_height * omg * cos(simTime * omg)};
        FR_foot_pos_L_des = {0.231, -0.16, -stand_legLength};
        FR_foot_vel_L_des = {0.0, 0.0, 0.0};
        RL_foot_pos_L_des = {-0.153, 0.16, -stand_legLength};
        RL_foot_vel_L_des = {0.0, 0.0, 0.0};
      } else {
        FL_foot_pos_L_des = {0.231, 0.16, -stand_legLength};
        FL_foot_vel_L_des = {0.0, 0.0, 0.0};
        RR_foot_pos_L_des = {-0.153, -0.16, -stand_legLength};
        RR_foot_vel_L_des = {0.0, 0.0, 0.0};
        FR_foot_pos_L_des = {0.231 + (simple_phase - T / 2) * foot_step, -0.16,
                             -stand_legLength + foot_height * temp2};
        FR_foot_vel_L_des = {foot_step, 0,
                             -foot_height * omg * cos(simTime * omg)};
        RL_foot_pos_L_des = {-0.153 + (simple_phase - T / 2) * foot_step, 0.16,
                             -stand_legLength + foot_height * temp2};
        RL_foot_vel_L_des = {foot_step, 0,
                             -foot_height * omg * cos(simTime * omg)};
      }
      Eigen::Vector3d FL_foot_pos_L_error =
          FL_foot_pos_L_des - RobotState.FL_foot_pos_L;
      Eigen::Vector3d FR_foot_pos_L_error =
          FR_foot_pos_L_des - RobotState.FR_foot_pos_L;
      Eigen::Vector3d RL_foot_pos_L_error =
          RL_foot_pos_L_des - RobotState.RL_foot_pos_L;
      Eigen::Vector3d RR_foot_pos_L_error =
          RR_foot_pos_L_des - RobotState.RR_foot_pos_L;
      Eigen::Vector3d FL_foot_vel_L_error =
          FL_foot_vel_L_des - RobotState.FL_foot_vel_L;
      Eigen::Vector3d FR_foot_vel_L_error =
          FR_foot_vel_L_des - RobotState.FR_foot_vel_L;
      Eigen::Vector3d RL_foot_vel_L_error =
          RL_foot_vel_L_des - RobotState.RL_foot_vel_L;
      Eigen::Vector3d RR_foot_vel_L_error =
          RR_foot_vel_L_des - RobotState.RR_foot_vel_L;
      Eigen::VectorXd tau_FL =
          RobotState.J_FL_foot.topRows(3)
              .rightCols(RobotState.model_nv - 6)
              .transpose() *
          (K_p * FL_foot_pos_L_error + K_d * FL_foot_vel_L_error);
      Eigen::VectorXd tau_FR =
          RobotState.J_FR_foot.topRows(3)
              .rightCols(RobotState.model_nv - 6)
              .transpose() *
          (K_p * FR_foot_pos_L_error + K_d * FR_foot_vel_L_error);
      Eigen::VectorXd tau_RL =
          RobotState.J_RL_foot.topRows(3)
              .rightCols(RobotState.model_nv - 6)
              .transpose() *
          (K_p * RL_foot_pos_L_error + K_d * RL_foot_vel_L_error);
      Eigen::VectorXd tau_RR =
          RobotState.J_RR_foot.topRows(3)
              .rightCols(RobotState.model_nv - 6)
              .transpose() *
          (K_p * RR_foot_pos_L_error + K_d * RR_foot_vel_L_error);
      Eigen::VectorXd tau = Eigen::VectorXd::Zero(RobotState.model_nv - 6);
      tau = tau_FL + tau_FR + tau_RL + tau_RR;
      PrintVecMat("tau", tau);
      PrintVecMat("tau_FL", tau_FL);
      PrintVecMat("local J FL foot", RobotState.J_FL_foot.topRows(3).rightCols(
                                         RobotState.model_nv-6));
      PrintVecMat("local f FL foot", (K_p * FL_foot_pos_L_error + K_d * FL_foot_vel_L_error));
      PrintVecMat("FL_foot_pos_L_error", FL_foot_pos_L_error);
                                         

      // Eigen::Vector4d FL_foot_quat_L_des = {0, -0.296, 0, 0.955};
      // Eigen::Vector4d FR_foot_quat_L_des = {0, -0.296, 0, 0.955};
      // Eigen::Vector4d RL_foot_quat_L_des = {0, -0.296, 0, 0.955};
      // Eigen::Vector4d RR_foot_quat_L_des = {0, -0.296, 0, 0.955};
      // Eigen::Matrix3d FL_foot_rot_L_des = Quat2rot(FL_foot_quat_L_des);
      // Eigen::Matrix3d FR_foot_rot_L_des = Quat2rot(FR_foot_quat_L_des);
      // Eigen::Matrix3d RL_foot_rot_L_des = Quat2rot(RL_foot_quat_L_des);
      // Eigen::Matrix3d RR_foot_rot_L_des = Quat2rot(RR_foot_quat_L_des);
      // auto resQ = kinDynSolver.computeInK_Leg_pos_only(
      //     FL_foot_rot_L_des, FL_foot_pos_L_des, FR_foot_rot_L_des,
      //     FR_foot_pos_L_des, RL_foot_rot_L_des, RL_foot_pos_L_des,
      //     RR_foot_rot_L_des, RR_foot_pos_L_des);

      // RobotState.motors_pos_des = eigen2std(resQ.jointPosRes);
      // RobotState.motors_vel_des = motors_vel_des;
      // RobotState.motors_tor_des = motors_tau_des;

      // pvtCtr.dataBusRead(RobotState);
      // pvtCtr.calMotorsPVT();

      // pvtCtr.dataBusWrite(RobotState);
      auto tau_std = eigen2std(tau);
      mj_interface.setMotorsTorque(tau_std);
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