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
#include "tools.h"
#include "useful_math.h"
#include <GLFW/glfw3.h>
#include <bezier_1D.h>
#include <cstdio>
#include <iostream>
#include <mujoco/mujoco.h>

// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/go2/scene.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);
double Trajectory(double phase, double hei, double len, double phi);
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
  double time_swing = 0.25;
  double phi = 0.0;
  const double foot_height = 0.1;
  const double stand_legLength = 0.3;

  // some const param for go2
  double body_radius = 0.24;
  double body_theta = 0.633; // rad
  Eigen::VectorXi foot_contact_state =
      Eigen::VectorXi::Zero(4);     // FL, FR, RL, RR
  foot_contact_state << 1, 0, 0, 1; // 1 for stance
  bool is_init = false;
  Eigen::Vector3d swingStartPos_W[2];  // 规定第一个为F，第二个为R
  Eigen::Vector3d stanceStartPos_W[2]; // 规定第一个为F，第二个为R
  Eigen::Vector3d posDes_W[4];
  Eigen::Vector3d posDes_W_old[4];

  while (!glfwWindowShouldClose(uiController.window)) {
    simstart = mj_data->time;
    while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) {
      mj_step(mj_model, mj_data);

      simTime = mj_data->time;
      double time = simTime - simstart;
      printf("-------------%.3f s, phi= %.3f------------\n", simTime, phi);
      // read sensor
      mj_interface.updateSensorValues();
      mj_interface.dataBusWrite(RobotState);
      Eigen::VectorXd joint_init_pos = Eigen::VectorXd::Zero(12);
      joint_init_pos << 0, 0.78, -1.57, 0, 0.78, -1.57, 0, 0.78, -1.57, 0, 0.78,
          -1.57;
      if (simTime <= 3) {
        RobotState.motors_pos_des = eigen2std(joint_init_pos);
        RobotState.motors_vel_des = motors_vel_des;
        RobotState.motors_tor_des = motors_tau_des;
        pvtCtr.dataBusRead(RobotState);
        if (simTime <= 1.5) {
          pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415);
        }
        { pvtCtr.calMotorsPVT(); }
        pvtCtr.dataBusWrite(RobotState);
        mj_interface.setMotorsTorque(RobotState.motors_tor_out);
      } else {
        // update kinematics and dynamics info
        kinDynSolver.dataBusRead(RobotState);
        kinDynSolver.computeJ_dJ();
        kinDynSolver.computeDyn();
        kinDynSolver.dataBusWrite(RobotState);

        // set base velocuty des
        // vx, vy, vz, roll, pitch, yaw
        Eigen::VectorXd base_vel_des = Eigen::VectorXd::Zero(6);
        base_vel_des[0] = 0.0;
        base_vel_des[5] = 0.0;
        double dPhi = 1.0 / time_swing * mj_model->opt.timestep;
        phi += dPhi;
        // 假设是FL, RR先为支撑腿
        if (!is_init) {
          is_init = true;
          if (foot_contact_state ==
              (Eigen::VectorXi(4) << 1, 0, 0, 1).finished()) {
            swingStartPos_W[0] = RobotState.FR_foot_pos_W;
            swingStartPos_W[1] = RobotState.RL_foot_pos_W;
            stanceStartPos_W[0] = RobotState.FL_foot_pos_W;
            stanceStartPos_W[1] = RobotState.RR_foot_pos_W;
            // swingStartPos_W[0][2] = 0.0;
            // swingStartPos_W[1][2] = 0.0;
            // stanceStartPos_W[0][2] = 0.0;
            // stanceStartPos_W[1][2] = 0.0;
            foot_contact_state << 1, 0, 0, 1;
          } else {
            swingStartPos_W[0] = RobotState.FL_foot_pos_W;
            swingStartPos_W[1] = RobotState.RR_foot_pos_W;
            stanceStartPos_W[0] = RobotState.FR_foot_pos_W;
            stanceStartPos_W[1] = RobotState.RL_foot_pos_W;
            // swingStartPos_W[0][2] = 0.0;
            // swingStartPos_W[1][2] = 0.0;
            // stanceStartPos_W[0][2] = 0.0;
            // stanceStartPos_W[1][2] = 0.0;
            foot_contact_state << 0, 1, 1, 0;
          }
        }
        // 切换
        if (foot_contact_state ==
                (Eigen::VectorXi(4) << 1, 0, 0, 1).finished() &&
            phi >= 1.0 - dPhi) {
          foot_contact_state << 0, 1, 1, 0;
          swingStartPos_W[0] = RobotState.FL_foot_pos_W;
          swingStartPos_W[1] = RobotState.RR_foot_pos_W;
          stanceStartPos_W[0] = RobotState.FR_thigh_pos_W;
          stanceStartPos_W[1] = RobotState.RL_thigh_pos_W;
          stanceStartPos_W[0][2] -= stand_legLength;
          stanceStartPos_W[1][2] -= stand_legLength;
          // swingStartPos_W[0][2] = 0.0;
          // swingStartPos_W[1][2] = 0.0;
          // stanceStartPos_W[0][2] = 0.0;
          // stanceStartPos_W[1][2] = 0.0;
          phi = 0;
        } else if (foot_contact_state ==
                       (Eigen::VectorXi(4) << 0, 1, 1, 0).finished() &&
                   phi >= 1.0 - dPhi) {
          foot_contact_state << 1, 0, 0, 1;
          swingStartPos_W[0] = RobotState.FR_foot_pos_W;
          swingStartPos_W[1] = RobotState.RL_foot_pos_W;
          stanceStartPos_W[0] = RobotState.FL_thigh_pos_W;
          stanceStartPos_W[1] = RobotState.RR_thigh_pos_W;
          stanceStartPos_W[0][2] -= stand_legLength;
          stanceStartPos_W[1][2] -= stand_legLength;
          // swingStartPos_W[0][2] = 0.0;
          // swingStartPos_W[1][2] = 0.0;
          // stanceStartPos_W[0][2] = 0.0;
          // stanceStartPos_W[1][2] = 0.0;
          phi = 0;
        }
        if (phi >= 1) {
          phi = 1;
        }

        Eigen::Matrix3d KP, Rz;
        double k_vx = 1.0;
        double k_vy = 1.0;
        double kp_wz = 1.0;
        KP.setZero();
        KP(0, 0) = k_vx;
        KP(1, 1) = k_vy;
        KP(2, 2) = 0.0;
        double yawCur = RobotState.rpy[2];
        Rz << cos(yawCur), -sin(yawCur), 0, sin(yawCur), cos(yawCur), 0, 0, 0,
            1; // world to base
        KP = Rz * KP * Rz.transpose();
        Eigen::Vector3d rot_temp;

        double theta_F = 0.0;
        double omegaZ_W = RobotState.base_omega_W(2);
        // FR theta_F
        theta_F = yawCur - body_theta + omegaZ_W * (1 - phi) * time_swing +
                  0.5 * omegaZ_W * time_swing +
                  kp_wz * (omegaZ_W - base_vel_des[2]);
        Eigen::Vector3d temp_rot;
        if (foot_contact_state ==
            (Eigen::VectorXi(4) << 1, 0, 0, 1).finished()) {
          // for swing FR
          temp_rot << cos(theta_F), sin(theta_F), 0.0;
          posDes_W[1] = RobotState.base_pos + body_radius * temp_rot +
                        KP * (RobotState.base_vel - base_vel_des.head(3)) +
                        0.5 * time_swing * RobotState.base_vel +
                        RobotState.base_vel * (1 - phi) * time_swing;
          // for swing RL
          theta_F += 3.1415; // RL theta_F
          temp_rot << cos(theta_F), sin(theta_F), 0.0;
          posDes_W[2] = RobotState.base_pos + body_radius * temp_rot +
                        KP * (RobotState.base_vel - base_vel_des.head(3)) +
                        0.5 * time_swing * RobotState.base_vel +
                        RobotState.base_vel * (1 - phi) * time_swing;
          posDes_W[1][2] = RobotState.base_pos[2] - stand_legLength;
          posDes_W[2][2] = RobotState.base_pos[2] - stand_legLength;
          posDes_W[0] = stanceStartPos_W[0];
          posDes_W[3] = stanceStartPos_W[1];

        } else if (foot_contact_state ==
                   (Eigen::VectorXi(4) << 0, 1, 1, 0).finished()) {
          theta_F += 2 * body_theta; // FL theta_F
          temp_rot << cos(theta_F), sin(theta_F), 0.0;
          posDes_W[0] = RobotState.base_pos + body_radius * temp_rot +
                        KP * (RobotState.base_vel - base_vel_des.head(3)) +
                        0.5 * time_swing * RobotState.base_vel +
                        RobotState.base_vel * (1 - phi) * time_swing;
          theta_F += 3.1415; // for RR
          temp_rot << cos(theta_F), sin(theta_F), 0.0;
          posDes_W[3] = RobotState.base_pos + body_radius * temp_rot +
                        KP * (RobotState.base_vel - base_vel_des.head(3)) +
                        0.5 * time_swing * RobotState.base_vel +
                        RobotState.base_vel * (1 - phi) * time_swing;
          posDes_W[0][2] = RobotState.base_pos[2] - stand_legLength;
          posDes_W[3][2] = RobotState.base_pos[2] - stand_legLength;
          posDes_W[1] = stanceStartPos_W[0];
          posDes_W[2] = stanceStartPos_W[1];
        }

        Eigen::Vector3d pDesCur[4];
        if (phi < 1.0) {
          if (foot_contact_state ==
              (Eigen::VectorXi(4) << 1, 0, 0, 1).finished()) {
            pDesCur[1][0] = swingStartPos_W[0][0] +
                            (posDes_W[1][0] - swingStartPos_W[0][0]) /
                                (2 * 3.1415) *
                                (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
            pDesCur[1][1] = swingStartPos_W[0][1] +
                            (posDes_W[1][1] - swingStartPos_W[0][1]) /
                                (2 * 3.1415) *
                                (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
            // pDesCur[1][2] =
            //     posDes_W[1][2] +
            //     Trajectory(0.2, stand_legLength,
            //                posDes_W[1][2] - swingStartPos_W[0][2], phi);
            pDesCur[1][2] = swingStartPos_W[0][2] +
                            foot_height * 0.5 * (1 - cos(2 * 3.1415 * phi));

            pDesCur[2][0] = swingStartPos_W[1][0] +
                            (posDes_W[2][0] - swingStartPos_W[1][0]) /
                                (2 * 3.1415) *
                                (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
            pDesCur[2][1] = swingStartPos_W[1][1] +
                            (posDes_W[2][1] - swingStartPos_W[1][1]) /
                                (2 * 3.1415) *
                                (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
            pDesCur[2][2] = swingStartPos_W[1][2] +
                            foot_height * 0.5 * (1 - cos(2 * 3.1415 * phi));
            ;
            pDesCur[0] = posDes_W[0];
            pDesCur[3] = posDes_W[3];
          } else if (foot_contact_state ==
                     (Eigen::VectorXi(4) << 0, 1, 1, 0).finished()) {
            pDesCur[0][0] = swingStartPos_W[0][0] +
                            (posDes_W[0][0] - swingStartPos_W[0][0]) /
                                (2 * 3.1415) *
                                (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
            pDesCur[0][1] = swingStartPos_W[0][1] +
                            (posDes_W[0][1] - swingStartPos_W[0][1]) /
                                (2 * 3.1415) *
                                (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
            pDesCur[0][2] =
                swingStartPos_W[0][2] + 0.1 * 0.5 * (1 - cos(2 * 3.1415 * phi));

            pDesCur[3][0] = swingStartPos_W[1][0] +
                            (posDes_W[3][0] - swingStartPos_W[1][0]) /
                                (2 * 3.1415) *
                                (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
            pDesCur[3][1] = swingStartPos_W[1][1] +
                            (posDes_W[3][1] - swingStartPos_W[1][1]) /
                                (2 * 3.1415) *
                                (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
            pDesCur[3][2] =
                swingStartPos_W[1][2] + 0.1 * 0.5 * (1 - cos(2 * 3.1415 * phi));
            pDesCur[1] = stanceStartPos_W[0];
            pDesCur[2] = stanceStartPos_W[1];
          }
        }
        Eigen::Vector3d pDesCur_L[4];
        Eigen::Vector3d posDes_L[4];
        for (int i = 0; i < 4; ++i) {
          pDesCur_L[i] = RobotState.base_rot.transpose() * pDesCur[i] -
                         RobotState.base_rot.transpose() * RobotState.base_pos;
          posDes_L[i] = RobotState.base_rot.transpose() * posDes_W[i] -
                        RobotState.base_rot.transpose() * RobotState.base_pos;
        }

        auto Qres = kinDynSolver.computeInK_Leg_pos_only(
            pDesCur_L[0], pDesCur_L[1], pDesCur_L[2], pDesCur_L[3]);

        RobotState.motors_pos_des = eigen2std(Qres.jointPosRes);
        RobotState.motors_vel_des = motors_vel_des;
        RobotState.motors_tor_des = motors_tau_des;
        pvtCtr.dataBusRead(RobotState);

        if (simTime <= 3) {
          pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415);
        }
        { pvtCtr.calMotorsPVT(); }

        pvtCtr.dataBusWrite(RobotState);

        Eigen::Vector3d FL_foot_pos_L_error =
            pDesCur_L[0] - RobotState.FL_foot_pos_L;
        Eigen::Vector3d FR_foot_pos_L_error =
            pDesCur_L[1] - RobotState.FR_foot_pos_L;
        Eigen::Vector3d RL_foot_pos_L_error =
            pDesCur_L[2] - RobotState.RL_foot_pos_L;
        Eigen::Vector3d RR_foot_pos_L_error =
            pDesCur_L[3] - RobotState.RR_foot_pos_L;

        Eigen::Matrix3d K_p;
        K_p.diagonal() << 200, 200, 200;
        Eigen::Matrix3d K_d;
        K_d.diagonal() << 50, 50, 50;
        Eigen::VectorXd tau_FL = RobotState.J_FL_foot.topRows(3)
                                     .rightCols(RobotState.model_nv - 6)
                                     .transpose() *
                                 (K_p * FL_foot_pos_L_error);
        Eigen::VectorXd tau_FR = RobotState.J_FR_foot.topRows(3)
                                     .rightCols(RobotState.model_nv - 6)
                                     .transpose() *
                                 (K_p * FR_foot_pos_L_error);
        Eigen::VectorXd tau_RL = RobotState.J_RL_foot.topRows(3)
                                     .rightCols(RobotState.model_nv - 6)
                                     .transpose() *
                                 (K_p * RL_foot_pos_L_error);
        Eigen::VectorXd tau_RR = RobotState.J_RR_foot.topRows(3)
                                     .rightCols(RobotState.model_nv - 6)
                                     .transpose() *
                                 (K_p * RR_foot_pos_L_error);
        Eigen::VectorXd tau_fd = Eigen::VectorXd::Zero(RobotState.model_nv - 6);
        tau_fd = tau_FL + tau_FR + tau_RL + tau_RR;
        auto tau_std = eigen2std(tau_fd);
        for (int i = 0; i < 12; i++) {
          tau_std.at(i) += RobotState.motors_tor_out.at(i);
        }
        mj_interface.setMotorsTorque(tau_std);
        std::cout << "foot_contact_state= " << foot_contact_state.transpose()
                  << std::endl;
        PrintVecMat("base pos ", RobotState.base_pos);
        PrintVecMat("swingStartPos_W 0 ", swingStartPos_W[0]);
        PrintVecMat("swingStartPos_W 1 ", swingStartPos_W[1]);
        PrintVecMat("stanceStartPos_W 0", stanceStartPos_W[0]);
        PrintVecMat("stanceStartPos_W 1", stanceStartPos_W[1]);
        PrintVecMat("posDes_W 0", posDes_W[0]);
        PrintVecMat("posDes_W 1", posDes_W[1]);
        PrintVecMat("posDes_W 2", posDes_W[2]);
        PrintVecMat("posDes_W 3", posDes_W[3]);

        PrintVecMat("posDes_L 0", posDes_L[0]);
        PrintVecMat("posDes_L 1", posDes_L[1]);
        PrintVecMat("posDes_L 2", posDes_L[2]);
        PrintVecMat("posDes_L 3", posDes_L[3]);
        PrintVecMat("pDesCur_L 0", pDesCur_L[0]);
        PrintVecMat("pDesCur_L 1", pDesCur_L[1]);
        PrintVecMat("pDesCur_L 2", pDesCur_L[2]);
        PrintVecMat("pDesCur_L 3", pDesCur_L[3]);
        PrintVecMat("FL_foot_pos_L", RobotState.FL_foot_pos_L);
        PrintVecMat("FR_foot_pos_L", RobotState.FR_foot_pos_L);
        PrintVecMat("RL_foot_pos_L", RobotState.RL_foot_pos_L);
        PrintVecMat("RR_foot_pos_L", RobotState.RR_foot_pos_L);
        PrintVecMat("Ik error", Qres.err);
        PrintVecMat("Qres", Qres.jointPosRes);
        PrintVecMat("Qrel", RobotState.q.tail(model_nv - 6));
        std::cout << Qres.status << std::endl;
      }
      if (mj_data->time >= simEndTime) {
        break;
      }
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

double Trajectory(double phase, double hei, double len, double phi) {
  Bezier_1D Bswpid;
  double para0 = 5, para1 = 3;
  for (int i = 0; i < para0; i++) {
    Bswpid.P.push_back(0.0);
  }
  for (int i = 0; i < para1; i++) {
    Bswpid.P.push_back(1.0);
  }

  double output;
  if (phi < phase) {
    output = hei * Bswpid.getOut(phi / phase);
  } else {
    double s = Bswpid.getOut((1.4 - phi) / (1.4 - phase));
    if (s > 0) {
      output = hei * s + len * (1.0 - s);
    } else {
      output = len;
    }
  }
  return output;
}