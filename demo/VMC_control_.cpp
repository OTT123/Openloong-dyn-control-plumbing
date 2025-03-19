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
#define PI (3.141892654f)
#define g (9.8)
#define M (15.0)
// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/go2/scene.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

double GetDesCurXY(double init_x, double des_x, double phi);

//************************
// main function
int main(int argc, const char **argv) {
  // ini classes
  UIctr uiController(mj_model, mj_data);        // UI control for Mujoco
  MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
  Pin_KinDyn kinDynSolver(
      "../models/go2/go2_description.urdf");  // kinematics and dynamics solver
  DataBus RobotState(kinDynSolver.model_nv_); // data bus
  PVT_Ctr pvtCtr(mj_model->opt.timestep,
                 "../common/joint_ctrl_config.json"); // PVT joint control
  DataLogger logger("../record/datalog.log");         // data logger

  // variables ini
  double xv_des = 0.7; // desired velocity in x direction
  int model_nv = kinDynSolver.model_nv_;

  // control param
  const double Kp_tau_x = 5.0;
  const double Kd_tau_x = 1.0;
  const double Kp_tau_y = 5.0;
  const double Kd_tau_y = 1.0;
  const double Kp_tau_z = 5.0;
  const double Kd_tau_z = 1.0;
  const double Kp_f_x = 5.0;
  const double Kd_f_x = 1.0;
  const double Kp_f_y = 5.0;
  const double Kd_f_y = 1.0;
  const double Kp_f_z = 5.0;
  const double Kd_f_z = 1.0;

  const double Tf = 0.3;
  const double Ts = 0.3;
  double loop_time = 0.0;
  const double dPhi = 1.0 / Ts * mj_model->opt.timestep;
  double phi = 0.0;
  // some const param for go2
  double body_radius = 0.24;
  const double body_theta = 0.633; // rad
  const double foot_height = 0.05;

  // 初始化支撑状态
  RobotState.Fst = FL;
  RobotState.Rst = RR;
  RobotState.Fsw = FR;
  RobotState.Rsw = RL;
  // ini position and posture for foot-end
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

  Eigen::VectorXd joint_init_pos = Eigen::VectorXd::Zero(12);
  joint_init_pos << 0, 0.78, -1.57, 0, 0.78, -1.57, 0, 0.78, -1.57, 0, 0.78,
      -1.57;
  std::vector<double> motors_vel_des(model_nv - 6, 0);
  std::vector<double> motors_tau_des(model_nv - 6, 0);

  while (!glfwWindowShouldClose(uiController.window)) {
    simstart = mj_data->time;
    while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) {
      mj_step(mj_model, mj_data);
      simTime = mj_data->time;
      // update sensor
      mj_interface.updateSensorValues();
      mj_interface.dataBusWrite(RobotState);

      printf("-------------%.3f s------------\n", simTime);
      if (simTime <= 2) {
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
        loop_time += mj_model->opt.timestep;
        phi += dPhi;
        // update kinematics and dynamics info
        kinDynSolver.dataBusRead(RobotState);
        kinDynSolver.computeJ_dJ();
        kinDynSolver.computeDyn();
        kinDynSolver.dataBusWrite(RobotState);

        // set target base vel, 机体坐标系
        // vx, vy, vz, wx, wy, wz
        Eigen::VectorXd base_vel_des = Eigen::VectorXd::Zero(6);
        base_vel_des[0] = 0.0;
        base_vel_des[5] = 0.0;
        double h_des = 0.3;

        // phase_swap
        if (loop_time > 0.75 * Tf) {
          if (RobotState.Fst == FL) { // 此时支撑为FL, RR
            // 持续检查摆动腿是否已经触地, 触地后换相
            if ((RobotState.foot_is_contact[FR]) &&
                (RobotState.foot_is_contact[RL])) {
              RobotState.Fst = FR;
              RobotState.Fsw = FL;
              RobotState.Rst = RL;
              RobotState.Rsw = RR;
              loop_time = 0.0;
              phi = 0.0;
            }
          } else if (RobotState.Fst = FR) { // 此时支撑为FR, RL
            if ((RobotState.foot_is_contact[FL]) &&
                (RobotState.foot_is_contact[RR])) {
              RobotState.Fst = FL;
              RobotState.Fsw = FR;
              RobotState.Rst = RR;
              RobotState.Rsw = RL;
              loop_time = 0.0;
              phi = 0.0;
            }
          }
        }

        // 记录摆动足起始位置和初始速度
        // static Eigen::Vector3d pre_Fst_toe = {0,0,0};
        // static Eigen::Vector3d pre_Rst_toe = {0,0,0};
        static Eigen::Vector3d F_sw_init_pos_W = {0, 0, 0};
        static Eigen::Vector3d R_sw_init_pos_W = {0, 0, 0};
        static Eigen::Vector3d F_sw_init_pos_L = {0, 0, 0};
        static Eigen::Vector3d R_sw_init_pos_L = {0, 0, 0};
        static Eigen::Vector3d F_sw_init_vel_L = {0, 0, 0};
        static Eigen::Vector3d R_sw_init_vel_L = {0, 0, 0};
        if (loop_time <= mj_model->opt.timestep) {
          if (RobotState.Fst == FL) { //摇摆腿为FR, RL
            F_sw_init_pos_W = RobotState.FR_foot_pos_W;
            R_sw_init_pos_W = RobotState.RL_foot_pos_W;
            F_sw_init_vel_L = RobotState.FR_foot_vel_L;
            R_sw_init_vel_L = RobotState.RL_foot_vel_L;
          } else if (RobotState.Fst == FR) { //摇摆腿为FL, RR
            F_sw_init_pos_W = RobotState.FL_foot_pos_W;
            R_sw_init_pos_W = RobotState.RR_foot_pos_W;
            F_sw_init_vel_L = RobotState.FL_foot_vel_L;
            R_sw_init_vel_L = RobotState.RR_foot_vel_L;
          }
          F_sw_init_pos_L =
              RobotState.base_rot.transpose() * F_sw_init_pos_W -
              RobotState.base_rot.transpose() * RobotState.base_pos;
          R_sw_init_pos_L =
              RobotState.base_rot.transpose() * R_sw_init_pos_W -
              RobotState.base_rot.transpose() * RobotState.base_pos;
        }

        // 提取ST足和SW足的坐标
        Eigen::VectorXd F_st_pos_L, R_st_pos_L, F_sw_pos_L, R_sw_pos_L;
        Eigen::VectorXd F_sw_pos_End_to_Hip, R_sw_pos_End_to_Hip;
        Eigen::MatrixXd F_st_J, R_st_J;
        if (RobotState.Fst == FL) {
          F_st_pos_L = RobotState.FL_foot_pos_L;
          R_st_pos_L = RobotState.RR_foot_pos_L;
          F_sw_pos_L = RobotState.FR_foot_pos_L;
          R_sw_pos_L = RobotState.RL_foot_pos_L;
          F_st_J =
              RobotState.J_FL_foot_body.topRows(3).middleCols(0, 3); // 线性部分
          R_st_J = RobotState.J_RR_foot_body.topRows(3).middleCols(9, 3);
        } else if (RobotState.Fst == FR) {
          F_st_pos_L = RobotState.FR_foot_pos_L;
          R_st_pos_L = RobotState.RL_foot_pos_L;
          F_sw_pos_L = RobotState.FL_foot_pos_L;
          R_sw_pos_L = RobotState.RR_foot_pos_L;
          F_st_J = RobotState.J_FR_foot_body.topRows(3).middleCols(3, 3);
          R_st_J = RobotState.J_RL_foot_body.topRows(3).middleCols(6, 3);
        }

        // 支撑相控制
        // 求伪pitch
        static double pre_psi = 0;
        double psi = atan((F_st_pos_L[2] - R_st_pos_L[2]) /
                          (F_st_pos_L[0] - R_st_pos_L[0])); // rad
        double dpsi = (psi - pre_psi) / (mj_model->opt.timestep);
        pre_psi = psi;
        // 求机身高度
        static double pre_h = 0.3;
        double h = -(F_st_pos_L[2] + R_st_pos_L[2]) / 2.0;
        double dh = (h - pre_h) / (mj_model->opt.timestep);
        pre_h = h;

        auto base_vel_L = RobotState.base_rot.transpose() * RobotState.base_vel;

        double Tx = -(Kp_tau_x * RobotState.base_rpy[0] +
                      Kd_tau_x * RobotState.base_omega_L[0]);
        double Ty = -(Kp_tau_y * psi + Kd_tau_y * dpsi);

        double Fz = -(Kp_f_z * (h - h_des) + Kd_f_z * dh);

        double Fx = -(Kp_f_x * (base_vel_L[0] - base_vel_des[0]));
        double Tz =
            -(Kp_tau_z * (RobotState.base_omega_L[2] - base_vel_des[5]));

        Eigen::MatrixXd inv_Q, Q;
        Eigen::VectorXd st_f, F;
        st_f = Eigen::VectorXd::Zero(6);
        inv_Q = Eigen::MatrixXd::Zero(6, 6);
        Q = Eigen::MatrixXd::Zero(6, 6);
        F = Eigen::VectorXd::Zero(6);
        F[0] = Fx - M * g * sin(RobotState.base_rpy[1]);
        F[1] = Fz + M * g * cos(RobotState.base_rpy[1]);
        F[2] = Tx;
        F[3] = Ty;
        F[4] = Tz;
        F[5] = 0;
        Q(0, 0) = 1.0;
        Q(0, 3) = 1.0;
        Q(1, 2) = 1.0;
        Q(1, 5) = 1.0;
        Q(2, 1) = -F_st_pos_L[2];
        Q(2, 2) = F_st_pos_L[1];
        Q(2, 4) = -R_st_pos_L[2];
        Q(2, 5) = R_st_pos_L[1];
        Q(3, 0) = F_st_pos_L[2];
        Q(3, 2) = -F_st_pos_L[0];
        Q(3, 3) = R_st_pos_L[2];
        Q(3, 5) = -R_st_pos_L[0];
        Q(4, 0) = -F_st_pos_L[1];
        Q(4, 1) = F_st_pos_L[0];
        Q(4, 3) = -R_st_pos_L[1];
        Q(4, 4) = R_st_pos_L[0];
        Q(5, 1) = 1;
        Q(5, 4) = -1;
        inv_Q = Q.inverse();
        Eigen::MatrixXd con_J = Eigen::MatrixXd::Zero(6, 6);
        con_J.block(0, 0, 3, 3) = F_st_J.transpose();
        con_J.block(3, 3, 3, 3) = R_st_J.transpose();

        Eigen::VectorXd tau_temp = -con_J * inv_Q * F;
        Eigen::VectorXd tau_st = Eigen::VectorXd::Zero(model_nv - 6);
        tau_st.segment(RobotState.Fst * 3, 3) = tau_temp.head(3);
        tau_st.segment(RobotState.Rst * 3, 3) = tau_temp.tail(3);

        // 接下来求摆动足,在机身坐标系讨论
        Eigen::Matrix3d KP, Rz;
        double k_vx = 1.0;
        double k_vy = 1.0;
        double kp_wz = 1.0;
        KP.setZero();
        KP(0, 0) = k_vx;
        double yawCur = RobotState.rpy[2];
        Rz << cos(yawCur), -sin(yawCur), 0, sin(yawCur), cos(yawCur), 0, 0, 0,
            1; // world to base
        KP = Rz * KP * Rz.transpose();

        double theta_F = 0.0;
        double omegaZ_W = RobotState.base_omega_W(2); // w or b?

        // FR theta_F
        theta_F = yawCur - body_theta + omegaZ_W * (1 - phi) * Ts +
                  0.5 * omegaZ_W * Ts + kp_wz * (omegaZ_W - base_vel_des[2]);
        Eigen::Vector3d temp_rot;
        Eigen::VectorXd sw_foot_des_W[2]; // F, R
        if (RobotState.Fsw == FR) {       // FR, RL is sw
          temp_rot << cos(theta_F), sin(theta_F), 0.0;
          sw_foot_des_W[0] =
              RobotState.base_pos + body_radius * temp_rot +
              KP * (base_vel_des.head(3) - RobotState.base_vel) * (1) +
              0.5 * Ts * RobotState.base_vel +
              RobotState.base_vel * (1 - phi) * Ts;
          theta_F += 3.1415; // RL theta_F
          temp_rot << cos(theta_F), sin(theta_F), 0.0;
          sw_foot_des_W[1] =
              RobotState.base_pos + body_radius * temp_rot +
              KP * (base_vel_des.head(3) - RobotState.base_vel) * (1) +
              0.5 * Ts * RobotState.base_vel +
              RobotState.base_vel * (1 - phi) * Ts;
          sw_foot_des_W[0][2] = RobotState.FR_thigh_pos_W[2] - h_des;
          sw_foot_des_W[1][2] = RobotState.RL_thigh_pos_W[2] - h_des;
        } else if (RobotState.Fsw == FL) { // FL, RR is sw
          theta_F += 2 * body_theta;       // FL theta_F
          temp_rot << cos(theta_F), sin(theta_F), 0.0;
          sw_foot_des_W[0] =
              RobotState.base_pos + body_radius * temp_rot +
              KP * (base_vel_des.head(3) - RobotState.base_vel) * (1) +
              0.5 * Ts * RobotState.base_vel +
              RobotState.base_vel * (1 - phi) * Ts;
          theta_F += 3.1415; // for RR
          temp_rot << cos(theta_F), sin(theta_F), 0.0;
          sw_foot_des_W[1] =
              RobotState.base_pos + body_radius * temp_rot +
              KP * (base_vel_des.head(3) - RobotState.base_vel) * (1) +
              0.5 * Ts * RobotState.base_vel +
              RobotState.base_vel * (1 - phi) * Ts;
          sw_foot_des_W[0][2] = RobotState.FL_thigh_pos_W[2] - h_des;
          sw_foot_des_W[1][2] = RobotState.RR_thigh_pos_W[2] - h_des;
        }
        Eigen::Vector3d sw_foot_des_cur_W[2];
        Eigen::Vector3d sw_foot_des_cur_dot_W[2];
        sw_foot_des_cur_W[0][0] =
            GetDesCurXY(F_sw_init_pos_W[0], sw_foot_des_W[0][0], phi);
        sw_foot_des_cur_W[0][1] =
            GetDesCurXY(F_sw_init_pos_W[1], sw_foot_des_W[0][1], phi);
        sw_foot_des_cur_W[0][2] =
            F_sw_init_pos_W[2] +
            foot_height * 0.5 * (1 - cos(2 * 3.1415 * phi));

        sw_foot_des_cur_W[1][0] =
            GetDesCurXY(R_sw_init_pos_W[0], sw_foot_des_W[1][0], phi);
        sw_foot_des_cur_W[1][1] =
            GetDesCurXY(R_sw_init_pos_W[1], sw_foot_des_W[1][1], phi);
        sw_foot_des_cur_W[1][2] =
            R_sw_init_pos_W[2] +
            foot_height * 0.5 * (1 - cos(2 * 3.1415 * phi));

        //转到body坐标系
        Eigen::Vector3d sw_foot_des_L[2];
        Eigen::Vector3d sw_foot_des_cur_L[2];
        for (int i = 0; i < 2; ++i) {
          sw_foot_des_L[i] =
              RobotState.base_rot.transpose() * sw_foot_des_W[i] -
              RobotState.base_rot.transpose() * RobotState.base_pos;
          sw_foot_des_cur_L[i] =
              RobotState.base_rot.transpose() * sw_foot_des_cur_W[i] -
              RobotState.base_rot.transpose() * RobotState.base_pos;
        }
        Eigen::VectorXd tau_sw = Eigen::VectorXd::Zero(RobotState.model_nv - 6);
        Eigen::VectorXd F_sw_pos_error;
        Eigen::VectorXd F_sw_vel_error;
        Eigen::VectorXd R_sw_pos_error;
        Eigen::VectorXd R_sw_vel_error;
        Eigen::Matrix3d K_p;
        K_p.diagonal() << 50, 50, 50;
        Eigen::Matrix3d K_d;
        K_d.diagonal() << 10, 10, 10;
        if (RobotState.Fsw == FL) {
          F_sw_pos_error = sw_foot_des_cur_L[0] - RobotState.FL_foot_pos_L;
          R_sw_pos_error = sw_foot_des_cur_L[1] - RobotState.RR_foot_pos_L;
          tau_sw += RobotState.J_FL_foot_body.topRows(3).transpose() *
                    (K_p * F_sw_pos_error);
          tau_sw += RobotState.J_RR_foot_body.topRows(3).transpose() *
                    (K_p * R_sw_pos_error);
        } else if (RobotState.Fsw == FR) {
          F_sw_pos_error = sw_foot_des_cur_L[0] - RobotState.FR_foot_pos_L;
          R_sw_pos_error = sw_foot_des_cur_L[1] - RobotState.RL_foot_pos_L;
          tau_sw += RobotState.J_FR_foot_body.topRows(3).transpose() *
                    (K_p * F_sw_pos_error);
          tau_sw += RobotState.J_RL_foot_body.topRows(3).transpose() *
                    (K_p * R_sw_pos_error);
        }

        // // test code
        // if (RobotState.Fsw == FL) {
        //   F_sw_pos_error = F_sw_init_pos_L - RobotState.FL_foot_pos_L;
        //   R_sw_pos_error = R_sw_init_pos_L - RobotState.RR_foot_pos_L;
        //   tau_sw += RobotState.J_FL_foot_body.topRows(3).transpose() *
        //             (K_p * F_sw_pos_error + K_d * (-RobotState.FL_foot_vel_L));
        //   tau_sw += RobotState.J_RR_foot_body.topRows(3).transpose() *
        //             (K_p * R_sw_pos_error + K_d * (-RobotState.RR_foot_vel_L));
        // } else if (RobotState.Fsw == FR) {
        //   F_sw_pos_error = F_sw_init_pos_L - RobotState.FR_foot_pos_L;
        //   R_sw_pos_error = R_sw_init_pos_L - RobotState.RL_foot_pos_L;
        //   tau_sw += RobotState.J_FR_foot_body.topRows(3).transpose() *
        //             (K_p * F_sw_pos_error + K_d * (-RobotState.FR_foot_vel_L));
        //   tau_sw += RobotState.J_RL_foot_body.topRows(3).transpose() *
        //             (K_p * R_sw_pos_error + K_d * (-RobotState.RL_foot_vel_L));
        // }

        auto tau_std = eigen2std(tau_st + tau_sw);
        mj_interface.setMotorsTorque(tau_std);

        std::cout << "Fst, Rst = " << RobotState.Fst << " " << RobotState.Rst
                  << std::endl;
        std::cout << "is contact = " << RobotState.foot_is_contact.transpose()
                  << std::endl;
        PrintVecMat(" tau_st ", tau_st);
        PrintVecMat(" tau_sw ", tau_sw);
        PrintVecMat(" F_sw_init_pos_W ", F_sw_init_pos_W);
        PrintVecMat(" R_sw_init_pos_W ", R_sw_init_pos_W);
        PrintVecMat(" F_sw_init_pos_L ", F_sw_init_pos_L);
        PrintVecMat(" R_sw_init_pos_L ", R_sw_init_pos_L);
        PrintVecMat(" F sw_foot_des_W    ", sw_foot_des_W[0]);
        PrintVecMat(" F sw_foot_des_W cur", sw_foot_des_cur_W[0]);
        PrintVecMat(" R sw_foot_des_W    ", sw_foot_des_W[1]);
        PrintVecMat(" R sw_foot_des_W cur", sw_foot_des_cur_W[1]);
        PrintVecMat(" F sw_foot_des_L    ", sw_foot_des_L[0]);
        PrintVecMat(" F sw_foot_des_L cur", sw_foot_des_cur_L[0]);
        PrintVecMat(" R sw_foot_des_L    ", sw_foot_des_L[1]);
        PrintVecMat(" R sw_foot_des_L cur", sw_foot_des_cur_L[1]);
        PrintVecMat(" F_sw_pos_error ", F_sw_pos_error);
        PrintVecMat(" R_sw_pos_error ", R_sw_pos_error);
        // PrintVecMat(" contact FL force= ", RobotState.foot_force_sensor[0]);
        // PrintVecMat(" contact FR force= ", RobotState.foot_force_sensor[1]);
        // PrintVecMat(" contact RL force= ", RobotState.foot_force_sensor[2]);
        // PrintVecMat(" contact RR force= ", RobotState.foot_force_sensor[3]);
        // spring damping system to get body target force and torque
      }
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

// 计算cur x或者y
double GetDesCurXY(double init_x, double des_x, double phi) {
  return (init_x + (des_x - init_x) / (2 * 3.1415) *
                       (2 * 3.1415 * phi - sin(2 * 3.1415 * phi)));
}
double GetDesCurDotXY(double init_x, double des_x, double phi, double Ts) {
  return ((des_x - init_x) / Ts * (1 - cos(2 * 3.1415 * phi)));
}
double GetDesXY(double init_x, double des_x, double phi) {
  return (init_x + (des_x - init_x) / (2 * 3.1415) *
                       (2 * 3.1415 * phi - sin(2 * 3.1415 * phi)));
}