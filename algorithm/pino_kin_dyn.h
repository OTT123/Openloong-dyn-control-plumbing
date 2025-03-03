/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "data_bus.h"
#include <string>
#include "json/json.h"
#include <vector>

class Pin_KinDyn
{
public:
    std::vector<bool> motorReachLimit_;
    const std::vector<std::string> motorName = { "FL_hip_joint","FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}; // joint name in urdf and jason config files
    Eigen::VectorXd motorMaxTorque_;
    Eigen::VectorXd motorMaxPos_;
    Eigen::VectorXd motorMinPos_;

    Eigen::VectorXd tauJointOld_;
    std::string urdf_path_;
    pinocchio::Model model_go2_;
    pinocchio::Model model_go2_fixed_;
    int model_nv;

    Eigen::VectorXd q_, dq_, ddq_;
    Eigen::Matrix3d Rcur_;
    Eigen::Quaternion<double> quatCur_;
    Eigen::MatrixXd dyn_M_, dyn_M_inv_, dyn_C_, dyn_G_, dyn_Ag_, dyn_dAg_;
    Eigen::VectorXd dyn_Non_;
    Eigen::Vector3d CoM_pos_;
    Eigen::Matrix3d inertia_;

    pinocchio::JointIndex base_joint_;
    pinocchio::JointIndex FL_calf_joint_, FR_calf_joint_, RL_calf_joint_, RR_calf_joint_;
    pinocchio::JointIndex FL_thigh_joint_, FR_thigh_joint_, RL_thigh_joint_, RR_thigh_joint_;
    pinocchio::JointIndex FL_calf_joint_fixed_, FR_calf_joint_fixed_, RL_calf_joint_fixed_, RR_calf_joint_fixed_;
    pinocchio::JointIndex FL_thigh_joint_fixed_, FR_thigh_joint_fixed_, RL_thigh_joint_fixed_, RR_thigh_joint_fixed_;

    pinocchio::FrameIndex FL_foot_frame_, FR_foot_frame_, RL_foot_frame_, RR_foot_frame_;
    pinocchio::FrameIndex FL_foot_frame_fixed_, FR_foot_frame_fixed_, RL_foot_frame_fixed_, RR_foot_frame_fixed_;

    Eigen::Matrix<double, 6, -1> J_FL_foot_, J_FR_foot_, J_RL_foot_, J_RR_foot_, J_base_, J_FL_thigh_, J_FR_thigh_, J_RL_thigh_, J_RR_thigh_;
    Eigen::Matrix<double, 6, -1> dJ_FL_foot_, dJ_FR_foot_, dJ_RL_foot_, dJ_RR_foot_, dJ_base_, dJ_FL_thigh_, dJ_FR_thigh_, dJ_RL_thigh_, dJ_RR_thigh_;
    Eigen::Matrix<double, 3, -1> Jcom_;

    Eigen::Vector3d FL_foot_pos_W_, FR_foot_pos_W_, RL_foot_pos_W_, RR_foot_pos_W_; // foot-end position in world frame
    Eigen::Vector3d FL_foot_pos_L_, FR_foot_pos_L_, RL_foot_pos_L_, RR_foot_pos_L_; // foot-end position in body frame
    Eigen::Vector3d FL_foot_vel_L_, FR_foot_vel_L_, RL_foot_vel_L_, RR_foot_vel_L_; // foot-end velcity in body frame
    
    Eigen::Vector3d FL_thigh_pos_W_, FR_thigh_pos_W_, RL_thigh_pos_W_, RR_thigh_pos_W_;
    Eigen::Vector3d FL_thigh_pos_L_, FR_thigh_pos_L_, RL_thigh_pos_L_, RR_thigh_pos_L_;

    Eigen::Matrix3d FL_foot_rot_W_, FR_foot_rot_W_, RL_foot_rot_W_, RR_foot_rot_W_;   // in world frame
    Eigen::Matrix3d FL_thigh_rot_W_, FR_thigh_rot_W_, RL_thigh_rot_W_, RR_thigh_rot_W_;
    Eigen::Matrix3d FL_foot_rot_L_, FR_foot_rot_L_, RL_foot_rot_L_, RR_foot_rot_L_;             // in Body frame (base)

    Eigen::Matrix3d base_rot_;  // in world frame
    Eigen::Vector3d base_pos_;  // in world frame

    enum legIdx
    {
        FL,
        FR,
        RL,
        RR
    };
    struct IkRes
    {
        int status;
        int itr;
        Eigen::VectorXd err;
        Eigen::VectorXd jointPosRes;
    };

    Pin_KinDyn(std::string urdf_pathIn);
    void dataBusRead(DataBus const &robotState);
    void dataBusWrite(DataBus &robotState);
    void computeJ_dJ();
    void computeDyn();
    IkRes computeInK_Leg(const Eigen::Matrix3d &Rdes_FL, const Eigen::Vector3d &Pdes_FL, const Eigen::Matrix3d &Rdes_FR, const Eigen::Vector3d &Pdes_FR, const Eigen::Matrix3d &Rdes_RL, const Eigen::Vector3d &Pdes_RL, const Eigen::Matrix3d &Rdes_RR, const Eigen::Vector3d &Pdes_RR);
    IkRes computeInK_Leg_pos_only(
    const Eigen::Matrix3d &Rdes_FL, const Eigen::Vector3d &Pdes_FL,
    const Eigen::Matrix3d &Rdes_FR, const Eigen::Vector3d &Pdes_FR,
    const Eigen::Matrix3d &Rdes_RL, const Eigen::Vector3d &Pdes_RL,
    const Eigen::Matrix3d &Rdes_RR, const Eigen::Vector3d &Pdes_RR);
    IkRes computeInK_SingleLeg(const Eigen::Matrix3d &Rdes, const Eigen::Vector3d &Pdes, const legIdx &leg_idx);
    IkRes computeInK_Hand(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
    Eigen::VectorXd integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI);
    static Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w);
    void workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT);

private:
    pinocchio::Data data_go2_, data_go2_fixed_;
};
