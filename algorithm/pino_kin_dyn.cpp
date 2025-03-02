/*
This is part of OpenLoong Dynamics Control, an open project for the control of
biped robot, Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under
Apache 2.0. Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control
in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "pino_kin_dyn.h"

#include <utility>

Pin_KinDyn::Pin_KinDyn(std::string urdf_pathIn) {
  pinocchio::JointModelFreeFlyer root_joint;
  pinocchio::urdf::buildModel(urdf_pathIn, root_joint, model_go2_);
  pinocchio::urdf::buildModel(urdf_pathIn, model_go2_fixed_);

  // 为什么有额外的fixed模型？
  // 这个fixed模型的求解结果全都是相对于body坐标系的
  data_go2_ = pinocchio::Data(model_go2_);
  data_go2_fixed_ = pinocchio::Data(model_go2_fixed_);
  model_nv = model_go2_.nv;

  // 变量初始化
  q_.setZero();
  dq_.setZero();
  ddq_.setZero();
  dyn_M_ = Eigen::MatrixXd::Zero(model_nv, model_nv);
  dyn_M_inv_ = Eigen::MatrixXd::Zero(model_nv, model_nv);
  dyn_C_ = Eigen::MatrixXd::Zero(model_nv, model_nv);
  dyn_G_ = Eigen::MatrixXd::Zero(model_nv, 1);
  Rcur_.setIdentity();

  J_FL_foot_ = Eigen::MatrixXd::Zero(6, model_nv);
  J_FR_foot_ = Eigen::MatrixXd::Zero(6, model_nv);
  J_RL_foot_ = Eigen::MatrixXd::Zero(6, model_nv);
  J_RR_foot_ = Eigen::MatrixXd::Zero(6, model_nv);
  J_base_ = Eigen::MatrixXd::Zero(6, model_nv);
  J_FL_hip_ = Eigen::MatrixXd::Zero(6, model_nv);
  J_FR_hip_ = Eigen::MatrixXd::Zero(6, model_nv);
  J_RL_hip_ = Eigen::MatrixXd::Zero(6, model_nv);
  J_RR_hip_ = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_FL_foot_ = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_FR_foot_ = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_RL_foot_ = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_RR_foot_ = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_base_ = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_FL_hip_ = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_FR_hip_ = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_RL_hip_ = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_RR_hip_ = Eigen::MatrixXd::Zero(6, model_nv);

  // get joint id
  base_joint_ = model_go2_.getJointId("root_joint");
  FL_calf_joint_ = model_go2_.getJointId("FL_calf_joint");
  FR_calf_joint_ = model_go2_.getJointId("FR_calf_joint");
  RL_calf_joint_ = model_go2_.getJointId("RL_calf_joint");
  RR_calf_joint_ = model_go2_.getJointId("RR_calf_joint");
  FL_hip_joint_ = model_go2_.getJointId("FL_hip_joint");
  FR_hip_joint_ = model_go2_.getJointId("FR_hip_joint");
  RL_hip_joint_ = model_go2_.getJointId("RL_hip_joint");
  RR_hip_joint_ = model_go2_.getJointId("RR_hip_joint");
  FL_calf_joint_fixed_ = model_go2_fixed_.getJointId("FL_calf_joint");
  FR_calf_joint_fixed_ = model_go2_fixed_.getJointId("FR_calf_joint");
  RL_calf_joint_fixed_ = model_go2_fixed_.getJointId("RL_calf_joint");
  RR_calf_joint_fixed_ = model_go2_fixed_.getJointId("RR_calf_joint");
  FL_hip_joint_fixed_ = model_go2_fixed_.getJointId("FL_hip_joint");
  FR_hip_joint_fixed_ = model_go2_fixed_.getJointId("FR_hip_joint");
  RL_hip_joint_fixed_ = model_go2_fixed_.getJointId("RL_hip_joint");
  RR_hip_joint_fixed_ = model_go2_fixed_.getJointId("RR_hip_joint");
  FL_foot_frame_ = model_go2_.getFrameId("FL_foot");
  FR_foot_frame_ = model_go2_.getFrameId("FR_foot");
  RL_foot_frame_ = model_go2_.getFrameId("RL_foot");
  RR_foot_frame_ = model_go2_.getFrameId("RR_foot");
  FL_foot_frame_fixed_ = model_go2_fixed_.getFrameId("FL_foot");
  FR_foot_frame_fixed_ = model_go2_fixed_.getFrameId("FR_foot");
  RL_foot_frame_fixed_ = model_go2_fixed_.getFrameId("RL_foot");
  RR_foot_frame_fixed_ = model_go2_fixed_.getFrameId("RR_foot");
  // read joint pvt parameters
  // 源代码中读出来全是0，可能是bug
  // 此外这些变量只出现在一个似乎从未被调用过的函数中
  // ？？
  Json::Reader reader;
  Json::Value root_read;
  std::ifstream in("joint_ctrl_config.json", std::ios::binary);
  motorMaxTorque_ = Eigen::VectorXd::Zero(motorName.size());
  motorMaxPos_ = Eigen::VectorXd::Zero(motorName.size());
  motorMinPos_ = Eigen::VectorXd::Zero(motorName.size());
  reader.parse(in, root_read);
  for (int i = 0; i < motorName.size(); i++) {
    motorMaxTorque_(i) = (root_read[motorName[i]]["maxTorque"].asDouble());
    motorMaxPos_(i) = (root_read[motorName[i]]["maxPos"].asDouble());
    motorMinPos_(i) = (root_read[motorName[i]]["minPos"].asDouble());
  }
  motorReachLimit_.assign(motorName.size(), false);
  tauJointOld_ = Eigen::VectorXd::Zero(motorName.size());
}

void Pin_KinDyn::dataBusRead(const DataBus &robotState) {
  //  For Pinocchio: The base translation part is expressed in the parent frame
  //  (here the world coordinate system) while its velocity is expressed in the
  //  body coordinate system.
  //  https://github.com/stack-of-tasks/pinocchio/issues/1137
  //  q_ = [global_base_position, global_base_quaternion, joint_positions]
  //  v = [local_base_velocity_linear, local_base_velocity_angular,
  //  joint_velocities]
  //  robotState.base_rot base to world
  q_ = robotState.q;
  dq_ = robotState.dq;
  dq_.block(0, 0, 3, 1) =
      robotState.base_rot.transpose() * dq_.block(0, 0, 3, 1);
  dq_.block(3, 0, 3, 1) =
      robotState.base_rot.transpose() * dq_.block(3, 0, 3, 1);
  ddq_ = robotState.ddq;
}

void Pin_KinDyn::dataBusWrite(DataBus &robotState) {
  // Jacobian and their dot
  robotState.J_FL_foot = J_FL_foot_;
  robotState.J_FR_foot = J_FR_foot_;
  robotState.J_RL_foot = J_RL_foot_;
  robotState.J_RR_foot = J_RR_foot_;
  robotState.J_base = J_base_;
  robotState.J_FL_hip = J_FL_hip_;
  robotState.J_FR_hip = J_FR_hip_;
  robotState.J_RL_hip = J_RL_hip_;
  robotState.J_RR_hip = J_RR_hip_;
  robotState.dJ_FL_foot = dJ_FL_foot_;
  robotState.dJ_FR_foot = dJ_FR_foot_;
  robotState.dJ_RL_foot = dJ_RL_foot_;
  robotState.dJ_RR_foot = dJ_RR_foot_;
  robotState.dJ_base = dJ_base_;

  // workspace linear pos and velocity
  robotState.FL_foot_pos_W = FL_foot_pos_W_;
  robotState.FR_foot_pos_W = FR_foot_pos_W_;
  robotState.RL_foot_pos_W = RL_foot_pos_W_;
  robotState.RR_foot_pos_W = RR_foot_pos_W_;
  robotState.FL_foot_pos_L = FL_foot_pos_L_;
  robotState.FR_foot_pos_L = FR_foot_pos_L_;
  robotState.RL_foot_pos_L = RL_foot_pos_L_;
  robotState.RR_foot_pos_L = RR_foot_pos_L_;
  robotState.FL_foot_vel_L = FL_foot_vel_L_;
  robotState.FR_foot_vel_L = FR_foot_vel_L_;
  robotState.RL_foot_vel_L = RL_foot_vel_L_;
  robotState.RR_foot_vel_L = RR_foot_vel_L_;
  robotState.FL_hip_pos_W = FL_hip_pos_W_;
  robotState.FR_hip_pos_W = FR_hip_pos_W_;
  robotState.RL_hip_pos_W = RL_hip_pos_W_;
  robotState.RR_hip_pos_W = RR_hip_pos_W_;
  robotState.FL_hip_pos_L = FL_hip_pos_L_;
  robotState.FR_hip_pos_L = FR_hip_pos_L_;
  robotState.RL_hip_pos_L = RL_hip_pos_L_;
  robotState.RR_hip_pos_L = RR_hip_pos_L_;

  // workspace rotation
  robotState.FL_foot_rot_W = FL_foot_rot_W_;
  robotState.FR_foot_rot_W = FR_foot_rot_W_;
  robotState.RL_foot_rot_W = RL_foot_rot_W_;
  robotState.RR_foot_rot_W = RR_foot_rot_W_;
  robotState.FL_foot_rot_L = FL_foot_rot_L_;
  robotState.FR_foot_rot_L = FR_foot_rot_L_;
  robotState.RL_foot_rot_L = RL_foot_rot_L_;
  robotState.RR_foot_rot_L = RR_foot_rot_L_;
  robotState.FL_hip_rot_W = FL_hip_rot_W_;
  robotState.FR_hip_rot_W = FR_hip_rot_W_;
  robotState.RL_hip_rot_W = RL_hip_rot_W_;
  robotState.RR_hip_rot_W = RR_hip_rot_W_;

  // dyn
  robotState.dyn_M = dyn_M_;
  robotState.dyn_M_inv = dyn_M_inv_;
  robotState.dyn_C = dyn_C_;
  robotState.dyn_G = dyn_G_;
  robotState.dyn_Ag = dyn_Ag_;
  robotState.dyn_dAg = dyn_dAg_;
  robotState.dyn_Non = dyn_Non_;

  // com
  robotState.pCoM_W = CoM_pos_;
  robotState.Jcom_W = Jcom_;

  robotState.inertia = inertia_;
}

// update jacobians and joint positions
void Pin_KinDyn::computeJ_dJ() {
  pinocchio::forwardKinematics(model_go2_, data_go2_, q_);
  pinocchio::jacobianCenterOfMass(model_go2_, data_go2_, q_, true);
  //   pinocchio::computeJointJacobians(model_go2_, data_go2, q_);
  pinocchio::computeJointJacobiansTimeVariation(model_go2_, data_go2_, q_, dq_);

  pinocchio::updateGlobalPlacements(model_go2_, data_go2_);
  pinocchio::updateFramePlacements(model_go2_, data_go2_);

  pinocchio::getFrameJacobian(model_go2_, data_go2_, FL_foot_frame_,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_FL_foot_);
  pinocchio::getFrameJacobian(model_go2_, data_go2_, FR_foot_frame_,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_FR_foot_);
  pinocchio::getFrameJacobian(model_go2_, data_go2_, RL_foot_frame_,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_RL_foot_);
  pinocchio::getFrameJacobian(model_go2_, data_go2_, RR_foot_frame_,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_RR_foot_);
  pinocchio::getJointJacobian(model_go2_, data_go2_, FL_hip_joint_,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_FL_hip_);
  pinocchio::getJointJacobian(model_go2_, data_go2_, FR_hip_joint_,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_FR_hip_);
  pinocchio::getJointJacobian(model_go2_, data_go2_, RL_hip_joint_,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_RL_hip_);
  pinocchio::getJointJacobian(model_go2_, data_go2_, RR_hip_joint_,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_RR_hip_);

  pinocchio::getJointJacobianTimeVariation(model_go2_, data_go2_, FL_hip_joint_,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_FL_hip_);
  pinocchio::getJointJacobianTimeVariation(model_go2_, data_go2_, FR_hip_joint_,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_FR_hip_);
  pinocchio::getJointJacobianTimeVariation(model_go2_, data_go2_, RL_hip_joint_,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_RL_hip_);
  pinocchio::getJointJacobianTimeVariation(model_go2_, data_go2_, RR_hip_joint_,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_RR_hip_);
  pinocchio::getJointJacobianTimeVariation(model_go2_, data_go2_, base_joint_,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_base_);
  pinocchio::getFrameJacobianTimeVariation(
      model_go2_, data_go2_, FL_foot_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
      dJ_FL_foot_);
  pinocchio::getFrameJacobianTimeVariation(
      model_go2_, data_go2_, FR_foot_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
      dJ_FR_foot_);
  pinocchio::getFrameJacobianTimeVariation(
      model_go2_, data_go2_, RL_foot_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
      dJ_RL_foot_);
  pinocchio::getFrameJacobianTimeVariation(
      model_go2_, data_go2_, RR_foot_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
      dJ_RR_foot_);

  FL_foot_pos_W_ = data_go2_.oMf[FL_foot_frame_].translation();
  FR_foot_pos_W_ = data_go2_.oMf[FR_foot_frame_].translation();
  RL_foot_pos_W_ = data_go2_.oMf[RL_foot_frame_].translation();
  RR_foot_pos_W_ = data_go2_.oMf[RR_foot_frame_].translation();
  FL_foot_rot_W_ = data_go2_.oMf[FL_foot_frame_].rotation();
  FR_foot_rot_W_ = data_go2_.oMf[FR_foot_frame_].rotation();
  RL_foot_rot_W_ = data_go2_.oMf[RL_foot_frame_].rotation();
  RR_foot_rot_W_ = data_go2_.oMf[RR_foot_frame_].rotation();
  RR_hip_pos_W_ = data_go2_.oMi[RR_hip_joint_].translation();
  RL_hip_pos_W_ = data_go2_.oMi[RL_hip_joint_].translation();
  FR_hip_pos_W_ = data_go2_.oMi[FR_hip_joint_].translation();
  FL_hip_pos_W_ = data_go2_.oMi[FL_hip_joint_].translation();
  RR_hip_rot_W_ = data_go2_.oMi[RR_hip_joint_].rotation();
  RL_hip_rot_W_ = data_go2_.oMi[RL_hip_joint_].rotation();
  FR_hip_rot_W_ = data_go2_.oMi[FR_hip_joint_].rotation();
  FL_hip_rot_W_ = data_go2_.oMi[FL_hip_joint_].rotation();
  base_pos_ = data_go2_.oMi[base_joint_].translation();
  base_rot_ = data_go2_.oMi[base_joint_].rotation();
  Jcom_ = data_go2_.Jcom;

  // trans to world frame
  // 注意这里。pinocchio 的dq定义为：
  // dq_ = [local_base_velocity_linear,
  // local_base_velocity_angular,
  // joint_velocities]
  // 而我们普遍认为广义坐标系的前六个是在世界坐标系中表达，
  // 因此要将前六个自由度的速度从world转换到body坐标系中
  Eigen::MatrixXd Mpj;
  Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
  Mpj.block(0, 0, 3, 3) = base_rot_.transpose();
  Mpj.block(3, 3, 3, 3) = base_rot_.transpose();
  J_base_ = J_base_ * Mpj;
  J_FL_foot_ = J_FL_foot_ * Mpj;
  J_FR_foot_ = J_FR_foot_ * Mpj;
  J_RL_foot_ = J_RL_foot_ * Mpj;
  J_RR_foot_ = J_RR_foot_ * Mpj;
  J_FL_hip_ = J_FL_hip_ * Mpj;
  J_FR_hip_ = J_FR_hip_ * Mpj;
  J_RL_hip_ = J_RL_hip_ * Mpj;
  J_RR_hip_ = J_RR_hip_ * Mpj;
  Jcom_ = Jcom_ * Mpj;
  dJ_base_ = dJ_base_ * Mpj;
  dJ_FL_foot_ = dJ_FL_foot_ * Mpj;
  dJ_FR_foot_ = dJ_FR_foot_ * Mpj;
  dJ_RL_foot_ = dJ_RL_foot_ * Mpj;
  dJ_RR_foot_ = dJ_RR_foot_ * Mpj;
  dJ_FL_hip_ = dJ_FL_hip_ * Mpj;
  dJ_FR_hip_ = dJ_FR_hip_ * Mpj;
  dJ_RL_hip_ = dJ_RL_hip_ * Mpj;
  dJ_RR_hip_ = dJ_RR_hip_ * Mpj;

  // calculate body frame var
  // 这里使用fixed 模型，只要是获取相对于body坐标系的位置和姿态
  Eigen::VectorXd q_fixed;
  q_fixed = q_.block(7, 0, model_go2_fixed_.nv, 1);
  pinocchio::forwardKinematics(model_go2_fixed_, data_go2_fixed_, q_fixed);
  pinocchio::updateGlobalPlacements(model_go2_fixed_, data_go2_fixed_);
  pinocchio::updateFramePlacements(model_go2_fixed_, data_go2_fixed_);
  FL_foot_pos_L_ = data_go2_fixed_.oMf[FL_foot_frame_].translation();
  FR_foot_pos_L_ = data_go2_fixed_.oMf[FR_foot_frame_].translation();
  RL_foot_pos_L_ = data_go2_fixed_.oMf[RL_foot_frame_].translation();
  RR_foot_pos_L_ = data_go2_fixed_.oMf[RR_foot_frame_].translation();
  FL_foot_rot_L_ = data_go2_fixed_.oMf[FL_foot_frame_].rotation();
  FR_foot_rot_L_ = data_go2_fixed_.oMf[FR_foot_frame_].rotation();
  RL_foot_rot_L_ = data_go2_fixed_.oMf[RL_foot_frame_].rotation();
  RR_foot_rot_L_ = data_go2_fixed_.oMf[RR_foot_frame_].rotation();
  FL_hip_pos_L_ = data_go2_fixed_.oMi[FL_hip_joint_fixed_].translation();
  FR_hip_pos_L_ = data_go2_fixed_.oMi[FR_hip_joint_fixed_].translation();
  RL_hip_pos_L_ = data_go2_fixed_.oMi[RL_hip_joint_fixed_].translation();
  RR_hip_pos_L_ = data_go2_fixed_.oMi[RR_hip_joint_fixed_].translation();
}

// update dynamic parameters, M*ddq+C*dq+G=tau
void Pin_KinDyn::computeDyn() {
  // cal M
  pinocchio::crba(model_go2_, data_go2_, q_);
  data_go2_.M.triangularView<Eigen::Lower>() =
      data_go2_.M.transpose().triangularView<Eigen::Lower>();
  dyn_M_ = data_go2_.M;
  // cal Minv
  data_go2_.Minv.triangularView<Eigen::Lower>() =
      data_go2_.Minv.transpose().triangularView<Eigen::Lower>();
  dyn_M_inv_ = data_go2_.Minv;
  // cal C
  pinocchio::computeCoriolisMatrix(model_go2_, data_go2_, q_, dq_);
  dyn_C_ = data_go2_.C;
  // cal G
  pinocchio::computeGeneralizedGravity(model_go2_, data_go2_, q_);
  dyn_G_ = data_go2_.g;
  // cal Ag, Centroidal Momentum Matrix. First three rows: linear, other three
  // rows: angular
  pinocchio::dccrba(model_go2_, data_go2_, q_, dq_);
  pinocchio::computeCentroidalMomentum(model_go2_, data_go2_, q_, dq_);
  dyn_Ag_ = data_go2_.Ag;
  dyn_dAg_ = data_go2_.dAg;
  // cal nonlinear item
  dyn_Non_ = dyn_C_ * dq_ + dyn_G_;

  // cal I
  pinocchio::ccrba(model_go2_, data_go2_, q_, dq_);
  inertia_ = data_go2_.Ig.inertia().matrix();
  // cal CoM
  CoM_pos_ = data_go2_.com[0];
  // transform into world frame
  // 这里同样
  Eigen::MatrixXd Mpj, Mpj_inv;
  Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
  Mpj_inv = Eigen::MatrixXd::Identity(model_nv, model_nv);
  Mpj.block(0, 0, 3, 3) = base_rot_.transpose();
  Mpj.block(3, 3, 3, 3) = base_rot_.transpose();
  Mpj_inv.block(0, 0, 3, 3) = base_rot_;
  Mpj_inv.block(3, 3, 3, 3) = base_rot_;
  dyn_M_ = Mpj_inv * dyn_M_ * Mpj;
  dyn_M_inv_ = Mpj_inv * dyn_M_inv_ * Mpj;
  dyn_C_ = Mpj_inv * dyn_C_ * Mpj;
  dyn_G_ = Mpj_inv * dyn_G_;
  dyn_Non_ = Mpj_inv * dyn_Non_;
}

// Inverse kinematics for leg posture. Note: the Rdes and Pdes are both w.r.t
// the baselink coordinate in body frame!
Pin_KinDyn::IkRes Pin_KinDyn::computeInK_Leg(
    const Eigen::Matrix3d &Rdes_FL, const Eigen::Vector3d &Pdes_FL,
    const Eigen::Matrix3d &Rdes_FR, const Eigen::Vector3d &Pdes_FR,
    const Eigen::Matrix3d &Rdes_RL, const Eigen::Vector3d &Pdes_RL,
    const Eigen::Matrix3d &Rdes_RR, const Eigen::Vector3d &Pdes_RR) {
  const pinocchio::SE3 oMdesFL(Rdes_FL, Pdes_FL);
  const pinocchio::SE3 oMdesFR(Rdes_FR, Pdes_FR);
  const pinocchio::SE3 oMdesRL(Rdes_RL, Pdes_RL);
  const pinocchio::SE3 oMdesRR(Rdes_RR, Pdes_RR);

  // qIk只有关节空间
  Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_go2_fixed_.nv);
  for (int i = 0; i < 4; i++) {
    qIk[2 + 3 * i] = -1.78;
  }

  const double eps = 1e-3;
  const int IT_MAX = 1e4;
  const double DT = 5e-2;
  const double damp = 5e-3;
  Eigen::MatrixXd J_FL = Eigen::MatrixXd::Zero(6, model_go2_fixed_.nv);
  Eigen::MatrixXd J_FR = Eigen::MatrixXd::Zero(6, model_go2_fixed_.nv);
  Eigen::MatrixXd J_RL = Eigen::MatrixXd::Zero(6, model_go2_fixed_.nv);
  Eigen::MatrixXd J_RR = Eigen::MatrixXd::Zero(6, model_go2_fixed_.nv);
  Eigen::MatrixXd JCompact = Eigen::MatrixXd::Zero(24, model_go2_fixed_.nv);

  bool success = false;
  Eigen::Matrix<double, 6, 1> errFL, errFR, errRL, errRR;
  Eigen::Matrix<double, 24, 1> errCompact;
  Eigen::VectorXd v(model_go2_fixed_.nv);
  pinocchio::FrameIndex J_Idx_FL, J_Idx_FR, J_Idx_RL, J_Idx_RR;
  J_Idx_FL = FL_foot_frame_;
  J_Idx_FR = FR_foot_frame_;
  J_Idx_RL = RL_foot_frame_;
  J_Idx_RR = RR_foot_frame_;
  int itr_count{0};
  for (itr_count = 0;; itr_count++) {
    pinocchio::forwardKinematics(model_go2_fixed_, data_go2_fixed_, qIk);
    pinocchio::updateFramePlacements(model_go2_fixed_, data_go2_fixed_);
    const pinocchio::SE3 iMdFL = data_go2_fixed_.oMf[J_Idx_FL].actInv(oMdesFL);
    const pinocchio::SE3 iMdFR = data_go2_fixed_.oMf[J_Idx_FR].actInv(oMdesFR);
    const pinocchio::SE3 iMdRL = data_go2_fixed_.oMf[J_Idx_RL].actInv(oMdesRL);
    const pinocchio::SE3 iMdRR = data_go2_fixed_.oMf[J_Idx_RR].actInv(oMdesRR);
    errFL = pinocchio::log6(iMdFL).toVector();
    errFR = pinocchio::log6(iMdFR).toVector();
    errRL = pinocchio::log6(iMdRL).toVector();
    errRR = pinocchio::log6(iMdRR).toVector();
    errCompact << errFL, errFR, errRL, errRR;
    if (errCompact.norm() < eps) {
      success = true;
      break;
    }
    if (itr_count > IT_MAX) {
      success = false;
      break;
    }

    // in joint frame
    pinocchio::computeFrameJacobian(model_go2_fixed_, data_go2_fixed_, qIk,
                                    J_Idx_FL, J_FL);
    pinocchio::computeFrameJacobian(model_go2_fixed_, data_go2_fixed_, qIk,
                                    J_Idx_FR, J_FR);
    pinocchio::computeFrameJacobian(model_go2_fixed_, data_go2_fixed_, qIk,
                                    J_Idx_RL, J_RL);
    pinocchio::computeFrameJacobian(model_go2_fixed_, data_go2_fixed_, qIk,
                                    J_Idx_RR, J_RR);

    Eigen::MatrixXd W;
    W = Eigen::MatrixXd::Identity(model_go2_fixed_.nv, model_go2_fixed_.nv);
    // J_FL.rightCols(model_go2_fixed_.nv - 3).setZero();
    // J_FR.leftCols(3).setZero();
    // J_FR.rightCols(model_go2_fixed_.nv - 6).setZero();
    // J_RL.leftCols(6).setZero();
    // J_RL.rightCols(model_go2_fixed_.nv - 9).setZero();
    // J_RR.leftCols(9).setZero();

    pinocchio::Data::Matrix6 JlogFL;
    pinocchio::Data::Matrix6 JlogFR;
    pinocchio::Data::Matrix6 JlogRL;
    pinocchio::Data::Matrix6 JlogRR;
    pinocchio::Jlog6(iMdFL.inverse(), JlogFL);
    pinocchio::Jlog6(iMdFR.inverse(), JlogFR);
    pinocchio::Jlog6(iMdRL.inverse(), JlogRL);
    pinocchio::Jlog6(iMdRR.inverse(), JlogRR);
    J_FL = -JlogFL * J_FL;
    J_FR = -JlogFR * J_FR;
    J_RL = -JlogRL * J_RL;
    J_RR = -JlogRR * J_RR;
    JCompact.block(0, 0, 6, model_go2_fixed_.nv) = J_FL;
    JCompact.block(6, 0, 6, model_go2_fixed_.nv) = J_FR;
    JCompact.block(12, 0, 6, model_go2_fixed_.nv) = J_RL;
    JCompact.block(18, 0, 6, model_go2_fixed_.nv) = J_RR;
    Eigen::Matrix<double, 24, 24> JJt;
    JJt.noalias() = JCompact * W * JCompact.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -W * JCompact.transpose() * JJt.ldlt().solve(errCompact);
    qIk = pinocchio::integrate(model_go2_fixed_, qIk, v * DT);
  }
  IkRes res;
  res.err = errCompact;
  res.itr = itr_count;

  if (success) {
    res.status = 0;
  } else {
    res.status = -1;
  }
  res.jointPosRes = qIk;

  return res;
}

Pin_KinDyn::IkRes
Pin_KinDyn::computeInK_SingleLeg(const Eigen::Matrix3d &Rdes,
                                 const Eigen::Vector3d &Pdes,
                                 const Pin_KinDyn::legIdx &leg_idx) {
  pinocchio::FrameIndex leg_foot_frame_id;
  switch (leg_idx) {
  case FL:
    leg_foot_frame_id = FL_foot_frame_;
    break;
  case FR:
    leg_foot_frame_id = FR_foot_frame_;
    break;
  case RL:
    leg_foot_frame_id = RL_foot_frame_;
    break;
  case RR:
    leg_foot_frame_id = RR_foot_frame_;
    break;
  default:
    std::cout << "error: leg index not found!" << std::endl;
    break;
  }
  const pinocchio::SE3 oMdes(Rdes, Pdes);
  Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_go2_fixed_.nv);
  for (int i = 0; i < 4; i++) {
    qIk[2 + 3 * i] = -1.78;
  }

  const double eps = 1e-4;
  const int IT_MAX = 1e3;
  const double DT = 1e-2;
  const double damp = 1e-6;
  bool success = false;
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model_go2_fixed_.nv);
  Eigen::Matrix<double, 6, 1> error;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_go2_fixed_.nv);
  const Eigen::MatrixXd W =
      Eigen::MatrixXd::Identity(model_go2_fixed_.nv, model_go2_fixed_.nv);

  int itr_count{0};
  for (itr_count = 0;; itr_count++) {
    pinocchio::forwardKinematics(model_go2_fixed_, data_go2_fixed_, qIk);
    pinocchio::updateFramePlacements(model_go2_fixed_, data_go2_fixed_);
    const pinocchio::SE3 dMi =
        oMdes.actInv(data_go2_fixed_.oMf[leg_foot_frame_id]);
    error = pinocchio::log6(dMi).toVector();
    if (error.norm() < eps) {
      success = true;
      break;
    }
    if (itr_count > IT_MAX) {
      success = false;
      break;
    }
    pinocchio::computeFrameJacobian(model_go2_fixed_, data_go2_fixed_, qIk,
                                    leg_foot_frame_id, J);
    if (!(itr_count % 10)) {
      std::cout << itr_count << std::endl;
      std::cout << "qik= " << qIk.transpose() << std::endl;
      std::cout << "J= \n" << J << std::endl;
      std::cout << "leg_foot_frame_id= " << leg_foot_frame_id << std::endl;
      std::cout << "data_go2_fixed_.oMf[leg_foot_frame_id] = \n"
                << data_go2_fixed_.oMf[leg_foot_frame_id] << std::endl;
    }
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(error);
    qIk = pinocchio::integrate(model_go2_fixed_, qIk, v * DT);
  }
  IkRes res;
  res.err = error;
  res.itr = itr_count;

  if (success) {
    res.status = 0;
  } else {
    res.status = -1;
  }
  res.jointPosRes = qIk;
  return res;
}

// Inverse Kinematics for hand posture. Note: the Rdes and Pdes are both w.r.t
// the baselink coordinate in body frame!
Pin_KinDyn::IkRes Pin_KinDyn::computeInK_Hand(const Eigen::Matrix3d &Rdes_L,
                                              const Eigen::Vector3d &Pdes_L,
                                              const Eigen::Matrix3d &Rdes_R,
                                              const Eigen::Vector3d &Pdes_R) {
  IkRes res;
  return res;
}

/////////////////////
// below are tools //
/////////////////////
// must call computeDyn() first!
// 这个函数似乎从未被调用过
void Pin_KinDyn::workspaceConstraint(Eigen::VectorXd &qFT,
                                     Eigen::VectorXd &tauJointFT) {
  for (int i = 0; i < motorName.size(); i++)
    if (qFT(i + 7) > motorMaxPos_(i)) {
      qFT(i + 7) = motorMaxPos_(i);
      motorReachLimit_[i] = true;
      tauJointFT(i) = tauJointOld_(i);
    } else if (qFT(i + 7) < motorMinPos_(i)) {
      qFT(i + 7) = motorMinPos_(i);
      motorReachLimit_[i] = true;
      tauJointFT(i) = tauJointOld_(i);
    } else
      motorReachLimit_[i] = false;

  tauJointOld_ = tauJointFT;
}

// 积分函数
Eigen::Quaterniond Pin_KinDyn::intQuat(const Eigen::Quaterniond &quat,
                                       const Eigen::Matrix<double, 3, 1> &w) {
  Eigen::Matrix3d Rcur = quat.normalized().toRotationMatrix();
  Eigen::Matrix3d Rinc = Eigen::Matrix3d::Identity();
  double theta = w.norm();
  if (theta > 1e-8) {
    Eigen::Vector3d w_norm;
    w_norm = w / theta;
    Eigen::Matrix3d a;
    a << 0, -w_norm(2), w_norm(1), w_norm(0), 0, -w_norm(0), -w_norm(1),
        w_norm(0), 0;
    Rinc =
        Eigen::Matrix3d::Identity() + a * sin(theta) + a * a * (1 - cos(theta));
  }
  Eigen::Matrix3d Rend = Rcur * Rinc;
  Eigen::Quaterniond quatRes;
  quatRes = Rend;
  return quatRes;
}

// intergrate the q with dq, for floating base dynamics
Eigen::VectorXd Pin_KinDyn::integrateDIY(const Eigen::VectorXd &qI,
                                         const Eigen::VectorXd &dqI) {
  Eigen::VectorXd qRes = Eigen::VectorXd::Zero(model_nv + 1);
  Eigen::Vector3d wDes;
  wDes << dqI(3), dqI(4), dqI(5);
  Eigen::Quaterniond quatNew, quatNow;
  quatNow.x() = qI(3);
  quatNow.y() = qI(4);
  quatNow.z() = qI(5);
  quatNow.w() = qI(6);
  quatNew = intQuat(quatNow, wDes);
  qRes = qI;
  qRes(0) += dqI(0);
  qRes(1) += dqI(1);
  qRes(2) += dqI(2);
  qRes(3) = quatNew.x();
  qRes(4) = quatNew.y();
  qRes(5) = quatNew.z();
  qRes(6) = quatNew.w();
  for (int i = 0; i < model_nv - 6; i++)
    qRes(7 + i) += dqI(6 + i);
  return qRes;
}
