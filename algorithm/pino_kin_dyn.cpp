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
  pinocchio::urdf::buildModel(urdf_pathIn, root_joint, model_go2);
  pinocchio::urdf::buildModel(urdf_pathIn, model_go2_fixed);

  // 为什么有额外的fixed模型？
  // 这个fixed模型的求解结果全都是相对于body坐标系的
  data_go2 = pinocchio::Data(model_go2);
  data_go2_fixed = pinocchio::Data(model_go2_fixed);
  model_nv = model_go2.nv;

  // 变量初始化
  q.setZero();
  dq.setZero();
  ddq.setZero();
  dyn_M = Eigen::MatrixXd::Zero(model_nv, model_nv);
  dyn_M_inv = Eigen::MatrixXd::Zero(model_nv, model_nv);
  dyn_C = Eigen::MatrixXd::Zero(model_nv, model_nv);
  dyn_G = Eigen::MatrixXd::Zero(model_nv, 1);

  J_FL_foot = Eigen::MatrixXd::Zero(6, model_nv);
  J_FR_foot = Eigen::MatrixXd::Zero(6, model_nv);
  J_RL_foot = Eigen::MatrixXd::Zero(6, model_nv);
  J_RR_foot = Eigen::MatrixXd::Zero(6, model_nv);
  J_base = Eigen::MatrixXd::Zero(6, model_nv);
  J_FL_hip = Eigen::MatrixXd::Zero(6, model_nv);
  J_FR_hip = Eigen::MatrixXd::Zero(6, model_nv);
  J_RL_hip = Eigen::MatrixXd::Zero(6, model_nv);
  J_RR_hip = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_FL_foot = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_FR_foot = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_RL_foot = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_RR_foot = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_base = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_FL_hip = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_FR_hip = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_RL_hip = Eigen::MatrixXd::Zero(6, model_nv);
  dJ_RR_hip = Eigen::MatrixXd::Zero(6, model_nv);

  // get joint id
  base_joint = model_go2.getJointId("base_link");
  FL_calf_joint = model_go2.getJointId("FL_calf_joint");
  FR_calf_joint = model_go2.getJointId("FR_calf_joint");
  RL_calf_joint = model_go2.getJointId("RL_calf_joint");
  RR_calf_joint = model_go2.getJointId("RR_calf_joint");
  FL_hip_joint = model_go2.getJointId("FL_hip_joint");
  FR_hip_joint = model_go2.getJointId("FR_hip_joint");
  RL_hip_joint = model_go2.getJointId("RL_hip_joint");
  RR_hip_joint = model_go2.getJointId("RR_hip_joint");
  FL_calf_joint_fixed = model_go2_fixed.getJointId("FL_calf_joint");
  FR_calf_joint_fixed = model_go2_fixed.getJointId("FR_calf_joint");
  RL_calf_joint_fixed = model_go2_fixed.getJointId("RL_calf_joint");
  RR_calf_joint_fixed = model_go2_fixed.getJointId("RR_calf_joint");
  FL_hip_joint_fixed = model_go2_fixed.getJointId("FL_hip_joint");
  FR_hip_joint_fixed = model_go2_fixed.getJointId("FR_hip_joint");
  RL_hip_joint_fixed = model_go2_fixed.getJointId("RL_hip_joint");
  RR_hip_joint_fixed = model_go2_fixed.getJointId("RR_hip_joint");
  FL_foot_frame = model_go2.getFrameId("FL_foot");
  FR_foot_frame = model_go2.getFrameId("FR_foot");
  RL_foot_frame = model_go2.getFrameId("RL_foot");
  RR_foot_frame = model_go2.getFrameId("RR_foot");
  // read joint pvt parameters
  // 源代码中读出来全是0，可能是bug
  // 此外这些变量只出现在一个似乎从未被调用过的函数中
  // ？？
  Json::Reader reader;
  Json::Value root_read;
  std::ifstream in("joint_ctrl_config.json", std::ios::binary);
  motorMaxTorque = Eigen::VectorXd::Zero(motorName.size());
  motorMaxPos = Eigen::VectorXd::Zero(motorName.size());
  motorMinPos = Eigen::VectorXd::Zero(motorName.size());
  reader.parse(in, root_read);
  for (int i = 0; i < motorName.size(); i++) {
    motorMaxTorque(i) = (root_read[motorName[i]]["maxTorque"].asDouble());
    motorMaxPos(i) = (root_read[motorName[i]]["maxPos"].asDouble());
    motorMinPos(i) = (root_read[motorName[i]]["minPos"].asDouble());
  }
  motorReachLimit.assign(motorName.size(), false);
  tauJointOld = Eigen::VectorXd::Zero(motorName.size());
}

void Pin_KinDyn::dataBusRead(const DataBus &robotState) {
  //  For Pinocchio: The base translation part is expressed in the parent frame
  //  (here the world coordinate system) while its velocity is expressed in the
  //  body coordinate system.
  //  https://github.com/stack-of-tasks/pinocchio/issues/1137
  //  q = [global_base_position, global_base_quaternion, joint_positions]
  //  v = [local_base_velocity_linear, local_base_velocity_angular,
  //  joint_velocities]
  q = robotState.q;
  dq = robotState.dq;
  dq.block(0, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(0, 0, 3, 1);
  dq.block(3, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(3, 0, 3, 1);
  ddq = robotState.ddq;
}

void Pin_KinDyn::dataBusWrite(DataBus &robotState) {
  // Jacobian and their dot
  robotState.J_FL_foot = J_FL_foot;
  robotState.J_FR_foot = J_FR_foot;
  robotState.J_RL_foot = J_RL_foot;
  robotState.J_RR_foot = J_RR_foot;
  robotState.J_base = J_base;
  robotState.J_FL_hip = J_FL_hip;
  robotState.J_FR_hip = J_FR_hip;
  robotState.J_RL_hip = J_RL_hip;
  robotState.J_RR_hip = J_RR_hip;
  robotState.dJ_FL_foot = dJ_FL_foot;
  robotState.dJ_FR_foot = dJ_FR_foot;
  robotState.dJ_RL_foot = dJ_RL_foot;
  robotState.dJ_RR_foot = dJ_RR_foot;
  robotState.dJ_base = dJ_base;

  // workspace linear pos and velocity
  robotState.FL_foot_pos_W = FL_foot_pos_W;
  robotState.FR_foot_pos_W = FR_foot_pos_W;
  robotState.RL_foot_pos_W = RL_foot_pos_W;
  robotState.RR_foot_pos_W = RR_foot_pos_W;
  robotState.FL_foot_pos_L = FL_foot_pos_L;
  robotState.FR_foot_pos_L = FR_foot_pos_L;
  robotState.RL_foot_pos_L = RL_foot_pos_L;
  robotState.RR_foot_pos_L = RR_foot_pos_L;
  robotState.FL_foot_vel_L = FL_foot_vel_L;
  robotState.FR_foot_vel_L = FR_foot_vel_L;
  robotState.RL_foot_vel_L = RL_foot_vel_L;
  robotState.RR_foot_vel_L = RR_foot_vel_L;
  robotState.FL_hip_pos_W = FL_hip_pos_W;
  robotState.FR_hip_pos_W = FR_hip_pos_W;
  robotState.RL_hip_pos_W = RL_hip_pos_W;
  robotState.RR_hip_pos_W = RR_hip_pos_W;
  robotState.FL_hip_pos_L = FL_hip_pos_L;
  robotState.FR_hip_pos_L = FR_hip_pos_L;
  robotState.RL_hip_pos_L = RL_hip_pos_L;
  robotState.RR_hip_pos_L = RR_hip_pos_L;

  // workspace rotation
  robotState.FL_foot_rot_W = FL_foot_rot_W;
  robotState.FR_foot_rot_W = FR_foot_rot_W;
  robotState.RL_foot_rot_W = RL_foot_rot_W;
  robotState.RR_foot_rot_W = RR_foot_rot_W;
  robotState.FL_foot_rot_L = FL_foot_rot_L;
  robotState.FR_foot_rot_L = FR_foot_rot_L;
  robotState.RL_foot_rot_L = RL_foot_rot_L;
  robotState.RR_foot_rot_L = RR_foot_rot_L;
  robotState.FL_hip_rot_W = FL_hip_rot_W;
  robotState.FR_hip_rot_W = FR_hip_rot_W;
  robotState.RL_hip_rot_W = RL_hip_rot_W;
  robotState.RR_hip_rot_W = RR_hip_rot_W;

  // dyn
  robotState.dyn_M = dyn_M;
  robotState.dyn_M_inv = dyn_M_inv;
  robotState.dyn_C = dyn_C;
  robotState.dyn_G = dyn_G;
  robotState.dyn_Ag = dyn_Ag;
  robotState.dyn_dAg = dyn_dAg;
  robotState.dyn_Non = dyn_Non;

  // com
  robotState.pCoM_W = CoM_pos;
  robotState.Jcom_W = Jcom;

  robotState.inertia = inertia;
}

// update jacobians and joint positions
void Pin_KinDyn::computeJ_dJ() {
  pinocchio::forwardKinematics(model_go2, data_go2, q);
  pinocchio::jacobianCenterOfMass(model_go2, data_go2, q, true);
  pinocchio::computeJointJacobians(model_go2, data_go2, q);
  pinocchio::computeJointJacobiansTimeVariation(model_go2, data_go2, q, dq);

  
  pinocchio::updateGlobalPlacements(model_go2, data_go2);

  pinocchio::getFrameJacobian(model_go2, data_go2, FL_foot_frame,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_FL_foot);
  pinocchio::getFrameJacobian(model_go2, data_go2, FR_foot_frame,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_FR_foot);
  pinocchio::getFrameJacobian(model_go2, data_go2, RL_foot_frame,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_RL_foot);
  pinocchio::getFrameJacobian(model_go2, data_go2, RR_foot_frame,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_RR_foot);
  pinocchio::getJointJacobian(model_go2, data_go2, FL_hip_joint,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_FL_hip);
  pinocchio::getJointJacobian(model_go2, data_go2, FR_hip_joint,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_FR_hip);
  pinocchio::getJointJacobian(model_go2, data_go2, RL_hip_joint,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_RL_hip);
  pinocchio::getJointJacobian(model_go2, data_go2, RR_hip_joint,
                              pinocchio::LOCAL_WORLD_ALIGNED, J_RR_hip);
  pinocchio::getJointJacobianTimeVariation(model_go2, data_go2, FL_hip_joint,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_FL_hip);
  pinocchio::getJointJacobianTimeVariation(model_go2, data_go2, FR_hip_joint,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_FR_hip);
  pinocchio::getJointJacobianTimeVariation(model_go2, data_go2, RL_hip_joint,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_RL_hip);
  pinocchio::getJointJacobianTimeVariation(model_go2, data_go2, RR_hip_joint,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_RR_hip);
  pinocchio::getFrameJacobianTimeVariation(model_go2, data_go2, FL_foot_frame,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_FL_foot);
  pinocchio::getFrameJacobianTimeVariation(model_go2, data_go2, FR_foot_frame,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_FR_foot);
  pinocchio::getFrameJacobianTimeVariation(model_go2, data_go2, RL_foot_frame,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_RL_foot);
  pinocchio::getFrameJacobianTimeVariation(model_go2, data_go2, RR_foot_frame,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ_RR_foot);
  pinocchio::getJointJacobianTimeVariation(
      model_go2, data_go2, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_base);

  FL_foot_pos_W = data_go2.oMf[FL_foot_frame].translation();
  FR_foot_pos_W = data_go2.oMf[FR_foot_frame].translation();
  RL_foot_pos_W = data_go2.oMf[RL_foot_frame].translation();
  RR_foot_pos_W = data_go2.oMf[RR_foot_frame].translation();
  FL_foot_rot_W = data_go2.oMf[FL_foot_frame].rotation();
  FR_foot_rot_W = data_go2.oMf[FR_foot_frame].rotation();
  RL_foot_rot_W = data_go2.oMf[RL_foot_frame].rotation();
  RR_foot_rot_W = data_go2.oMf[RR_foot_frame].rotation();
  RR_hip_pos_W = data_go2.oMi[RR_hip_joint].translation();
  RL_hip_pos_W = data_go2.oMi[RL_hip_joint].translation();
  FR_hip_pos_W = data_go2.oMi[FR_hip_joint].translation();
  FL_hip_pos_W = data_go2.oMi[FL_hip_joint].translation();
  RR_hip_rot_W = data_go2.oMi[RR_hip_joint].rotation();
  RL_hip_rot_W = data_go2.oMi[RL_hip_joint].rotation();
  FR_hip_rot_W = data_go2.oMi[FR_hip_joint].rotation();
  FL_hip_rot_W = data_go2.oMi[FL_hip_joint].rotation();
  base_pos = data_go2.oMi[base_joint].translation();
  base_rot = data_go2.oMi[base_joint].rotation();
  Jcom = data_go2.Jcom;
  std::cout << "trans" << std::endl;
  // trans to world frame
  Eigen::MatrixXd Mpj;
  Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
  Mpj.block(0, 0, 3, 3) = base_rot.transpose();
  Mpj.block(3, 3, 3, 3) = base_rot.transpose();
  J_base = J_base * Mpj;
  J_FL_foot = J_FL_foot * Mpj;
  J_FR_foot = J_FR_foot * Mpj;
  J_RL_foot = J_RL_foot * Mpj;
  J_RR_foot = J_RR_foot * Mpj;
  J_FL_hip = J_FL_hip * Mpj;
  J_FR_hip = J_FR_hip * Mpj;
  J_RL_hip = J_RL_hip * Mpj;
  J_RR_hip = J_RR_hip * Mpj;
  Jcom = Jcom * Mpj;
  dJ_base = dJ_base * Mpj;
  dJ_FL_foot = dJ_FL_foot * Mpj;
  dJ_FR_foot = dJ_FR_foot * Mpj;
  dJ_RL_foot = dJ_RL_foot * Mpj;
  dJ_RR_foot = dJ_RR_foot * Mpj;
  dJ_FL_hip = dJ_FL_hip * Mpj;
  dJ_FR_hip = dJ_FR_hip * Mpj;
  dJ_RL_hip = dJ_RL_hip * Mpj;
  dJ_RR_hip = dJ_RR_hip * Mpj;
  dJ_base = dJ_base * Mpj;
  std::cout << "calculate body frame var" << std::endl;
  // calculate body frame var
  Eigen::VectorXd q_fixed;
  q_fixed = q.block(7, 0, model_go2_fixed.nv, 1);
  pinocchio::forwardKinematics(model_go2_fixed, data_go2_fixed, q_fixed);
  pinocchio::updateGlobalPlacements(model_go2_fixed, data_go2_fixed);
  FL_foot_pos_L = data_go2_fixed.oMf[FL_foot_frame].translation();
  FR_foot_pos_L = data_go2_fixed.oMf[FR_foot_frame].translation();
  RL_foot_pos_L = data_go2_fixed.oMf[RL_foot_frame].translation();
  RR_foot_pos_L = data_go2_fixed.oMf[RR_foot_frame].translation();
  FL_foot_rot_L = data_go2_fixed.oMf[FL_foot_frame].rotation();
  FR_foot_rot_L = data_go2_fixed.oMf[FR_foot_frame].rotation();
  RL_foot_rot_L = data_go2_fixed.oMf[RL_foot_frame].rotation();
  RR_foot_rot_L = data_go2_fixed.oMf[RR_foot_frame].rotation();
  RR_hip_pos_L = data_go2_fixed.oMi[RR_hip_joint_fixed].translation();
  RL_hip_pos_L = data_go2_fixed.oMi[RL_hip_joint_fixed].translation();
  FR_hip_pos_L = data_go2_fixed.oMi[FR_hip_joint_fixed].translation();
  FL_hip_pos_L = data_go2_fixed.oMi[FL_hip_joint_fixed].translation();
}

// update dynamic parameters, M*ddq+C*dq+G=tau
void Pin_KinDyn::computeDyn() {
  // cal M

  // cal Minv

  // cal C

  // cal G

  // cal Ag, Centroidal Momentum Matrix. First three rows: linear, other three
  // rows: angular

  // cal nonlinear item

  // cal I

  // cal CoM
}

// Inverse kinematics for leg posture. Note: the Rdes and Pdes are both w.r.t
// the baselink coordinate in body frame!
Pin_KinDyn::IkRes Pin_KinDyn::computeInK_Leg(const Eigen::Matrix3d &Rdes_L,
                                             const Eigen::Vector3d &Pdes_L,
                                             const Eigen::Matrix3d &Rdes_R,
                                             const Eigen::Vector3d &Pdes_R) {
  IkRes res;
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
    if (qFT(i + 7) > motorMaxPos(i)) {
      qFT(i + 7) = motorMaxPos(i);
      motorReachLimit[i] = true;
      tauJointFT(i) = tauJointOld(i);
    } else if (qFT(i + 7) < motorMinPos(i)) {
      qFT(i + 7) = motorMinPos(i);
      motorReachLimit[i] = true;
      tauJointFT(i) = tauJointOld(i);
    } else
      motorReachLimit[i] = false;

  tauJointOld = tauJointFT;
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
