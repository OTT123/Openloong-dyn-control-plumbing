#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
int main(int argc, char const *argv[]) {
  std::string urdf_pathIn = "../models/go2/go2_description.urdf";
  pinocchio::Model model_go2_fixed_;
  pinocchio::Data data_go2_, data_go2_fixed_;
  pinocchio::urdf::buildModel(urdf_pathIn, model_go2_fixed_);
  data_go2_fixed_ = pinocchio::Data(model_go2_fixed_);
  pinocchio::JointIndex FL_foot_joint_ =
      model_go2_fixed_.getJointId("FL_foot_joint");
  pinocchio::JointIndex FL_calf_joint_ =
      model_go2_fixed_.getJointId("FL_calf_joint");
  pinocchio::FrameIndex FL_foot_frame_ = model_go2_fixed_.getFrameId("FL_foot");

  Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_go2_fixed_.nv);
  for (int i = 0; i < 4; i++) {
    qIk[2 + 3 * i] = -1.78;
  }
  pinocchio::forwardKinematics(model_go2_fixed_, data_go2_fixed_, qIk);
  pinocchio::updateGlobalPlacements(model_go2_fixed_, data_go2_fixed_);
  // pinocchio::updateFramePlacements(model_go2_fixed_, data_go2_fixed_);
  std::cout<<"FL_foot_joint_= "<<FL_foot_joint_<<std::endl;
  std::cout<<"FL_calf_joint_= "<<FL_foot_joint_<<std::endl;
  std::cout<<"FL_foot_frame_= "<<FL_foot_frame_<<std::endl;
  std::cout << "qik= " << qIk.transpose() << std::endl;
  std::cout << "FL_foot_joint_ oMi = \n"
            << data_go2_fixed_.oMi[FL_foot_joint_] << std::endl;
  std::cout << "FL_foot_joint_ oMf= \n"
            << data_go2_fixed_.oMf[FL_foot_joint_] << std::endl;
  std::cout << "FL_foot_frame_ oMi = \n"
            << data_go2_fixed_.oMi[FL_foot_frame_] << std::endl;
  std::cout << "FL_foot_frame_ oMf = \n"
            << data_go2_fixed_.oMf[FL_foot_frame_] << std::endl;
  std::cout << "FL_calf_joint_ oMi = \n"
            << data_go2_fixed_.oMi[FL_calf_joint_] << std::endl;
  std::cout << "FL_calf_joint_ oMf = \n"
            << data_go2_fixed_.oMf[FL_calf_joint_] << std::endl;
  return 0;
}
