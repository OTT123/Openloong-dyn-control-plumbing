#include "tools.h"

void PrintVecMat(const char *name, const Eigen::MatrixXd mat) {
  if (mat.cols() == 1) {
    std::cout << YELLOW << name << " = [ ";
    std::cout << LIGHT_CYAN << mat.transpose();
    std::cout << YELLOW << " ];" << std::endl;
  } else {
    std::cout << YELLOW << name << " = [ \n";
    std::cout << LIGHT_CYAN << mat;
    std::cout << YELLOW << " ];" << NONE << std::endl;
  }
}