// Standalone numerical sanity check: no ROS, TF, robot or simulator involved.
#include <cmath>
#include <iostream>
#include <xtensor/xtensor.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xeval.hpp>

int main()
{
  xt::xtensor<float, 2> yaw = xt::zeros<float>({2, 8});
  auto cosine = xt::eval(xt::cos(yaw));
  auto sine = xt::eval(xt::sin(yaw));
  std::cout << "scalar cos(0)=" << std::cos(0.0f)
    << " tensor cos(0)=" << cosine(0,1) << " tensor sin(0)=" << sine(0,1) << '\n';
  for (size_t i=0; i<2; ++i) {
    for (size_t j=0; j<8; ++j) {
      if (std::abs(cosine(i,j)-1.0f)>1e-6f || std::abs(sine(i,j))>1e-6f) {return 1;}
    }
  }
  return 0;
}
