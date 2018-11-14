#include <iostream>
using namespace std;
//Add Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

int main(int argc, char** argv)
{
  //旋转平移1
  Quaterniond q_1,q_2;
  Vector3d t_1(0.7,1.1,0.2),t_2(-0.11,0.4,0.8),p_1(0.5,-0.1,0.2);
  q_1.coeffs()<<0.3,0.2,0.2,0.55;
  q_2.coeffs()<<0.3,-0.7,0.2,-0.1;

  Isometry3d T1=Isometry3d::Identity();
  Isometry3d T2=Isometry3d::Identity();
  
  T1.rotate(q_1.normalized());
  T1.pretranslate(t_1);
  T2.rotate(q_2.normalized());
  T2.pretranslate(t_2);
  
  //世界坐标系下坐标
  Vector3d p_w=T1.inverse()*p_1;


  cout<<T2*p_w<<endl;
  return 0;
}
