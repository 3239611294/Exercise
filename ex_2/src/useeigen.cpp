#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
int main(int argc, char** argv)
{
	//定义100*100矩阵
    MatrixXd matrix100_100=MatrixXd::Random(100,100);
	//定义100*1向量
	VectorXd vector100_1=MatrixXd::Random(100,1);
	//QR求解
	VectorXd vector100_1_2=matrix100_100.colPivHouseholderQr().solve(vector100_1);
	cout<<vector100_1_2<<endl;
	return 0;
	

}
