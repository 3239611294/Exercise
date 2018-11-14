#include <iostream>
using namespace std;
#include "math.h"
//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

//Sophus
#include "sophus/se3.h"
using namespace Sophus;

//Pangolin
#include "pangolin/pangolin.h"
typedef Matrix<double,6,1> Vector6d;
// path to trajectory file
string trajectory_file_1 = "../groundtruth.txt";
string trajectory_file_2 = "../estimated.txt";
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void LoadTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &pose, string file);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_1;
	vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_2;
    /// implement pose reading code
	LoadTrajectory(poses_1,trajectory_file_1);
	LoadTrajectory(poses_2,trajectory_file_2);
	
	//RMSE
	double RMSE=0;
	double e;
	Vector6d a;
	for(size_t i=0;i<poses_1.size();i++)
	{
	  a=(poses_1[i].inverse()*poses_2[i]).log();
	  e=a.transpose()*a;
	  RMSE += e;
	}
	
	cout<<"The RMSE is : "<<sqrt(RMSE/double(poses_1.size()))<<endl;
    // draw trajectory in pangolin
    DrawTrajectory(poses_1,poses_2);
    return 0;
}
/*******************************************************************************************/
void LoadTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &pose, string files)
{
	ifstream file;
	file.open(files.c_str());
	while(!file.eof())
	{
	  string s;
	  getline(file,s);
	  if(!s.empty())
	  {
		stringstream ss;
		ss<<s;
		Vector3d t;
		Quaterniond q;
		double time;
		ss>>time>>t[0]>>t[1]>>t[2]>>q.coeffs()[0]>>q.coeffs()[1]>>q.coeffs()[2]>>q.coeffs()[3];
		Sophus::SE3 pose1(q,t);
		pose.push_back(pose1);
	  }
	}
}


/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1) {
    if (poses.empty()||poses1.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t j = 0; j < poses1.size() - 1; j++) {
            glColor3f(1 - (float) j / poses1.size(), 0.0f, (float) j / poses1.size());
            glBegin(GL_LINES);
            auto p1 = poses1[j], p2 = poses1[j + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
		}
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}