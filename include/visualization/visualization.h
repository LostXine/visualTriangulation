#ifndef _visualization_
#define _visualization_
#if defined(_MSC_VER)||defined(__CYGWIN__)||defined(__MINGW32__)
#	ifdef visualization_EXPORTS	//这个名字需要和项目名称visualization相符合，否则会报C4273
#		define visualization_EXPORT __declspec(dllexport)
#	else
#		define visualization_EXPORT __declspec(dllimport)
#	endif
#else
#	define visualization_EXPORT
#endif

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <mutex>
using namespace Eigen;

extern "C" class visualization_EXPORT visualization
{
private:
int ilines = 0;
std::mutex ctx;
std::deque<std::pair<std::string,pcl::PolygonMesh> > cam_meshes;
std::deque<std::pair<std::string,std::vector<Matrix<float,6,1> > > > linesToShow;
public:
pcl::visualization::PCLVisualizer* viewer;
void visualizerShowCamera(const float R[9], const float t[3], float r, float g, float b);
void visualizerShowCamera(const float R[9], const float t[3], float r, float g, float b, double s);
void visualizerShowCamera(const cv::Matx33f& R, const cv::Vec3f& t, float r, float g, float b, double s, const std::string& name = "");
void visualizerShowCamera(const Matrix3f& R, const Vector3f& _t, float r, float g, float b, double s = 0.1 /*downscale factor*/, const std::string& name = "");
void checkoutCamera();

visualization();
~visualization();
};

#endif //_visualization_
