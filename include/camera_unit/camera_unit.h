#ifndef _camera_unit_
#define _camera_unit_
#if defined(_MSC_VER)||defined(__CYGWIN__)||defined(__MINGW32__)
#	ifdef camera_unit_EXPORTS	//这个名字需要和项目名称camera_unit相符合，否则会报C4273
#		define camera_unit_EXPORT __declspec(dllexport)
#	else
#		define camera_unit_EXPORT __declspec(dllimport)
#	endif
#else
#	define camera_unit_EXPORT
#endif

#include <opencv2/opencv.hpp>

#ifdef USE_GPU_SIFT
// SiftGPU模块
#include <SiftGPU.h>
#include <stddef.h>
#include <vector>
#endif
extern "C" class camera_unit_EXPORT camera_single
{
private:
//位置信息
int b_box[4];
//本帧相对于上一帧的变化
float delta_R[3];
float delta_T[3];
//本帧的绝对姿态与位置
//float abs_T[3];
//float abs_R[3];
cv::Matx33f abs_R;
cv::Vec3f abs_T;
cv::Mat P;
//图像路径
char img_path[256];
//JSON指针
char* chjs;
public:
int index;//图像的序号
camera_single(const char* js,const char* path);
//更新绝对位置
void updateAbs(cv::Vec3f& last_T,cv::Matx33f& last_R);
void initAbs();//第一帧初始化
cv::Mat getCameraMat(cv::Mat K);
//获得坐标指针
cv::Mat getP(){return P;};
cv::Vec3f*   getabs_T(){return &abs_T;}
cv::Matx33f* getabs_R(){return &abs_R;}
//提取图像特征
bool isUsed;
bool updateIsUsed(int begin,int end){isUsed = (index<end&&index>=begin);return isUsed;}
#ifdef USE_GPU_SIFT
std::vector<SiftGPU::SiftKeypoint> kpts;//特征点
std::vector<float> desp;//特征向量
#else
std::vector<cv::KeyPoint> kpts;
cv::Mat desp;
#endif
cv::Mat img;//图像本体
cv::Mat img_color;//彩色图像本体
int loadImg();//打开图像
int drawKeypoints();//绘制特征点图像
int* getBBox(){return b_box;}
};

extern "C" class camera_unit_EXPORT camera_tri
{
struct lrmatch
{
int left;
int right;
std::vector<cv::DMatch> match;
lrmatch(int l,int r)
{
left = l;
right = r;
}
};
private:
std::vector<camera_single*> seq;
std::vector<lrmatch> mh;
int begin,end;
cv::Mat K;
#ifdef USE_GPU_SIFT
SiftGPU *sift;
#else
cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("FAST");
cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create("ORB");
cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
#endif
char root[256];
public:
camera_tri(const char* path);
~camera_tri();
camera_single* addCamera(const char* js);
double matchCamera(int l,int r,cv::Mat& result,cv::Mat& color);
int drawMatch(int l,int r);
void matchAll(int step,std::vector<cv::Mat>& res,std::vector<cv::Mat> &color);
int getBegin(){return begin;}
int getEnd(){return end;}
};
#endif //_camera_unit_

