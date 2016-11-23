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
#include <vector>

extern "C" class camera_unit_EXPORT camera_single
{
private:
//本帧相对于上一帧的变化
float delta_R[3];
float delta_T[3];
//本帧的绝对姿态与位置
//float abs_T[3];
//float abs_R[3];
cv::Matx33f abs_R;
cv::Vec3f abs_T;
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
//获得坐标指针
cv::Vec3f*   getabs_T(){return &abs_T;}
cv::Matx33f* getabs_R(){return &abs_R;}
//提取图像特征
bool isUsed;
bool updateIsUsed(int begin,int end){isUsed = (index<end&&index>=begin);return isUsed;}
std::vector<cv::KeyPoint> kpt;//特征点
cv::Mat dep;//特征向量
cv::Mat img;//图像本体
int loadImg();//打开图像

};

extern "C" class camera_unit_EXPORT camera_tri
{
private:
std::vector<camera_single*> seq;
int begin,end;
cv::Mat K;

char root[256];
public:
camera_tri(const char* path);
~camera_tri();
camera_single* addCamera(const char* js);
};
#endif //_camera_unit_

