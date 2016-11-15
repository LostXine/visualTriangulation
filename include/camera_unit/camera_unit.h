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
 
extern "C" class camera_unit_EXPORT camera_unit
{
private:
//本帧相对于上一帧的变化
double delta_R[3];
double delta_T[3];
//本帧的绝对姿态与位置
double abs_T[3];
double abs_R[3];
//图像路径
char img_path[256];
//JSON指针
char* chjs;
public:
int index;//图像的序号
camera_unit(const char* js);

};

#endif //_camera_unit_

