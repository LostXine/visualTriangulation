#include <cstdio>
#include <glog/logging.h>  
#include "camera_unit.h"

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/reader.h>
#include <rapidjson/pointer.h>

using namespace rapidjson;

double parseDouble(Document &d,const char * p,int i)
{
double r = -1;
char q[32];
sprintf(q,"/%s/%d",p,i);
if(Value* _r = Pointer(q).Get(d)){r= _r->GetDouble();};
return r;
}

//构造：构造的同时解析json
camera_unit::camera_unit(const char* js)
{
index = -1;
memset(img_path,0,sizeof(img_path));
Document d;
d.Parse(js);	
if (d.HasParseError())
{
LOG(ERROR)<<js<<" HasParseError";
return;
}
else
{
if (Value* _index = Pointer("/index").Get(d)){index = _index->GetInt();}
if (Value* _path = Pointer("/image path").Get(d)){memcpy(img_path,_path->GetString(),_path->GetStringLength());}
for(int i = 0;i<3;i++)
{
delta_R[i] = (float)parseDouble(d,"R vector",i);
delta_T[i] = (float)parseDouble(d,"T vector",i);
//abs_R[i] = 0;
//abs_T[i] = 0;
}
//清除前两个坐标
delta_R[0] = 0;
delta_R[1] = delta_R[2];
delta_R[2] = 0;
//注意这里的坐标需要转换
float tmp = delta_T[2];
delta_T[2]=delta_T[1];
delta_T[1]=tmp;
LOG(INFO)<<"Load {Index:"<<index<<" Path:"<<img_path<<" R:["<<delta_R[0]<<"|"<<delta_R[1]<<"|"<<delta_R[2]<<"] T:["<<delta_T[0]<<"|"<<delta_T[1]<<"|"<<delta_T[2]<<"]}";
}
}

//迭代上次的运动计算绝对运动值
void camera_unit::updateAbs(cv::Vec3f& last_T,cv::Matx33f& last_R)
{
abs_T = last_T+cv::Vec3f(delta_T);
cv::Matx33f delta_RM;
cv::Rodrigues(cv::Vec3f(delta_R),delta_RM);
abs_R = delta_RM*last_R;
//LOG(INFO)<<"t:"<<abs_T<<"\nR:"<<abs_R;
}

void camera_unit::initAbs()
{
cv::Vec3f t(0,0,0);
cv::Matx33f r(0,0,-1,0,1,0,1,0,0);//因为IMU的关系需要顺时针旋转90度
updateAbs(t,r);
}
