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
camera_single::camera_single(const char* js,const char* path)
{
isUsed = false;
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
if (Value* _path = Pointer("/image path").Get(d))
{
sprintf(img_path,"%s%s",path,_path->GetString());
//memcpy(img_path,_path->GetString(),_path->GetStringLength());
}
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
void camera_single::updateAbs(cv::Vec3f& last_T,cv::Matx33f& last_R)
{
abs_T = last_T+cv::Vec3f(delta_T);
cv::Matx33f delta_RM;
cv::Rodrigues(cv::Vec3f(delta_R),delta_RM);
abs_R = delta_RM*last_R;
//LOG(INFO)<<"t:"<<abs_T<<"\nR:"<<abs_R;
}

void camera_single::initAbs()
{
cv::Vec3f t(0,0,0);
cv::Matx33f r(0,0,-1,0,1,0,1,0,0);//因为IMU的关系需要顺时针旋转90度
updateAbs(t,r);
}

int camera_single::loadImg()//打开图像
{
LOG(INFO)<<"Load image:"<<img_path;
//cv::Mat tmp
cv::Mat p = cv::imread(img_path);
cv::Mat s;
cv::resize(p,s,cv::Size(640,480));
cv::imshow("img",s);
cv::waitKey(50);
cv::cvtColor(p,img,CV_RGB2GRAY);
//cv::resize(tmp,img,cv::Size(1024,1024));
return !(img.cols>0 && img.rows>0);
}

camera_tri::camera_tri(const char* path)
{
LOG(INFO)<<"Camera Triangulation Manager";
sprintf(root,"%s/",path);//定义数据目录
LOG(INFO)<<"Root:"<<root;

char data[256];
sprintf(data,"%s/info.xml",path);
cv::FileStorage fs;
fs.open(data,cv::FileStorage::READ);
if(fs.isOpened())
{
fs["K"]>>K;
LOG(INFO)<<"Load camera matrix:"<<std::endl<<K;
cv::Mat range;
fs["range"]>>range;
begin = range.at<double>(0,0);
end = range.at<double>(1,0);
LOG(INFO)<<"Begin:"<<begin<<" End:"<<end;
fs.release();
}
else
{
LOG(INFO)<<"Fail to open:"<<data;
begin = -1;
end = -1;
}
}

camera_tri::~camera_tri()
{
for(size_t t = 0;t<seq.size();t++)
{
if(seq[t]!=NULL){delete seq[t];}
}
}


camera_single* camera_tri::addCamera(const char* js)
{
camera_single* tcu = new camera_single(js,root);
if(tcu->index<=0){return 0;}//加载失败
int idx = seq.size();
if(idx > 0)
{
tcu->updateAbs(*(seq[idx-1]->getabs_T()),*(seq[idx-1]->getabs_R()));//更新坐标
}
else
{
//第一帧要初始化坐标
tcu->initAbs();
}
tcu->updateIsUsed(begin,end);//检查是否在范围内
seq.push_back(tcu);//输入队列
//tcu->calculateKpt();//检测特征点
tcu->loadImg();//读取图像
return tcu;
}





