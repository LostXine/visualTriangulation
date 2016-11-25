#include <cstdio>
#include <glog/logging.h>  
#include "camera_unit.h"

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/reader.h>
#include <rapidjson/pointer.h>
#include <thread>

#ifdef USE_GPU_SIFT
// OpenGL
#include <GL/gl.h>
#endif
using namespace rapidjson;

//计算距离
double getDistance(cv::Point2f a,cv::Point2f b)
{
double dx,dy;
dx = a.x-b.x;
dy = b.y-a.y;
return dx*dx+dy*dy;
}


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
img_color = cv::imread(img_path);
cv::cvtColor(img_color,img,CV_RGB2GRAY);
//cv::resize(tmp,img,cv::Size(1024,1024));
return !(img.cols>0 && img.rows>0);
}

int camera_single::drawKeypoints()
{
cv::Mat draw = img_color.clone();
for(int i = 0;i<kpts.size();i++)
{
cv::circle(draw,kpts[i].pt,2,cv::Scalar(255,0,255));
}
cv::imshow("img",draw);
cv::waitKey(0);
}

cv::Mat camera_single::getCameraMat(cv::Mat K)
{
cv::Mat rc = -cv::Mat(abs_R)*cv::Mat(abs_T);
P = cv::Mat(3,4,CV_64FC1);
for(int i = 0;i<3;i++)
{
for(int j = 0;j<3;j++)
{
P.at<double>(i,j) = (double)abs_R(i,j);
}
P.at<double>(i,3) = (double)rc.at<float>(i,0);
}
P = K*P;
return P;

}

camera_tri::camera_tri(const char* path)
{
LOG(INFO)<<"Camera Triangulation Manager";
#ifdef USE_GPU_SIFT
sift = new SiftGPU;
char* myargv[4] ={ "-fo", "-1", "-v", "1"};
    sift->ParseParam(4, myargv);

    //检查硬件是否支持SiftGPU
    int support = sift->CreateContextGL();
    if ( support != SiftGPU::SIFTGPU_FULL_SUPPORTED )
    {
        LOG(ERROR)<<"SiftGPU is not supported!";
    }
#endif
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
#ifdef USE_GPU_SIFT
if(sift!=NULL){delete sift;}
#endif
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
tcu->getCameraMat(K);//更新摄像机矩阵 
seq.push_back(tcu);//输入队列
//tcu->calculateKpt();//检测特征点
tcu->loadImg();//读取图像

static bool gomatch = false;

if(tcu->updateIsUsed(begin,end))//检查是否在范围内
{


#ifdef USE_GPU_SIFT
int width = tcu->img.cols;
int height = tcu->img.rows;
LOG(INFO)<<"W:"<<width<<" H:"<<height;
//提取特征点
if(sift->RunSIFT(width, height, tcu->img.data, GL_INTENSITY8, GL_UNSIGNED_BYTE))
{
int num = sift->GetFeatureNum();
LOG(INFO)<<"Found SIFT num:"<<num;
tcu->kpts.resize(num);tcu->desp.resize(128*num);
sift->GetFeatureVector(&(tcu->kpts[0]),&(tcu->desp[0]));
LOG(INFO)<<"Extract Feature Vector Done";
}
#else
detector->detect(tcu->img, tcu->kpts);
descriptor->compute(tcu->img, tcu->kpts,tcu->desp);
LOG(INFO)<<"Found FAST num:"<<tcu->kpts.size();
//tcu->drawKeypoints();
#endif
/*
if(gomatch)
{
int p = seq.size();
matchCamera(p-2,p-1);
}
gomatch = true;
*/
}

return tcu;
}

int camera_tri::matchCamera(int l,int r,cv::Mat& result,cv::Mat& color)
{
if(l>=seq.size() || l<0 || r<0 || r>seq.size())
{
LOG(ERROR)<<"matchCamera() Wrong index L:"<<l <<" R:"<<r;
return -1;
}
if(seq[l]->kpts.size()==0||seq[l]->kpts.size()==0)
{
LOG(ERROR)<<"matchCamera() empty keypoints";
return -2;
}
std::vector<cv::DMatch>dm;
mh.push_back(lrmatch(l,r));
matcher->match(seq[l]->desp,seq[r]->desp, dm);
LOG(INFO)<<"Index "<<l<<" and "<<r<<" found "<<dm.size()<<" pairs";
//匹配完毕，开始对齐
std::vector<cv::Point2f>lp,rp;
for(int i = 0;i<dm.size();i++)
{
//(mh.end()-1)->match.push_back(dm[i]);
lp.push_back(seq[l]->kpts[dm[i].queryIdx].pt);
rp.push_back(seq[r]->kpts[dm[i].trainIdx].pt);
}
cv::Mat mask;
cv::findFundamentalMat(lp,rp,CV_FM_RANSAC,3,0.99,mask);
//滤除错配点
//std::cout<<mask;
for(int p = 0;p<dm.size();p++)
{
if(mask.at<char>(p,0) == 1)
{
if(getDistance(seq[l]->kpts[dm[p].queryIdx].pt,seq[r]->kpts[dm[p].trainIdx].pt)<1e5)
{
(mh.end()-1)->match.push_back(dm[p]);
}
}
}
LOG(INFO)<<"Index "<<l<<" and "<<r<<" found ("<<(mh.end()-1)->match.size()<<"/"<<dm.size()<<") pairs";
//显示匹配
drawMatch(l,r);
//重新对齐特征点
lp.clear();rp.clear();

for(int i = 0;i<(mh.end()-1)->match.size();i++)
{
//(mh.end()-1)->match.push_back(dm[i]);
lp.push_back(seq[l]->kpts[(mh.end()-1)->match[i].queryIdx].pt);
rp.push_back(seq[r]->kpts[(mh.end()-1)->match[i].trainIdx].pt);
}

LOG(INFO)<<lp.size()<<" "<<rp.size();
LOG(INFO)<<seq[l]->getP()<<" "<<seq[r]->getP();
cv::triangulatePoints(seq[l]->getP(),seq[r]->getP(),lp,rp,result);
color = cv::Mat(3,lp.size(),CV_8UC1);//获取颜色
for(int i = 0;i<color.cols;i++)
{
color.at<char>(0,i) = seq[l]->img_color.at<cv::Vec3b>(lp[i])[0];
color.at<char>(1,i) = seq[l]->img_color.at<cv::Vec3b>(lp[i])[1];
color.at<char>(2,i) = seq[l]->img_color.at<cv::Vec3b>(lp[i])[2];
}

}


int camera_tri::drawMatch(int l,int r)
{
for(int i = 0;i<mh.size();i++)
{
if(mh[i].left==l && mh[i].right==r)
{
cv::Mat cm;
std::vector<cv::Mat>chs;
chs.push_back(seq[l]->img);
chs.push_back(seq[l]->img);
chs.push_back(seq[r]->img);
cv::merge(chs,cm);
for(int b = 0;b<mh[i].match.size();b++)
{
cv::line(cm,seq[l]->kpts[mh[i].match[b].queryIdx].pt,seq[r]->kpts[mh[i].match[b].trainIdx].pt,cv::Scalar(0,255,0));
}

//cv::drawMatches(seq[l]->img_color,seq[l]->kpts,seq[r]->img_color,seq[r]->kpts,mh[i].match,cm);
cv::imshow("matches",cm);
cv::waitKey(0);
return 0;
}
}
return -1;
}




