#include <tri_manager/tri_manager.h>
#include <camera_unit/camera_unit.h>
#include <visualization/visualization.h>
#include <glog/logging.h>  
#include <unistd.h>
#include <vector>
#include <fstream>
#include <thread>
#include <pcl/io/pcd_io.h>

//#include <opencv2/gpu/gpu.hpp>
//#define DRAW_PCL
#define HISTGRAM
#define JS_BUF 1024

double dabs(double i){return i<0?-i:i;}
//计算伪彩色
cv::Vec3f calColor(double in)
{
float gray = std::min(dabs(in)/(double)MAX_ERROR,1.0);
float r,g,b;
r = gray;
g = (gray<0.5)? gray*2:(1.0-gray)*2;
b = 1.0-gray;
return cv::Vec3f (r,g,b);
}
//绘制线程
void update_PCL(visualization** _viz)
{
LOG(INFO)<<"--PCL Thread online--";
visualization viz;
*_viz = &viz;//传递指针出去
//LOG(INFO)<<"PCL_VIS:"<<(long int)(*_viz);
while(!(viz.viewer->wasStopped())) 
{
viz.checkoutCamera();
viz.checkoutPointCloud();
viz.viewer->spinOnce(100);
} 
_viz = NULL;
LOG(INFO)<<"--PCL Thread done--";
}

//绘制图
void update_PLOT(plot** _viz)
{
LOG(INFO)<<"--PLOT Thread online--";
plot viz;
*_viz = &viz;//传递指针出去
//LOG(INFO)<<"PCL_VIS:"<<(long int)(*_viz);
while(!(viz.viewer->wasStopped())) 
{
viz.checkoutBar();
viz.viewer->spinOnce(1000);
} 
_viz = NULL;
LOG(INFO)<<"--PLOT Thread done--";
}
//MAT->PCL
pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatToPointXYZRGB(cv::Mat OpencVPointCloud,cv::Mat color)
         {
             /*
             *  Function: Get from a Mat to pcl pointcloud datatype
             *  In: cv::Matqq
             */

             pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::PointXYZRGB>);

             for(int i=0;i<OpencVPointCloud.cols;i++)
             {
                //std::cout<<i<<endl;

                pcl::PointXYZRGB point;
		float ratio = OpencVPointCloud.at<float>(3,i);
                point.x = OpencVPointCloud.at<float>(0,i)/ratio;
                point.y = OpencVPointCloud.at<float>(1,i)/ratio;
                point.z = OpencVPointCloud.at<float>(2,i)/ratio;
//LOG(INFO)<<"x:"<<point.x<<" y:"<<point.y<<" z:"<<point.z<<" d:"<<ratio;

                // when color needs to be added:
                uint32_t rgb = (static_cast<uint32_t>(color.at<char>(0,i)) << 16 | static_cast<uint32_t>(color.at<char>(1,i)) << 8 | static_cast<uint32_t>(color.at<char>(2,i)));
                point.rgb = *reinterpret_cast<float*>(&rgb);
/*		if(point.x*point.x+point.y*point.y + point.z*point.z<1e4)
{*/
                point_cloud_ptr -> points.push_back(point);
/*}
*/
             }
point_cloud_ptr->is_dense=false;
             point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
//printf("Point num:%d\n",point_cloud_ptr->width);
             point_cloud_ptr->height = 1;

             return point_cloud_ptr;

         }




int main(int argc, char** argv)
{
/*测试代码*/
/*
    tri_manager obj;
    obj.sayhello("world");
    obj.testOpenCV("path");
*/
//准备日志环节
google::InitGoogleLogging(argv[0]); 
FLAGS_colorlogtostderr = true;	//按颜色显示日志
FLAGS_logbufsecs = 0;		//不需要日志缓冲时间
FLAGS_stderrthreshold = 0;	//设置输出INFO级别的信息
char logdir[256] = {0};
sprintf(logdir,"%s-log/Tri-",argv[0]);
google::SetLogDestination(google::GLOG_INFO,logdir);	//设置输出目录

LOG(INFO) << "---TRIANGULATION MANAGER---";
LOG(INFO) << "Build time:"<< __TIME__ <<" "<< __DATE__;

//打印参数环节：
LOG(INFO)<<"---PRINT PARA---";
for(int i = 0;i<argc;i++)
{
LOG(INFO)<<"Para "<<i<<": "<<argv[i];
}
if(argc<2)
{
LOG(WARNING)<<"Usage: ./bin/main <img_dir>";
LOG(INFO)<<"Program return 1";
return 1;
}

#ifdef  DRAW_PCL
//启动PCL窗口环节
LOG(INFO)<<"---START VISUALIZATION---";

#ifdef HISTGRAM
plot* plt = 0;
std::thread t(update_PLOT, &plt);
while(plt==0){usleep(1000);}//等待指针赋值
#else
visualization* viz =0;
std::thread t(update_PCL, &viz);
while(viz==0){usleep(1000);}//等待指针赋值
#endif
#endif
//LOG(INFO)<<"MAIN_VIS:"<<(long int)viz;
LOG(INFO)<<"---VISUALIZATION ONLINE---";

//加载JSON环节
LOG(INFO)<<"---LOAD INFO---";
camera_tri ctri(argv[1]);//特征提取模块
char dt_dir[256] = {0};
sprintf(dt_dir,"%s/detection.json",argv[1]);
std::fstream dt;
dt.open(dt_dir,std::ios::in);
if(!dt.is_open())
{
LOG(ERROR)<<"Can't open "<<dt_dir;
LOG(INFO)<<"Program return 2";
return 2;
}
else
{
char tmp[JS_BUF]={0};
while(dt.getline(tmp,JS_BUF))
{
//检测注释！
int p;
for(p = 0;p<JS_BUF;p++)
{
if(tmp[p]=='#'||tmp[p]==0)
{
tmp[p]=0;break;//截断字符串，进入解析环节
}
}
if(p<2){continue;}
camera_single* tcu = ctri.addCamera(tmp);
if(tcu==0){continue;}//检查加载是否正确
else
{
#ifdef  DRAW_PCL
#ifndef HISTGRAM
//绘制
if(tcu->isUsed){viz->visualizerShowCamera(*(tcu->getabs_R()),*(tcu->getabs_T()),250.0f,10.0f,0.0f,0.1f);}
else{viz->visualizerShowCamera(*(tcu->getabs_R()),*(tcu->getabs_T()),50.0f,200.0f,0.0f,0.1f);}
#endif
#endif
}
}
}
dt.close();
//三角化
LOG(INFO)<<"---TRIANGULATION---";
std::vector<cv::Mat> res,color;

int begin,end;
begin = ctri.getBegin();
end = ctri.getEnd();

std::vector<double>gndth,err,err_min;
std::vector<int>err_c,err_idx;
for(int i = 0;i<(end-begin);i++)
{
double p = ctri.seq[i+begin]->groundtruth;
p  =sqrt(p*p+25);//标牌高5m
#ifdef DRAW_PCL
#ifdef HISTGRAM
plt->plotShowBar(0,i,0,p);
#endif
#endif
gndth.push_back(p);
}

std::vector<std::vector<double>>ft;
int step = std::min(end-begin,30);
LOG(INFO)<<"STEP:"<<step;
int sx = 1,sy = 0;
ft.resize(end-begin);
err.resize(step);
err_c.resize(step);
err_min.resize(step);
err_idx.resize(step);
for(int i = 0;i<err.size();i++){err[i]=0;err_c[i]=0;err_min[i] = 1e5;err_idx[i] = -1;}//清零误差和
for(int i = begin;i<end;i++)
//注意！这里是index坐标！
//for(int i = begin;i<begin+1;i++)
{
for(int j = 1;j<=end-i && j<=step;j++)
{
LOG(INFO)<<"---index:"<<i<<"---step:"<<j<<"---";
cv::Mat res,color;
double distance = ctri.matchCamera(i-1,i+j-1,res,color);
ft[i-begin].push_back(distance);
LOG(INFO)<<distance;
//LOG(INFO)<<res;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl;
//LOG(INFO)<<"M0";
if(res.cols<1){continue;}
//LOG(INFO)<<"M1";
pcl = MatToPointXYZRGB(res,color);
char d[32];
sprintf(d,"cc_%d_%d",i,j);
//LOG(INFO)<<"M2";

if(distance>0)
{
//LOG(INFO)<<"M3";
//LOG(INFO)<<"M4";
double err_this =distance-gndth[i-begin];
err[j-1]+= dabs(err_this);
if(dabs(err_this)<err_min[j-1])
{
//LOG(WARNING)<<"Abs:"<<abs(err_this)<<" no Abs:"<<err_this;
LOG(WARNING)<<"Image:"<<i<<" Step:"<<j<<" min_err from "<<err_min[j-1]<<" to "<<dabs(err_this);
err_min[j-1] = dabs(err_this);err_idx[j-1] = i;
}
err_c[j-1]++;
#ifdef  DRAW_PCL
#ifdef HISTGRAM
cv::Vec3f cc = calColor(err_this);
LOG(INFO)<<"Draw-sx:"<<sx<<" sy:"<<sy<<" v2:"<<err_this<<cc;
//plt->plotShowBar(sx,sy,gndth[i-begin],distance,cc[0],cc[1],cc[2]);
plt->plotShowBar(sx,sy,0.0,err_this,cc[0],cc[1],cc[2]);
#endif
#endif

}
#ifdef  DRAW_PCL
#ifndef HISTGRAM
viz->visualizationShowPointCloud(pcl,d);
#endif
#endif

sx++;
}
sx = 1;
sy++;
}

LOG(INFO)<<"Done";
//计算平均值
for(int i = 0;i<err.size();i++)
{
err[i] = err[i]/err_c[i];
}
LOG(INFO)<<"ABS ERR";
FILE *outfile;
sprintf(dt_dir,"%s/output_distance.txt",argv[1]);
outfile = fopen(dt_dir,"w");
fprintf(outfile,"Format\nIndex|mean_error|min_error|min_error_index|min_error_location\n");
for(int i = 0;i<err.size();i++){
printf("%03d | %0.3f | %0.3f | %03d | %0.3f\n",i+1,err[i],err_min[i],err_idx[i],gndth[err_idx[i]-1-begin]);
fprintf(outfile,"%03d | %0.3f | %.3f | %03d | %0.3f\n",i+1,err[i],err_min[i],err_idx[i],gndth[err_idx[i]-1-begin]);
};
fclose(outfile);
LOG(INFO)<<"TREE";
for(int i = 0;i<ft.size();i++)
{
printf("%03d|%.1f|",ctri.getBegin()+i,gndth[i]);
for(int j =0;j<ft[i].size();j++)
{
printf("%.1f ",ft[i][j]);
}
printf("\n");
}

LOG(INFO)<<"ALL DONE";
/*
std::string filename("test.pcd");  
pcl::PCDWriter writer;
writer.write(filename,*pcd);  
LOG(INFO)<<"Save done.";
*/

//结束了！
#ifdef DRAW_PCL
t.join();
#endif
cv::destroyAllWindows();
LOG(INFO)<<"---TRIANGULATION MANAGER---";
return 0;
}
