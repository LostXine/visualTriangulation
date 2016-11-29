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
//#define HISTGRAM
#define JS_BUF 1024
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
viz.viewer->spinOnce(100);
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
#ifndef HISTGRAM
//绘制
if(tcu->isUsed){viz->visualizerShowCamera(*(tcu->getabs_R()),*(tcu->getabs_T()),250.0f,10.0f,0.0f,0.1f);}
else{viz->visualizerShowCamera(*(tcu->getabs_R()),*(tcu->getabs_T()),50.0f,200.0f,0.0f,0.1f);}
#endif
}
}
}
dt.close();
//三角化
LOG(INFO)<<"---TRIANGULATION---";
std::vector<cv::Mat> res,color;

/*
ctri.matchCamera(20,40,res,color);
LOG(INFO)<<res.rows<<" x "<<res.cols;

pcd = MatToPointXYZRGB(res,color);
LOG(INFO)<<"show PCD";
viz->visualizationShowPointCloud(pcd,"maio");
LOG(INFO)<<"show DONE";
*/
/*
ctri.matchAll(10,res,color);
for(int i = 0;i<res.size();i++)
{
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl;
pcl = MatToPointXYZRGB(res[i],color[i]);
char d[16];
sprintf(d,"cc_%d",i);
viz->visualizationShowPointCloud(pcl,d);
}
*/
int begin,end;
begin = ctri.getBegin();
end = ctri.getEnd();
int step = 1;
int sx = 0;
for(int i = begin;i+step<end-1;i++)
//for(int i = begin;i<begin+1;i++)
{
cv::Mat res,color;
double distance = ctri.matchCamera(i,i+step,res,color);
LOG(INFO)<<distance;
//LOG(INFO)<<res;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl;
pcl = MatToPointXYZRGB(res,color);
char d[16];
sprintf(d,"cc_%d",i);
#ifdef HISTGRAM
if(distance>0){
plt->plotShowBar(sx,0,distance);
}
#else
viz->visualizationShowPointCloud(pcl,d);
#endif

sx++;
}
LOG(INFO)<<"Done";
/*
std::string filename("test.pcd");  
pcl::PCDWriter writer;
writer.write(filename,*pcd);  
LOG(INFO)<<"Save done.";
*/

//结束了！
t.join();
cv::destroyAllWindows();
LOG(INFO)<<"---TRIANGULATION MANAGER---";
return 0;
}
