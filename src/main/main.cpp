#include <tri_manager/tri_manager.h>
#include <camera_unit/camera_unit.h>
#include <visualization/visualization.h>
#include <glog/logging.h>  
#include <unistd.h>
#include <vector>
#include <fstream>
#include <thread>
//#include <opencv2/gpu/gpu.hpp>

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
viz.viewer->spinOnce(20);
} 
_viz = NULL;
LOG(INFO)<<"--PCL Thread done--";
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
visualization* viz =0;
std::thread t(update_PCL, &viz);
while(viz==0){usleep(1000);}//等待指针赋值
//LOG(INFO)<<"MAIN_VIS:"<<(long int)viz;
LOG(INFO)<<"---VISUALIZATION ONLINE---";

//加载JSON环节
LOG(INFO)<<"---LOAD INFO---";
camera_tri ctri(argv[1]);//特征提取模块
char dt_dir[256] = {0};
sprintf(dt_dir,"%s/data.json",argv[1]);
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
//调试加载慢一点
//usleep(1e4);


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
//绘制
if(tcu->isUsed){viz->visualizerShowCamera(*(tcu->getabs_R()),*(tcu->getabs_T()),250.0f,10.0f,0.0f,0.1f);}
else{viz->visualizerShowCamera(*(tcu->getabs_R()),*(tcu->getabs_T()),50.0f,200.0f,0.0f,0.1f);}
}
}
}
dt.close();
cv::destroyAllWindows();


//结束了！
t.join();
LOG(INFO)<<"---TRIANGULATION MANAGER---";
return 0;
}
