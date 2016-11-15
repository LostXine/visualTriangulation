#pragma GCC diagnostic error "-std=c++11"  

#include <tri_manager/tri_manager.h>
#include <camera_unit/camera_unit.h>
#include <glog/logging.h>  
#include <unistd.h>
#include <vector>
#include <fstream>

#define JS_BUF 1024

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
LOG(INFO) << "Build time:"<<__TIME__<<" "__DATE__;

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

//加载JSON环节
LOG(INFO)<<"---LOAD JSON---";
std::vector<camera_unit*>seq;
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
camera_unit* tcu = new camera_unit(tmp);
if(tcu->index<0){delete tcu;continue;}//检查加载是否正确
else
{
seq.push_back(tcu);
}
}

}
dt.close();

//启动PCL窗口环节

//结束了！
for(size_t t = 0;t<seq.size();t++)
{
if(seq[t]!=NULL){delete seq[t];}
}
LOG(INFO)<<"---TRIANGULATION MANAGER---";
return 0;
}
