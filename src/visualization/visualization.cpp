#include "visualization.h"


using namespace std;
using namespace Eigen;
 
////////////////////////////////// Show Camera ////////////////////////////////////

const int ipolygon[18] = {0,1,2,  0,3,1,  0,4,3,  0,2,4,  3,1,4,   2,4,1};

inline pcl::PointXYZ Eigen2PointXYZ(Eigen::Vector3f v) { return pcl::PointXYZ(v[0],v[1],v[2]); }
inline pcl::PointXYZRGB Eigen2PointXYZRGB(Eigen::Vector3f v, Eigen::Vector3f rgb) { pcl::PointXYZRGB p(rgb[0],rgb[1],rgb[2]); p.x = v[0]; p.y = v[1]; p.z = v[2]; return p; }
inline pcl::PointNormal Eigen2PointNormal(Eigen::Vector3f v, Eigen::Vector3f n) { pcl::PointNormal p; p.x=v[0];p.y=v[1];p.z=v[2];p.normal_x=n[0];p.normal_y=n[1];p.normal_z=n[2]; return p;}
inline float* Eigen2float6(Eigen::Vector3f v, Eigen::Vector3f rgb) { static float buf[6]; buf[0]=v[0];buf[1]=v[1];buf[2]=v[2];buf[3]=rgb[0];buf[4]=rgb[1];buf[5]=rgb[2]; return buf; }
inline Matrix<float,6,1> Eigen2Eigen(Vector3f v, Vector3f rgb) { return (Matrix<float,6,1>() << v[0],v[1],v[2],rgb[0],rgb[1],rgb[2]).finished(); }
inline std::vector<Matrix<float,6,1> > AsVector(const Matrix<float,6,1>& p1, const Matrix<float,6,1>& p2) { 	std::vector<Matrix<float,6,1> > v(2); v[0] = p1; v[1] = p2; return v; }

void visualization::visualizerShowCamera(const Matrix3f& R, const Vector3f& _t, float r, float g, float b, double s, const std::string& name) {
	std::string cam_name = name,line_name = name + "line";
	if (name.length() <= 0) {
		stringstream ss; ss<<"camera"<<ilines++;
		cam_name = ss.str();
		ss << "line";
		line_name = ss.str();
	}
	//这里如果Rt是从P阵直接取的，那么需要进行这么一行来解出实际的t的值
	//如果t已经是准确的坐标了，那么不需要进行这一步。
	//这里t已经是准确打坐标了，我就把下面注释掉了。
	//Vector3f t = -R.transpose() * _t;
	Vector3f t= _t;

	Vector3f vright = R.row(0).normalized() * s;
	Vector3f vup = -R.row(1).normalized() * s;
	Vector3f vforward = R.row(2).normalized() * s;

	Vector3f rgb(r,g,b);

	pcl::PointCloud<pcl::PointXYZRGB> mesh_cld;
	mesh_cld.push_back(Eigen2PointXYZRGB(t,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright/2.0 + vup/2.0,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright/2.0 - vup/2.0,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright/2.0 + vup/2.0,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright/2.0 - vup/2.0,rgb));

	pcl::PolygonMesh pm;
	pm.polygons.resize(6); 
	for(int i=0;i<6;i++)
		for(int _v=0;_v<3;_v++)
			pm.polygons[i].vertices.push_back(ipolygon[i*3 + _v]);
	//pcl::fromROSmsg(mesh_cld,pm.cloud);
	//pcl::fromPCLPointCloud2(mesh_cld,pm.cloud);
	pcl::toPCLPointCloud2(mesh_cld,pm.cloud);//这里更新了函数，原来的fromROSmsg不能用了，还要换一下参数位置～
ctx.lock();
	cam_meshes.push_back(std::make_pair(cam_name,pm));
	linesToShow.push_back(std::make_pair(line_name,AsVector(Eigen2Eigen(t,rgb),Eigen2Eigen(t + vforward*3.0,rgb))));
ctx.unlock();
}
void visualization::visualizerShowCamera(const float R[9], const float t[3], float r, float g, float b) {
	visualizerShowCamera(Matrix3f(R).transpose(),Vector3f(t),r,g,b);
}
void visualization::visualizerShowCamera(const float R[9], const float t[3], float r, float g, float b, double s) {
	visualizerShowCamera(Matrix3f(R).transpose(),Vector3f(t),r,g,b,s);
}
void visualization::visualizerShowCamera(const cv::Matx33f& R, const cv::Vec3f& t, float r, float g, float b, double s, const std::string& name) {
	visualizerShowCamera(Matrix<float,3,3,RowMajor>(R.val),Vector3f(t.val),r,g,b,s,name);
}
void visualization::visualizationShowPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd,const std::string& name)
{
cdx.lock();
pointcloudptr.push_back(std::make_pair(name,pcd));
cdx.unlock();
}
/////////////////////////////////////////////////////////////////////////////////
//构造与X够
visualization::visualization()
{
viewer = new pcl::visualization::PCLVisualizer("Tri viewer");
viewer->setCameraPosition(0,0,-3.0,0,-1,0);  
viewer->addCoordinateSystem(1); 
pcl::PointXYZRGB p;
p.x = 1.2f;
viewer->addText3D("X",p,0.2,1.0,0.0,0.0);
p.y = 1.2f;p.x=0;
viewer->addText3D("Y",p,0.2,0.0,1.0,0.0);
p.z=1.2f;p.y = 0;
viewer->addText3D("Z",p,0.2,0.0,0.0,1.0);
}

visualization::~visualization()
{
if(viewer!=NULL)
{
delete viewer;
}
}

//消费相机
void visualization::checkoutCamera()
{
ctx.lock();
//添加相机
for(int i = 0;i < cam_meshes.size();i++)
{
viewer->addPolygonMesh(cam_meshes[i].second,cam_meshes[i].first);
vector<Eigen::Matrix<float,6,1> > oneline = linesToShow[i].second;
pcl::PointXYZRGB A(oneline[0][3],oneline[0][4],oneline[0][5]),B(oneline[1][3],oneline[1][4],oneline[1][5]);
for(int j=0;j<3;j++) {A.data[j] = oneline[0][j]; B.data[j] = oneline[1][j];}
viewer->addLine(A,B,0.0,0.7,0.2,linesToShow[i].first);
//viewer->addText3D
}
cam_meshes.clear();
linesToShow.clear();
ctx.unlock();
}

//消费点云
void visualization::checkoutPointCloud()
{
cdx.lock();
for(int i = 0;i<pointcloudptr.size();i++)
{
viewer->addPointCloud(pointcloudptr[i].second,pointcloudptr[i].first);
}
pointcloudptr.clear();
cdx.unlock();
}
/////////////////////////////////////////////////////////////////////////////////
//plot类
plot::plot()
{
viewer = new pcl::visualization::PCLVisualizer("Plot");
//viewer->setCameraPosition(0,0,-3.0,0,-1,0);  
//viewer->addCoordinateSystem(1); 
pcl::PointXYZRGB p;
p.x = 1.2f;
viewer->addText3D("X",p,0.2,1.0,0.0,0.0);
p.y = 1.2f;p.x=0;
viewer->addText3D("Y",p,0.2,0.0,1.0,0.0);
p.z=1.2f;p.y = 0;
viewer->addText3D("Z",p,0.2,0.0,0.0,1.0);
}

plot::~plot()
{
if(viewer!=NULL)
{
delete viewer;
}
}
void plot::plotShowBar(int x,int y,double value,double value2,double r,double g,double b)
{
ctx.lock();
bar.push_back(std::make_pair(cv::Point(x,y),cv::Point2f(value,value2)));
bar_color.push_back(cv::Vec3f(r,g,b));
ctx.unlock();
}

void plot::plotShowBar(int x,int y,double value,double value2)
{
plotShowBar(x,y,value,value2,0.8,0.8,0.0);
}
void plot::checkoutBar()
{
ctx.lock();
//添加Bar
for(int i = 0;i < bar.size();i++)
{
//printf("%d,%d",bar[i].first.x,bar[i].first.y);
int x = bar[i].first.x;
int y = bar[i].first.y;
double r,g,b;
r = bar_color[i][0];
g = bar_color[i][1];
b = bar_color[i][2];
//std::cout<<"--Add Bar-x:"<<x<<" y:"<<y<<" r:"<<r<<" g:"<<g<<" b:"<<b<<std::endl;
char p[32]; 
sprintf(p,"bar%d-%d",x,y);

viewer->addCube(x*barl,x*barl+bara,y*barl,y*barl+bara,std::min(bar[i].second.x,bar[i].second.y),std::max(bar[i].second.x,bar[i].second.y),r,g,b,p);
//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,p);
pcl::PointXYZRGB pos;
pos.x = x*barl+bara*0.5;
pos.y = y*barl+bara*0.5;

pos.z = (bar[i].second.x>bar[i].second.y)?bar[i].second.y-1.0:bar[i].second.y+1.0;
sprintf(p,"%03.2f",bar[i].second.y);
char id[16];
sprintf(id,"%d-%d-v",x,y);
if(x == 0)
{
viewer->addText3D(p,pos,0.4,1.0,1.0,1.0,id);
}
else
{

viewer->addText3D(p,pos,0.4,0.7,0.7,0.7,id);
}
sprintf(id,"%d-%d-y",x,y);
//viewer->addText3D
if(x==0)
{
sprintf(p,"t = %d",y);
pos.x = -4;
pos.y = y*barl+bara*0.5;
pos.z = 0;
viewer->addText3D(p,pos,0.8,1.0,0.5,0.5,id);
}
sprintf(id,"%d-%d-x",x,y);
if(y==0)
{
sprintf(p,"Skip = %d",x);
pos.y = -4;
pos.x = x*barl+bara*0.5;
pos.z = 0;
viewer->addText3D(p,pos,0.8,0.5,1.0,0.5,id);
}
}
bar.clear();
bar_color.clear();
ctx.unlock();
}
