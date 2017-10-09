#include <cstdio>
#include <glog/logging.h> 
#include <math.h> 
#include "camera_unit.h"

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/reader.h>
#include <rapidjson/pointer.h>
#include <thread>

#include<ctime>
#include<cstdlib>

#ifdef USE_GPU_SIFT
// OpenGL
#include <GL/gl.h>
#endif
using namespace rapidjson;

int min (int a,int b){return a>b?b:a;}
int max (int a,int b){return a>b?a:b;}

//getCommonBox
void getCommonBox(int* box1,int* box2,int *result)
{
	for(int i = 0;i<2;i++)
	{
		result[i] = min(box1[i],box2[i]) - 20;
		result[i+2] = max(box1[i+2],box2[i+2]) + 20;
	}
	LOG(INFO)<<"x1:"<<result[0]<<" y1:"<<result[1]<<" x2:"<<result[2]<<" y2:"<<result[3];
}

//Vec3f/double
cv::Vec3f cmod(cv::Vec3f p,double b)
{
	float q[3];
	for(int i = 0;i<3;i++)
	{
		q[i] = p[i]/b;
	}
	return cv::Vec3f(q);
}

double getDistance(cv::Vec3f a,cv::Vec3f b)
{
	cv::Vec3f p = a-b;
	double res = 0;
	for(int i = 0;i<3;i++)
	{
		res +=p[i]*p[i];
	}
	return sqrt(res);
}
//计算距离
double getDistance(cv::Point2f a,cv::Point2f b)
{
	double dx,dy;
	dx = a.x-b.x;
	dy = b.y-a.y;
	return dx*dx+dy*dy;
}

//判断在球内
bool isIn(cv::Vec3f a,cv::Vec3b b,double radiu)
{
	return getDistance(a,b)<radiu;
}

//生成模型
cv::Vec3f genModel(std::vector<cv::Vec3f> l)
{
	cv::Vec3f ret(0,0,0);
	for(int i = 0;i<l.size();i++)
	{
		ret = ret + l[i];
	}

	return cmod(ret,l.size());
}

//IN/OUT/OUT/IN/IN
int refilterPoints(cv::Mat result,std::vector<int>& mask,cv::Vec3f& ref,double radiu,double rate = 0.7)
{
	double thre = -rate;
	std::vector<int> index;
	int inv = 0;
	std::vector<cv::Vec3f> plist;
	for(int i = 0;i<result.cols;i++)
	{
		plist.push_back(cv::Vec3f(result.at<float>(0,i)/result.at<float>(3,i),result.at<float>(1,i)/result.at<float>(3,i),result.at<float>(2,i)/result.at<float>(3,i)));
	}

	while(thre<rate && inv<5000)
	{
		int m_size = 5;
//候选模型
		std::vector<cv::Vec3f> m_list;
		for(int i = 0;i<m_size;i++)
		{
			m_list.push_back(plist[rand()%plist.size()]);
		}
		cv::Vec3f m_model = genModel(m_list);

		std::vector<int> m_index;
//判断模型
		for(int i = 0;i<plist.size();i++)
		{
			if(isIn(m_model,plist[i],radiu))
			{
				m_index.push_back(i);
			}
		}
//结算模型
		double m_thre = ((double)m_index.size())/plist.size();

		if(m_thre>thre)
		{
			LOG(INFO)<<"INV:"<<inv<<" From "<<thre<<" to "<<m_thre;
			thre = m_thre;//移交最大值
			index = m_index;
		}
		inv++;
	}
//移交模板
	mask = index;
	std::vector<cv::Vec3f> flist;
	for(int i = 0;i<index.size();i++)
	{
		flist.push_back(plist[index[i]]);
	}
	ref= genModel(flist);
//计算距离
	if(thre>rate){return 0;}
	else{return 1;}
}




int parseInt(Document& d,const char* p,int i)
{
	int r = -1;
	char q[32];
	sprintf(q,"/%s/%d",p,i);
	if(Value* _r = Pointer(q).Get(d)){r= _r->GetInt();};
	return r;
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
	for(int i = 0;i<4;i++){b_box[i]=-1;}
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
		if (Value* _gt = Pointer("/rtk distance").Get(d))
		{
			groundtruth = _gt->GetDouble();
		}
		else
		{
			groundtruth = -1;
		}
		for(int i = 0;i<3;i++)
		{
			delta_R[i] = (float)parseDouble(d,"R vector",i);
			delta_T[i] = (float)parseDouble(d,"T vector",i);
//abs_R[i] = 0;
//abs_T[i] = 0;
		}
		for(int i = 0;i<4;i++)
		{
			b_box[i] = parseInt(d,"box",i);
		}

//清除前两个坐标
		delta_R[0] = 0;
		delta_R[1] = delta_R[2];
		delta_R[2] = 0;
//注意这里的坐标需要转换
		delta_T[2] = delta_T[0];
		delta_T[0] = delta_T[1];
		delta_T[1] = 0;
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
//cv::Matx33f r(0,0,-1,0,1,0,1,0,0);//因为IMU的关系需要顺时针旋转90度
	cv::Matx33f r(1,0,0,0,1,0,0,0,1);
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
	if(b_box[0]>0)
	{
		cv::rectangle(draw,cv::Point(b_box[0],b_box[1]),cv::Point(b_box[2],b_box[3]),cv::Scalar(255,255,0),2);
	}
	cv::imshow("img",draw);
	cv::waitKey(0);
	cv::destroyWindow("img");
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
	try
	{
		fs["K"]>>K;
		LOG(INFO)<<"Load camera matrix:"<<std::endl<<K;
	#ifdef DISTORTION_CORR
		fs["dist"]>>distortion;
		LOG(INFO)<<"Load camera distortion:"<<std::endl<<distortion;
	#endif
		cv::Mat range;
		fs["range"]>>range;
		begin = range.at<double>(0,0);
		end = range.at<double>(1,0);
		LOG(INFO)<<"Begin:"<<begin<<" End:"<<end;
	}
	catch(cv::Exception e)
	{
		LOG(INFO)<<e.what();
	}
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
	if(tcu->index<0){return 0;}//加载失败
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

	if(tcu->updateIsUsed(begin,end))//检查是否在范围内
	{

#ifdef DISTORTION_CORR
	//Distortion correction
		tcu->img_before_distortion = tcu->img.clone();
		cv::undistort(tcu->img_before_distortion,tcu->img,K,distortion);
	//display distorion
	/*
	std::vector<cv::Mat> chs;
	cv::Mat comp;
	chs.push_back(tcu->img_before_distortion.clone());
	chs.push_back(tcu->img.clone());
	chs.push_back(tcu->img.clone());
	cv::merge(chs,comp);
	cv::imshow("distortion",comp);
	cv::waitKey();*/
#endif
	//display done.
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
		if(*(tcu->getBBox())>0)
		{
			int *bx = tcu->getBBox();
			cv::Mat mk = cv::Mat::zeros(tcu->img.rows,tcu->img.cols,CV_8UC1);
			cv::rectangle(mk,cv::Point(*bx,*(bx+1)),cv::Point(*(bx+2),*(bx+3)),cv::Scalar(255,255,255),CV_FILLED);
/*
cv::imshow("mask",mk);
cv::waitKey(0);
cv::destroyWindow("mask");
*/
			detector->detect(tcu->img,tcu->kpts,mk);
		}
		else{detector->detect(tcu->img, tcu->kpts);}
		descriptor->compute(tcu->img, tcu->kpts,tcu->desp);
		LOG(INFO)<<"Found FAST num:"<<tcu->kpts.size();
//tcu->drawKeypoints();
#endif

	}

	return tcu;
}

double camera_tri::matchCamera(int l,int r,cv::Mat& result,cv::Mat& color)
{
	if(l>=seq.size() || l<0 || r<0 || r>seq.size())
	{
		LOG(ERROR)<<"matchCamera() Wrong index L:"<<l <<" R:"<<r;
		return -1;
	}
	if(seq[l]->kpts.size()==0||seq[r]->kpts.size()==0)
	{
		LOG(ERROR)<<"matchCamera() empty keypoints";
		return -2;
	}
	LOG(INFO)<<"Left:"<<l <<" have "<<seq[l]->kpts.size()<<" pts.";
	LOG(INFO)<<"Right:"<<r <<" have "<<seq[r]->kpts.size()<<" pts.";
	std::vector<cv::DMatch>rough_dmatch;
	mh.push_back(lrmatch(l,r));
	std::vector<lrmatch>::iterator this_match = mh.end()-1;
	matcher->match(seq[l]->desp,seq[r]->desp, rough_dmatch);
	LOG(INFO)<<"Index "<<l<<" and "<<r<<" found "<<rough_dmatch.size()<<" pairs";
	
	std::vector<cv::Point2f>point_in_left,point_in_right;
	std::vector<int>index_vec_in_rough_dmatch;
	if(rough_dmatch.size()>10)
	{
		//input all the match
		for(int i = 0;i<rough_dmatch.size();i++)
		{
			this_match->out_match.push_back(rough_dmatch[i]);
		}

//匹配完毕，开始对齐
		for(int i = 0;i<rough_dmatch.size();i++)
		{
			point_in_left.push_back(seq[l]->kpts[rough_dmatch[i].queryIdx].pt);
			point_in_right.push_back(seq[r]->kpts[rough_dmatch[i].trainIdx].pt);
		}
		cv::Mat mask;
		cv::findFundamentalMat(point_in_left,point_in_right,CV_FM_RANSAC,1,0.99,mask);
//滤除错配点
//std::cout<<mask;
		for(int p = 0;p<rough_dmatch.size();p++)
		{
			if(mask.at<char>(p,0) == 1)
			{
				if(getDistance(seq[l]->kpts[rough_dmatch[p].queryIdx].pt,seq[r]->kpts[rough_dmatch[p].trainIdx].pt)<1e5)
				{
					this_match->match.push_back(rough_dmatch[p]);
					index_vec_in_rough_dmatch.push_back(p);
				}
			}
			
		}
	}
	else
	{
		LOG(WARNING)<<"rough_dmatch is too small to use fundamentalMat";
		return -5;
		/*
		for(int p = 0;p<rough_dmatch.size();p++)
		{
			this_match->match.push_back(rough_dmatch[p]);
		}*/
	}
	LOG(INFO)<<"Index "<<l<<" and "<<r<<" found ("<<this_match->match.size()<<"/"<<rough_dmatch.size()<<") pairs";
	if(this_match->match.size()<1){LOG(WARNING)<<"No matched pairs.";return -3;}
//重新对齐特征点
	point_in_left.clear();point_in_right.clear();

	for(int i = 0;i<this_match->match.size();i++)
	{
//(mh.end()-1)->match.push_back(dm[i]);
		point_in_left.push_back(seq[l]->kpts[this_match->match[i].queryIdx].pt);
		point_in_right.push_back(seq[r]->kpts[this_match->match[i].trainIdx].pt);
	}

//LOG(INFO)<<point_in_left.size()<<" "<<point_in_right.size();
//LOG(INFO)<<seq[l]->getP()<<" "<<seq[r]->getP();
	cv::Mat result_of_triangulate_points;
	cv::triangulatePoints(seq[l]->getP(),seq[r]->getP(),point_in_left,point_in_right,result_of_triangulate_points);
//根据距离再滤一遍
	std::vector<int> remain_idx_after_distance_ransac;
	cv::Vec3f distance_after_distance_ransac;//最终距离
	double distance_return = -1;

	if(!refilterPoints(result_of_triangulate_points,remain_idx_after_distance_ransac,distance_after_distance_ransac,15,0.6))
	{
		LOG(INFO)<<"After RANSAC: ("<<remain_idx_after_distance_ransac.size()<<"/"<<result_of_triangulate_points.cols<<")";
		distance_return = getDistance(*(seq[l]->getabs_T()),distance_after_distance_ransac);
		cv::Mat cloudpoint_result_tmp (4,remain_idx_after_distance_ransac.size(),CV_32FC1);
		point_in_left.clear();
		point_in_right.clear();
		this_match->match.clear();//清除原有匹配
		for(int i = 0;i<remain_idx_after_distance_ransac.size();i++)
		{
			int idx_in_triangulate_points = remain_idx_after_distance_ransac[i];
	//写入显示
			//cloudpoints
			for(int j = 0;j<4;j++)
			{
				cloudpoint_result_tmp.at<float>(j,i) = result_of_triangulate_points.at<float>(j,idx_in_triangulate_points);
			}
			result = cloudpoint_result_tmp.clone();

			//feature points
			cv::DMatch* dmatch_tmp = &(rough_dmatch[index_vec_in_rough_dmatch[idx_in_triangulate_points]]);
			this_match->match.push_back(*dmatch_tmp);//推入新的匹配
			point_in_left.push_back(seq[l]->kpts[dmatch_tmp->queryIdx].pt);
			point_in_right.push_back(seq[r]->kpts[dmatch_tmp->trainIdx].pt);
		}

	}
	else
	{
	//没有成功
		LOG(INFO)<<"Fail to RANSAC.";
		return -4;
		/*
		std::vector<cv::Vec3f>m_list;
		for(int i = 0;i<result_of_triangulate_points.cols;i++)
		{
			m_list.push_back(cv::Vec3f(result_of_triangulate_points.at<float>(0,i)/result_of_triangulate_points.at<float>(3,i),result_of_triangulate_points.at<float>(1,i)/result_of_triangulate_points.at<float>(3,i),result_of_triangulate_points.at<float>(2,i)/result_of_triangulate_points.at<float>(3,i)));
		}
		distance_after_distance_ransac = genModel(m_list);
		distance_return = getDistance(*(seq[l]->getabs_T()),distance_after_distance_ransac);
		result = result_of_triangulate_points.clone();
		*/
		
	}


	//get Color from origin pic.
	color = cv::Mat(3,result_of_triangulate_points.cols,CV_8UC1);//获取颜色
	for(int i = 0;i<color.cols;i++)
	{
		color.at<char>(0,i) = seq[l]->img_color.at<cv::Vec3b>(point_in_left[i])[2];
		color.at<char>(1,i) = seq[l]->img_color.at<cv::Vec3b>(point_in_left[i])[1];
		color.at<char>(2,i) = seq[l]->img_color.at<cv::Vec3b>(point_in_left[i])[0];
	}

	//drawMatch(l,r);
	return distance_return;
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

			for(int b = 0;b<seq[l]->kpts.size();b++)
			{
				cv::circle(cm,seq[l]->kpts[b].pt,3,cv::Scalar(255,0,128));
			}
			for(int b = 0;b<seq[r]->kpts.size();b++)
			{
				cv::circle(cm,seq[r]->kpts[b].pt,3,cv::Scalar(255,128,0));
			}

			for(int b = 0;b<mh[i].out_match.size();b++)
			{
				cv::line(cm,seq[l]->kpts[mh[i].out_match[b].queryIdx].pt,seq[r]->kpts[mh[i].out_match[b].trainIdx].pt,cv::Scalar(0,0,255));
			}
			for(int b = 0;b<mh[i].match.size();b++)
			{
				cv::line(cm,seq[l]->kpts[mh[i].match[b].queryIdx].pt,seq[r]->kpts[mh[i].match[b].trainIdx].pt,cv::Scalar(0,255,0));
			}
			int common_box[4];
			getCommonBox(seq[l]->getBBox(),seq[r]->getBBox(),common_box);
			cv::Rect roi_region(cv::Point(common_box[0],common_box[1]),cv::Point(common_box[2],common_box[3]));
			cv::imshow("matches",cm(roi_region));
			cv::waitKey(0);

			return 0;
		}
	}
	return -1;
}

void camera_tri::matchAll(int step,std::vector<cv::Mat>& res,std::vector<cv::Mat>&color)
{
	int s = step;
	if(step<1){s = 1;}
	LOG(INFO)<<"Matching inv:"<<s;
	for(int i = begin;i+s<end;i+=s)
	{
		cv::Mat _res,_color;

		matchCamera(i,s+i,_res,_color);

		res.push_back(_res);
		color.push_back(_color);
	}

}
