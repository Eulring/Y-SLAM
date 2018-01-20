#include "Yslam/vodometry.h"

namespace Yslam
{
//############################################################################//
//############################################################################//
//############################################################################//
//
void VO::generatePoint()
{
	triangulation();
	matchFilterByCam();
	//point3d_[cur_id_-1]=point3d_cur_;

	int match_num=0;
	double scale=0;

	for(int i=0;i<matches_.size();i++)
	{
		//printf("match:  %d\n",i);
		int ref_id=matches_[i].trainIdx;
		int cur_id=matches_[i].queryIdx;

		int classid=kpt_ref_[ref_id].class_id;
		int psize=points_.size();

		//cout<<"class_id  -> :"<<classid<<endl;
		//cout<<"points_size:  -> : "<<psize<<endl;
		if(ref_id>kpt_ref_.size()||cur_id>kpt_cur_.size()) continue;
		if(classid>psize) continue;

		if(cur_id_==1||classid==-1)
		{
			//cout<<" new point added !!!"<<endl;
			MapPoint mpt=MapPoint();
			mpt.pos_=point3d_cur_[i];
			points_.push_back(mpt);
			//cout<<"size->:  "<<points_.size()<<endl;
			int idtmp=points_.size()-1;
			kpt_ref_[ref_id].class_id=idtmp;
			kpt_cur_[cur_id].class_id=idtmp;
		}
		else
		{
			//if(i==409) cout<<"here 1";
			match_num++;
			kpt_cur_[cur_id].class_id=kpt_ref_[ref_id].class_id;
			//cout<<"--1--"<<endl;   cout<<kpt_ref_[ref_id].class_id<<endl;
			Point3d ref_pos=points_[kpt_ref_[ref_id].class_id].pos_;
			//cout<<"--2--"<<endl;
			double sscale=getScale(ref_pos,point3d_cur_[i]);
			//cout<<"--2--"<<endl;
			//cout<<"Scale: "<<sscale<<endl;
			if(sscale>10) sscale=10;
			scale+=sscale;
		}

		points_[kpt_cur_[cur_id].class_id].ob_num_++;
	}


	if(match_num>0)
	{
		updateScale(scale/match_num);
		relocate();
	}
	cout<<"Match_num: "<<match_num<<endl;
}

void VO::relocate()
{
	triangulation();
	matchFilterByCam();

	for(int i=0;i<matches_.size();i++)
	{
		//printf("match:  %d\n",i);
		int ref_id=matches_[i].trainIdx;

		if(ref_id>kpt_ref_.size()) continue;

		int pid=kpt_ref_[ref_id].class_id;
		int psize=points_.size();
		if(pid>=psize) continue;
		points_[pid].pos_=point3d_cur_[i];
		//cout<<kpt_ref_[ref_id].class_id<<endl;
		//cout<<kpt_cur_[cur_id].class_id<<endl;
	}
}

//############################################################################//
//############################################################################//
//############################################################################//
//
void VO::updateScale(double scale)
{
	frame_cur_.t_delta_=frame_cur_.t_delta_*scale;
	int cur_id=frames_.size()-1;
	frame_cur_.t_=frames_[cur_id].t_+frames_[cur_id].R_*frame_cur_.t_delta_;
}

double VO::getScale(Point3d p,Point3d p_)
{
	int cur_id=frames_.size()-1;
	Mat R0=frames_[cur_id].R_delta_;
	Mat t0=frames_[cur_id].t_delta_;

	Mat mp=tool_.p3d2mat(p);
	//cout<<R0<<endl;
	R0=R0.inv();
	mp=(mp-t0);
	mp=R0*mp;

	p=tool_.mat2p3d(mp);

	return norm(p)/norm(p_);
}


void VO::triangulation()
{
	point3d_cur_.clear();
	//Eigen::Matrix3d R1,R2;
	//Vector3d t1,t2;
	int cur_id=frames_.size()-1;
	Mat R1,R2,t1,t2;
	R1=frames_[cur_id].R_;
	t1=frames_[cur_id].t_;
	R2=frame_cur_.R_;
	t2=frame_cur_.t_;
	Mat T1=Mat(3,4,CV_64F);
	Mat T2=Mat(3,4,CV_64F);


	for(int i=0;i<3;i++) for(int j=0;j<4;j++)
	{
		if(j==3)
		{
			T1.at<double>(i,j)=t1.at<double>(i,0);
			T2.at<double>(i,j)=t2.at<double>(i,0);
		}
		else
		{
			T1.at<double>(i,j)=R1.at<double>(i,j);
			T2.at<double>(i,j)=R2.at<double>(i,j);
		}
	}


	Mat K(3,3,CV_64F);
	K=cam_.K_;


	vector<Point2d> cpt_1;
	vector<Point2d> cpt_2;

	cv::Point2f fp1,fp2;
	cv::Point2d tmp;

	for(int i=0;i<matches_.size();i++)
	{
		fp1=kpt_ref_[matches_[i].queryIdx].pt;
		fp2=kpt_cur_[matches_[i].trainIdx].pt;

		tmp=tool_.pixel2cam(Point2d(fp1.x,fp1.y),K);
		cpt_1.push_back(tmp); //cout<<tmp<<endl;
		tmp=tool_.pixel2cam(Point2d(fp2.x,fp2.y),K);
		cpt_2.push_back(tmp); //cout<<tmp<<endl;
	}


	Mat pts_4d;
	triangulatePoints( T1, T2, cpt_1, cpt_2, pts_4d );

	for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i); //cout<<x<<endl<<endl;
        x /= x.at<double>(3,0);
        Point3d p (
            x.at<double>(0,0),
            x.at<double>(1,0),
            x.at<double>(2,0));
        point3d_cur_.push_back( p );

    }

}

//############################################################################//
//############################################################################//
//############################################################################//
//
void VO::matchFilterByCam()
{
	cout<<"start ft_cam: "<<matches_.size()<<"->";
	int cur_id=frames_.size()-1;
	Point3d cam_pos=tool_.mat2p3d(frames_[cur_id].t_);

	double lim=25;

	for(int i=matches_.size();i>0;i--)
	{
		Point3d x=point3d_cur_[i];
		if(norm(x-cam_pos)>lim||x.z<0)
		{
			matches_.erase(matches_.begin()+i);
			point3d_cur_.erase(point3d_cur_.begin()+i);
		}
	}
	cout<<matches_.size()<<endl;
}

void VO::matchFilterByPt3()
{
	cout<<"start ft_pt3: "<<matches_.size()<<"->";

	Point3d center(0,0,0);

	for(int i=matches_.size()-1;i>=0;i--)
		center+=point3d_cur_[i];
	center.x/=matches_.size();
	center.y/=matches_.size();
	center.z/=matches_.size();

	vector<double> dis;
	double tot=0;

	for(int i=0;i<matches_.size();i++)
	{
		dis.push_back(norm(center-point3d_cur_[i]));
		tot+=dis[i];
	}
	tot/=matches_.size();  tot*=1;

	for(int i=matches_.size()-1;i>=0;i--)
	{
		if(dis[i]>tot)
		{
			matches_.erase(matches_.begin()+i);
			point3d_cur_.erase(point3d_cur_.begin()+i);
		}
	}

	cout<<matches_.size()<<endl;
	cout<<point3d_cur_.size()<<endl;
}



void VO::matchFilterByDis()
{
	cout<<"start ft_dis: "<<matches_.size()<<"->";
	vector<double> dis,dis2;
	cv::Point2f a,b;
	double dis_mid=0;
	for(int i=0;i<matches_.size();i++)
	{
		a=kpt_ref_[matches_[i].queryIdx].pt;
		b=kpt_cur_[matches_[i].trainIdx].pt;
		dis.push_back(norm(a-b));
		dis2.push_back(norm(a-b));
	}
	sort(dis2.begin(),dis2.end());
	dis_mid=dis2[matches_.size()/2];
	for(int i=matches_.size()-1;i>=0;i--)
	{
		if(abs(dis[i]-dis_mid)>dis_mid/2)
		 	matches_.erase(matches_.begin()+i);
	}
	cout<<matches_.size()<<endl;
}

}
