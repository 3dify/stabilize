#include <node.h>
#include <v8.h>
#include <nan.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp> 
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include "binding.h"

using namespace cv;
using namespace std;
using namespace v8;

void Method(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = Isolate::GetCurrent();
  HandleScope scope(isolate);
  args.GetReturnValue().Set(v8::String::NewFromUtf8(isolate, "world"));
}

NAN_METHOD(ComputeKalman) {
  NanEscapableScope();
  // extract quad args
  Local<Array> srcArray = Local<Array>::Cast(args[0]->ToObject());
  Local<Array> destArray = Local<Array>::Cast(args[1]->ToObject());
  

	Mat cur, cur_grey;
	Mat prev, prev_grey;

	prev = cv::imread(std::string(*NanAsciiString(srcArray->Get(0)->ToString())), CV_LOAD_IMAGE_COLOR);//get the first frame.ch
	cvtColor(prev, prev_grey, COLOR_BGR2GRAY);


	// Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
	vector <TransformParam> prev_to_cur_transform; // previous to current
	// Accumulated frame to frame transform
	double a = 0;
	double x = 0;
	double y = 0;
	// Step 2 - Accumulate the transformations to get the image trajectory
	vector <Trajectory> trajectory; // trajectory at all frames
	//
	// Step 3 - Smooth out the trajectory using an averaging window
	vector <Trajectory> smoothed_trajectory; // trajectory at all frames
	Trajectory X;//posteriori state estimate
	Trajectory	X_;//priori estimate
	Trajectory P;// posteriori estimate error covariance
	Trajectory P_;// priori estimate error covariance
	Trajectory K;//gain
	Trajectory	z;//actual measurement
	double pstd = 4e-3;//can be changed
	double cstd = 0.25;//can be changed
	Trajectory Q(pstd,pstd,pstd);// process noise covariance
	Trajectory R(cstd,cstd,cstd);// measurement noise covariance 
	// Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
	vector <TransformParam> new_prev_to_cur_transform;
	//
	// Step 5 - Apply the new transformation to the video
	//cap.set(CV_CAP_PROP_POS_FRAMES, 0);
	Mat T(2,3,CV_64F);

	int vert_border = HORIZONTAL_BORDER_CROP * prev.rows / prev.cols; // get the aspect ratio correct
	VideoWriter outputVideo; 
	outputVideo.open("compare.avi" , CV_FOURCC('X','V','I','D'), 24,cvSize(cur.rows, cur.cols*2+10), true);  
	//
	int k=1;
	int max_frames = srcArray->Length();
	Mat last_T;
	Mat prev_grey_,cur_grey_;

  for(int i = 0; i<max_frames; i++) {
    cur = cv::imread(std::string(*NanAsciiString(srcArray->Get(i)->ToString())), CV_LOAD_IMAGE_COLOR);
     
		if(cur.data == NULL) {
			break;
		}

		cvtColor(cur, cur_grey, COLOR_BGR2GRAY);

		// vector from prev to cur
		vector <Point2f> prev_corner, cur_corner;
		vector <Point2f> prev_corner2, cur_corner2;
		vector <uchar> status;
		vector <float> err;

		goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
		calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

		// weed out bad matches
		for(size_t i=0; i < status.size(); i++) {
			if(status[i]) {
				prev_corner2.push_back(prev_corner[i]);
				cur_corner2.push_back(cur_corner[i]);
			}
		}

		// translation + rotation only
		Mat T = estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

		// in rare cases no transform is found. We'll just use the last known good transform.
		if(T.data == NULL) {
			last_T.copyTo(T);
		}

		T.copyTo(last_T);

		// decompose T
		double dx = T.at<double>(0,2);
		double dy = T.at<double>(1,2);
		double da = atan2(T.at<double>(1,0), T.at<double>(0,0));
		//
		//prev_to_cur_transform.push_back(TransformParam(dx, dy, da));

		//
		// Accumulated frame to frame transform
		x += dx;
		y += dy;
		a += da;
		//trajectory.push_back(Trajectory(x,y,a));
		//
		//
		z = Trajectory(x,y,a);
		//
		if(k==1){
			// intial guesses
			X = Trajectory(0,0,0); //Initial estimate,  set 0
			P =Trajectory(1,1,1); //set error variance,set 1
		}
		else
		{
			//time update£¨prediction£©
			X_ = X; //X_(k) = X(k-1);
			P_ = P+Q; //P_(k) = P(k-1)+Q;
			// measurement update£¨correction£©
			K = P_/( P_+R ); //gain;K(k) = P_(k)/( P_(k)+R );
			X = X_+K*(z-X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
			P = (Trajectory(1,1,1)-K)*P_; //P(k) = (1-K(k))*P_(k);
		}
		//smoothed_trajectory.push_back(X);
		//-
		// target - current
		double diff_x = X.x - x;//
		double diff_y = X.y - y;
		double diff_a = X.a - a;

		dx = dx + diff_x;
		dy = dy + diff_y;
		da = da + diff_a;

		T.at<double>(0,0) = cos(da);
		T.at<double>(0,1) = -sin(da);
		T.at<double>(1,0) = sin(da);
		T.at<double>(1,1) = cos(da);

		T.at<double>(0,2) = dx;
		T.at<double>(1,2) = dy;

		Mat cur2;
		
		warpAffine(prev, cur2, T, cur.size());
    
    cv::imwrite(std::string(*NanAsciiString(srcArray->Get(0)->ToString())), CV_LOAD_IMAGE_COLOR);
  }


  NanReturnUndefined();
}

void init(Handle<Object> target) {
  NanScope();
  NODE_SET_METHOD(target, "hello", Method);
  NODE_SET_METHOD(target, "stabilize", ComputeKalman);
}

NODE_MODULE(binding, init);
