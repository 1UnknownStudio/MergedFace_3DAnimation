#pragma once
#include <opencv2/opencv.hpp>

class CKalman
{
public:
	CKalman();//构造函数
	~CKalman();//析构函数
	void init(int dim, double wn = 1e-3, double vn = 1e-3);//初始化函数
	//CameraSpacePoint ProcessKalman(CameraSpacePoint);//进行卡尔曼滤波
	//Vector4 ProcessKalman(Vector4 Quaternion);
	double ProcessKalman(double depth);
	float* ProcessKalman(float *data,const int length);
	cv::Rect ProcessKalman(cv::Rect rect);
private:
	cv::Mat state, processNoise, measurement;
	cv::KalmanFilter KF;
};
