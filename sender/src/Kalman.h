#pragma once
#include <opencv2/opencv.hpp>

class CKalman
{
public:
	CKalman();//���캯��
	~CKalman();//��������
	void init(int dim, double wn = 1e-3, double vn = 1e-3);//��ʼ������
	//CameraSpacePoint ProcessKalman(CameraSpacePoint);//���п������˲�
	//Vector4 ProcessKalman(Vector4 Quaternion);
	double ProcessKalman(double depth);
	float* ProcessKalman(float *data,const int length);
	cv::Rect ProcessKalman(cv::Rect rect);
private:
	cv::Mat state, processNoise, measurement;
	cv::KalmanFilter KF;
};
