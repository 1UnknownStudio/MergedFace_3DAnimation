#include "Kalman.h"

using namespace cv;

CKalman::CKalman()
{

}

CKalman::~CKalman()
{

}

void CKalman::init(int dim, double wn,double vn)
{
	//������ֵΪ0����׼��0.1�Ķ�ά��˹������
	KF.init(2 * dim, dim);
	state.create(2 * dim, 1, CV_32F);
	processNoise.create(2 * dim, 1, CV_32F);
	measurement = Mat::zeros(dim, 1, CV_32F);
	randn(state, Scalar::all(0), Scalar::all(0.1));
	//transitionMatrixΪ��KalmanFilter�е�һ��������Mat�ͣ���Kalmanģ���е�״̬ת�ƾ���
	Mat transMtx(2 * dim, 2 * dim, CV_32F);
	for (int i = 0; i < 2 * dim; i++)
	{
		for (int j = 0; j < 2 * dim; j++)
		{
			if ((i == j) || (i == j + dim))
			{
				transMtx.at<float>(i, j) = 1;
			}
			else
			{
				transMtx.at<float>(i, j) = 0;
			}
		}
	}
	KF.transitionMatrix = transMtx;

	//����setIdentity�Ǹ���������Խ��߸���ֵͬ��Ĭ�϶Խ���ֵֵΪ1
	setIdentity(KF.measurementMatrix);
	//ϵͳ���������������
	setIdentity(KF.processNoiseCov, Scalar::all(wn));
	//�������������������
	setIdentity(KF.measurementNoiseCov, Scalar::all(vn));
	//����������Э�������
	setIdentity(KF.errorCovPost, Scalar::all(1));
	//statePostΪУ��״̬���䱾�ʾ���ǰһʱ�̵�״̬
	randn(KF.statePost, Scalar::all(0), Scalar::all(1));
}

//CameraSpacePoint CKalman::ProcessKalman(CameraSpacePoint pt)
//{
//	Mat prediction = KF.predict();
//	CameraSpacePoint predictPt{ prediction.at<float>(0), prediction.at<float>(1),prediction.at<float>(2)};
//	measurement.at<float>(0) = pt.X;
//	measurement.at<float>(1) = pt.Y;
//	measurement.at<float>(2) = pt.Z;
//	KF.correct(measurement);
//	return CameraSpacePoint{ predictPt.X, predictPt.Y, predictPt.Z };
//}
//
//Vector4 CKalman::ProcessKalman(Vector4 Quaternion)
//{
//	Mat prediction = KF.predict();
//	Vector4 predictQuaternion{ prediction.at<float>(0), prediction.at<float>(1), prediction.at<float>(2), prediction.at<float>(3) };
//	measurement.at<float>(0) = Quaternion.x;
//	measurement.at<float>(1) = Quaternion.y;
//	measurement.at<float>(2) = Quaternion.z;
//	measurement.at<float>(3) = Quaternion.w;
//	KF.correct(measurement);
//	return predictQuaternion;
//}

double CKalman::ProcessKalman(double depth)
{
	Mat prediction = KF.predict();
	double predictDepth = prediction.at<float>(0);
	measurement.at<float>(0) = depth;
	KF.correct(measurement);
	return predictDepth;
}

float* CKalman::ProcessKalman(float *data,const int length) {
	Mat prediction = KF.predict();
	float* out = new float[length];
	for (int i = 0; i < length; ++i) {
		out[i] = prediction.at<float>(i);
	}
	for (int i = 0; i < length; ++i) {
		measurement.at<float>(i) = data[i];
	}
	KF.correct(measurement);
	return out;
}

cv::Rect CKalman::ProcessKalman(cv::Rect rect) {
	cv::Mat prediction = KF.predict();
	int out[4];
	for (int i = 0; i < 4; ++i) {
		out[i] = prediction.at<float>(i);
	}

	measurement.at<float>(0) = rect.x;
	measurement.at<float>(1) = rect.y;
	measurement.at<float>(2) = rect.width;
	measurement.at<float>(3) = rect.height;

	KF.correct(measurement);

	return cv::Rect(out[0], out[1], out[2], out[3]);
}