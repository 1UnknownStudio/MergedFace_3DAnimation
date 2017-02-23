#pragma once
#define WIN32_LEAN_AND_MEAN     // Exclude rarely-used stuff from Windows headers
#include <Kinect.h>
#include <Kinect.Face.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include<functional>
#include<chrono>
#include<thread>
#include<atomic>
#include<memory>
#include<mutex>
#include<condition_variable>

#include "FaceAnalyser.h"
#include "Kalman.h"
#include "MessageHander.h"

struct FaceData {
	// �沿ģ�͵�
	const ColorSpacePoint*  const kinectFacePoints = nullptr;
	// �沿ģ�͵�����
	const UINT              kinectFacePointsCount = 0;
	// ������Ԫ
	float                   sd[FaceShapeDeformations_Count];
	//kinect��������״̬
	bool					kinectFaceTracked = false;
	//kinectͷ����̬
	Vector4					kinectFaceRotation;
	//OpenFaceͷ����̬
	cv::Point3f				headRotation;
	//ͷ������
	CameraSpacePoint		headPivotPoint;
	//2Dͷ������
	ColorSpacePoint			headPivotPoint2D;
};

struct ControlParam {
	//�����ڵ�����
	Joint jointsData;
	// �沿���Ʋ���
	float value;
};

class ImageRenderer {
public:
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;

	//���ͼ�񳤿�
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

	static const int		auParamsCount = 14;
	static const int		headRotationParamsCount = 4;
	static const int		eyeParamsCount = 5;
	static const int		faceControlParamsCount = auParamsCount + headRotationParamsCount + eyeParamsCount;
	static const int		controlParamsCount = 25;


	ImageRenderer();
	~ImageRenderer();

	void showColorFrame(RGBQUAD* pBuffer);

	void showDepthFrame(RGBQUAD* m_pDepthRGBX, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);

	void sendControlParams();


	FaceData							data;
	Joint								joints[JointType_Count];
	RGBQUAD*							colorFrameData;
	bool								colorFrameArrived = false;
private:
	//reading kinect color frame stream to mat format
	void writeColorFrameData();
	//get head roi from color frame
	void getHeadROI();
	//processing color image
	void processImage();
	//������ʱ���̺߳���
	void StartProcessingThread();
	//���ٶ�ʱ���߳�
	void Expire();

	cv::Mat								colorFrame;
	cv::Mat								headFrame;


	float*								faceControlParams;
	float*								smoothedFaceControlParams;
	float*								eyeState;
	ControlParam*						controlParams;

	cv::Rect							smoothedHeadRoi;
	//ͷ������
	cv::Rect							headRoi;
	
	CFaceAnalyser*						faceAnalyser;
	CKalman*							headRoiFilter;
	CKalman*							controlParamsFilter;
	C_UDPSandG*							message;

	//���߳�
	std::atomic<bool>					expired_ = true;
	std::atomic<bool>					try_to_expire_ = false;
	std::mutex							mutex_;
	std::condition_variable				expired_cond_;

	//OpenFace model files
	const std::vector<std::string> paths = {
		"E:/Program Files/OpenFaceLib/model/main_clnf_general.txt",
		"E:/Program Files/OpenFaceLib/AU_predictors/AU_all_best.txt",
		"E:/Program Files/OpenFaceLib/model/tris_68_full.txt"
	};
};