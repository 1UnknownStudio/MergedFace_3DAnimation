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
	// 面部模型点
	const ColorSpacePoint*  const kinectFacePoints = nullptr;
	// 面部模型点数量
	const UINT              kinectFacePointsCount = 0;
	// 动画单元
	float                   sd[FaceShapeDeformations_Count];
	//kinect脸部跟踪状态
	bool					kinectFaceTracked = false;
	//kinect头部姿态
	Vector4					kinectFaceRotation;
	//OpenFace头部姿态
	cv::Point3f				headRotation;
	//头部中心
	CameraSpacePoint		headPivotPoint;
	//2D头部中心
	ColorSpacePoint			headPivotPoint2D;
};

struct ControlParam {
	//骨骼节点数据
	Joint jointsData;
	// 面部控制参数
	float value;
};

class ImageRenderer {
public:
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;

	//深度图像长宽
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
	//启动定时器线程函数
	void StartProcessingThread();
	//销毁定时器线程
	void Expire();

	cv::Mat								colorFrame;
	cv::Mat								headFrame;


	float*								faceControlParams;
	float*								smoothedFaceControlParams;
	float*								eyeState;
	ControlParam*						controlParams;

	cv::Rect							smoothedHeadRoi;
	//头部区域
	cv::Rect							headRoi;
	
	CFaceAnalyser*						faceAnalyser;
	CKalman*							headRoiFilter;
	CKalman*							controlParamsFilter;
	C_UDPSandG*							message;

	//新线程
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