#pragma once
#define WIN32_LEAN_AND_MEAN     // Exclude rarely-used stuff from Windows headers
#include <Kinect.h>
#include <Kinect.Face.h>
#include <iostream>


#include "ImageRenderer.h"



class CKinect
{
public:
	//彩色图像长宽
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;
	//深度图像长宽
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	//构造函数
	CKinect();
	//析构函数
	~CKinect();

	// 初始化Kinect
	HRESULT initKinect();
	//消息循环
	void runMessageLoop();
public:


private:
	// 检查彩色帧
	void checkColorFrame();
	// 检查骨骼帧
	void checkBodyFrame();
	// 检查高清面部
	void checkHDFaceFrame();
	// 检查深度帧
	void checkDepthFrame();
	// 启动定时器线程函数
	void StartHDFaceThread();
	// 销毁定时器线程
	void Expire();

private:
	// Kinect v2 传感器
	IKinectSensor*                      m_pKinect = nullptr;
	// 彩色帧读取器
	IColorFrameReader*                  m_pColorFrameReader = nullptr;
	// 高清面部帧源
	IHighDefinitionFaceFrameSource*     m_pHDFaceFrameSource = nullptr;
	// 高清面部帧读取器
	IHighDefinitionFaceFrameReader*     m_pHDFaceFrameReader = nullptr;
	// 面部特征对齐
	IFaceAlignment*                     m_pFaceAlignment = nullptr;
	//深度帧读取器
	IDepthFrameReader*					m_pDepthFrameReader = nullptr;
	// 骨骼帧读取器
	IBodyFrameReader*                   m_pBodyFrameReader = nullptr;
	// 面部模型
	IFaceModel*                         m_pFaceModel = nullptr;
	// 面部模型顶点
	CameraSpacePoint*                   m_pFaceVertices = nullptr;
	// 面部模型顶点数量
	UINT                                m_cFaceVerticeCount = 0;
	// 面部中心顶点坐标
	CameraSpacePoint					headPivotPoint;
	// 坐标映射器
	ICoordinateMapper*                  m_pMapper = nullptr;
	// 彩色临帧事件
	WAITABLE_HANDLE                     m_hColorFrameArrived = 0;
	// 深度临帧事件
	WAITABLE_HANDLE						m_hDepthFrameArrived = 0;
	// 骨骼临帧事件
	WAITABLE_HANDLE                     m_hBodyFrameArrived = 0;
	// 高清面部临帧事件
	WAITABLE_HANDLE                     m_hHDFFrameArrived = 0;
	//初始化状态检测
	HRESULT								initSuccess = false;
	//彩色图像帧原始格式数据
	RGBQUAD*							m_pColorRGBX;
	//深度图像帧原始格式数据
	RGBQUAD*							m_pDepthRGBX;
	//图像渲染类
	ImageRenderer*						imageRenderer;

	CameraSpacePoint*					colorFramePoints;

	//新线程
	std::atomic<bool>					expired_ = true;
	std::atomic<bool>					try_to_expire_ = false;
	std::mutex							mutex_;
	std::condition_variable				expired_cond_;
	//HD面部帧临帧
	bool								HDFaceArrived = false;

	int									trackID = -1;
};
template<class Interface>
inline void SafeRelease(Interface *&pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}