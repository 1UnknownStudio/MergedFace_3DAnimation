#pragma once
#define WIN32_LEAN_AND_MEAN     // Exclude rarely-used stuff from Windows headers
#include <Kinect.h>
#include <Kinect.Face.h>
#include <iostream>


#include "ImageRenderer.h"



class CKinect
{
public:
	//��ɫͼ�񳤿�
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;
	//���ͼ�񳤿�
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	//���캯��
	CKinect();
	//��������
	~CKinect();

	// ��ʼ��Kinect
	HRESULT initKinect();
	//��Ϣѭ��
	void runMessageLoop();
public:


private:
	// ����ɫ֡
	void checkColorFrame();
	// ������֡
	void checkBodyFrame();
	// �������沿
	void checkHDFaceFrame();
	// ������֡
	void checkDepthFrame();
	// ������ʱ���̺߳���
	void StartHDFaceThread();
	// ���ٶ�ʱ���߳�
	void Expire();

private:
	// Kinect v2 ������
	IKinectSensor*                      m_pKinect = nullptr;
	// ��ɫ֡��ȡ��
	IColorFrameReader*                  m_pColorFrameReader = nullptr;
	// �����沿֡Դ
	IHighDefinitionFaceFrameSource*     m_pHDFaceFrameSource = nullptr;
	// �����沿֡��ȡ��
	IHighDefinitionFaceFrameReader*     m_pHDFaceFrameReader = nullptr;
	// �沿��������
	IFaceAlignment*                     m_pFaceAlignment = nullptr;
	//���֡��ȡ��
	IDepthFrameReader*					m_pDepthFrameReader = nullptr;
	// ����֡��ȡ��
	IBodyFrameReader*                   m_pBodyFrameReader = nullptr;
	// �沿ģ��
	IFaceModel*                         m_pFaceModel = nullptr;
	// �沿ģ�Ͷ���
	CameraSpacePoint*                   m_pFaceVertices = nullptr;
	// �沿ģ�Ͷ�������
	UINT                                m_cFaceVerticeCount = 0;
	// �沿���Ķ�������
	CameraSpacePoint					headPivotPoint;
	// ����ӳ����
	ICoordinateMapper*                  m_pMapper = nullptr;
	// ��ɫ��֡�¼�
	WAITABLE_HANDLE                     m_hColorFrameArrived = 0;
	// �����֡�¼�
	WAITABLE_HANDLE						m_hDepthFrameArrived = 0;
	// ������֡�¼�
	WAITABLE_HANDLE                     m_hBodyFrameArrived = 0;
	// �����沿��֡�¼�
	WAITABLE_HANDLE                     m_hHDFFrameArrived = 0;
	//��ʼ��״̬���
	HRESULT								initSuccess = false;
	//��ɫͼ��֡ԭʼ��ʽ����
	RGBQUAD*							m_pColorRGBX;
	//���ͼ��֡ԭʼ��ʽ����
	RGBQUAD*							m_pDepthRGBX;
	//ͼ����Ⱦ��
	ImageRenderer*						imageRenderer;

	CameraSpacePoint*					colorFramePoints;

	//���߳�
	std::atomic<bool>					expired_ = true;
	std::atomic<bool>					try_to_expire_ = false;
	std::mutex							mutex_;
	std::condition_variable				expired_cond_;
	//HD�沿֡��֡
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