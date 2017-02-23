#include "CKinect.h"

#define lengthof(a) sizeof(a)/sizeof(*a)

CKinect::CKinect()
{	
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];// create heap storage for color pixel data in RGBX format
	m_pDepthRGBX = new RGBQUAD[cDepthWidth*cDepthHeight];// create heap storage for depth pixel data in RGBX format
	
	colorFramePoints = new CameraSpacePoint[cColorWidth * cColorHeight];
	imageRenderer = new ImageRenderer;
}

CKinect::~CKinect()
{
	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	if (m_pDepthRGBX)
	{
		delete[] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
	}

	delete[] colorFramePoints;

	Expire();

	delete imageRenderer;

	// 销毁事件
	if (m_hColorFrameArrived && m_pColorFrameReader) {
		m_pColorFrameReader->UnsubscribeFrameArrived(m_hColorFrameArrived);
		m_hColorFrameArrived = 0;
	}

	// 释放HighDefinitionFaceFrameReader
	SafeRelease(m_pHDFaceFrameReader);
	// 释放HighDefinitionFaceFrameSource
	SafeRelease(m_pHDFaceFrameSource);
	// 释放ColorFrameReader
	SafeRelease(m_pColorFrameReader);
	// 释放DepthFrameReader
	SafeRelease(m_pDepthFrameReader);
	// 释放BodyFrameReader
	SafeRelease(m_pBodyFrameReader);
	// 释放FaceAlignment
	SafeRelease(m_pFaceAlignment);
	// 释放Mapper
	SafeRelease(m_pMapper);
	// 释放Face Model
	SafeRelease(m_pFaceModel);

	//关闭Kinect
	if (m_pKinect) {
		m_pKinect->Close();
		m_pKinect->Release();
		m_pKinect = nullptr;
	}

	// 释放缓存
	if (m_pFaceVertices) {
		free(m_pFaceVertices);
		m_pFaceVertices = nullptr;
		const_cast<const ColorSpacePoint*>(imageRenderer->data.kinectFacePoints) = nullptr;
		const_cast<UINT&>(imageRenderer->data.kinectFacePointsCount) = 0;
	}
}


HRESULT CKinect::initKinect()
{
	IBodyFrameSource* pBodyFrameSource = nullptr;
	IColorFrameSource* pColorFrameSource = nullptr;
	IDepthFrameSource* pDepthFrameSource = nullptr;
	// 查找当前默认Kinect

	HRESULT hr = ::GetDefaultKinectSensor(&m_pKinect);

	if (SUCCEEDED(hr)) {
		hr = m_pKinect->Open();
	}
	// 获取彩色帧源
	if (SUCCEEDED(hr)) {
		m_pKinect->get_ColorFrameSource(&pColorFrameSource);
	}
	// 获取彩色帧读取器
	if (SUCCEEDED(hr)) {
		pColorFrameSource->OpenReader(&m_pColorFrameReader);
	}
	// 注册临帧事件
	if (SUCCEEDED(hr)) {
		m_pColorFrameReader->SubscribeFrameArrived(&m_hColorFrameArrived);
	}
	// 获取深度帧源
	if (SUCCEEDED(hr)) {
		m_pKinect->get_DepthFrameSource(&pDepthFrameSource);
	}
	// 获取深度帧读取器
	if (SUCCEEDED(hr)) {
		pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	}
	// 注册临帧事件
	if (SUCCEEDED(hr)) {
		m_pDepthFrameReader->SubscribeFrameArrived(&m_hDepthFrameArrived);
	}
	// 获取骨骼帧源
	if (SUCCEEDED(hr)) {
		m_pKinect->get_BodyFrameSource(&pBodyFrameSource);
	}
	// 获取骨骼帧读取器
	if (SUCCEEDED(hr)) {
		pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
	}
	// 注册临帧事件
	if (SUCCEEDED(hr)) {
		m_pBodyFrameReader->SubscribeFrameArrived(&m_hBodyFrameArrived);
	}
	// 创建高清面部帧源
	if (SUCCEEDED(hr)) {
		hr = CreateHighDefinitionFaceFrameSource(m_pKinect, &m_pHDFaceFrameSource);
	}
	// 创建高清面部帧读取器
	if (SUCCEEDED(hr)) {
		hr = m_pHDFaceFrameSource->OpenReader(&m_pHDFaceFrameReader);
	}
	// 注册临帧事件
	if (SUCCEEDED(hr)) {
		hr = m_pHDFaceFrameReader->SubscribeFrameArrived(&m_hHDFFrameArrived);
	}
	// 创建面部特征对齐
	if (SUCCEEDED(hr)) {
		hr = CreateFaceAlignment(&m_pFaceAlignment);
	}
	// 创建面部模型
	if (SUCCEEDED(hr)) {
		hr = CreateFaceModel(1.f, FaceShapeDeformations::FaceShapeDeformations_Count, imageRenderer->data.sd, &m_pFaceModel);
	}
	// 获取映射器
	if (SUCCEEDED(hr)) {
		hr = m_pKinect->get_CoordinateMapper(&m_pMapper);
	}
	// 获取面部定点数
	if (SUCCEEDED(hr)) {
		hr = GetFaceModelVertexCount(&m_cFaceVerticeCount);
	}
	// 创建顶点缓存
	if (SUCCEEDED(hr)) {
		m_pFaceVertices = reinterpret_cast<CameraSpacePoint*>(malloc(
			(sizeof(CameraSpacePoint) + sizeof(ColorSpacePoint)) * m_cFaceVerticeCount)
			);
		if (!m_pFaceVertices) hr = E_OUTOFMEMORY;
	}
	// 修改数据
	if (SUCCEEDED(hr)) {
		const_cast<const ColorSpacePoint*>(imageRenderer->data.kinectFacePoints) =
			reinterpret_cast<const ColorSpacePoint*>(m_pFaceVertices + m_cFaceVerticeCount);
		const_cast<UINT&>(imageRenderer->data.kinectFacePointsCount) = m_cFaceVerticeCount;
	}
	if (SUCCEEDED(hr)) {
		StartHDFaceThread();
	}
	SafeRelease(pColorFrameSource);
	SafeRelease(pBodyFrameSource);
	SafeRelease(pDepthFrameSource);
	
	return hr;
}

void CKinect::runMessageLoop()
{
	MSG msg;
	//事件类型
	HANDLE events[] = {
		reinterpret_cast<HANDLE>(m_hColorFrameArrived),
		reinterpret_cast<HANDLE>(m_hBodyFrameArrived),
		reinterpret_cast<HANDLE>(m_hDepthFrameArrived),
		reinterpret_cast<HANDLE>(m_hHDFFrameArrived),

	};
	while (true) {
		// 消息处理
		if (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
		// 设置事件
		// 事件0: 彩色临帧事件
		events[0] = reinterpret_cast<HANDLE>(m_hColorFrameArrived);
		// 事件1: 骨骼临帧事件
		events[1] = reinterpret_cast<HANDLE>(m_hBodyFrameArrived);
		// 事件3: 深度临帧事件
		events[2] = reinterpret_cast<HANDLE>(m_hDepthFrameArrived);
		// 事件2: 高清面部临帧事件
		events[3] = reinterpret_cast<HANDLE>(m_hHDFFrameArrived);

		// 检查事件
		switch (MsgWaitForMultipleObjects(lengthof(events), events, FALSE, INFINITE, QS_ALLINPUT))
		{
			// events[0]
		case WAIT_OBJECT_0 + 0:
			this->checkColorFrame();
			break;
			// events[1]
		case WAIT_OBJECT_0 + 1:
			this->checkBodyFrame();
			break;
			// events[2]
		case WAIT_OBJECT_0 + 2:
			this->checkDepthFrame();
			break;
		case WAIT_OBJECT_0 + 3:
			//this->checkHDFaceFrame();
			HDFaceArrived = true;
			break;
			// events[1]

		default:
			break;
		}
		//// 退出
		//if (cv::waitKey(1) == 27) {
		//	break;
		//}
		//if (imageRenderer->finish) {
		//	break;
		//}
	}
}

void CKinect::checkColorFrame()
{
	if (!m_pColorFrameReader) return;
	// 彩色临帧事件参数
	IColorFrameArrivedEventArgs* pArgs = nullptr;
	// 彩色帧引用
	IColorFrameReference* pCFrameRef = nullptr;
	// 彩色帧
	IColorFrame* pColorFrame = nullptr;
	// 帧描述
	IFrameDescription* pFrameDescription = nullptr;
	// 彩色帧宽度数据
	int width = 0;
	// 彩色帧高度数据
	int height = 0;
	// 帧格式
	ColorImageFormat imageFormat = ColorImageFormat_None;
	// 帧缓存大小
	UINT nBufferSize = 0;
	// 帧缓存
	RGBQUAD *pBuffer = nullptr;
	// 获取参数
	HRESULT hr = m_pColorFrameReader->GetFrameArrivedEventData(m_hColorFrameArrived, &pArgs);
	// 获取引用
	if (SUCCEEDED(hr)) {
		hr = pArgs->get_FrameReference(&pCFrameRef);
	}
	// 获取彩色帧
	if (SUCCEEDED(hr)) {
		hr = pCFrameRef->AcquireFrame(&pColorFrame);
	}
	// 获取帧描述
	if (SUCCEEDED(hr)) {
		hr = pColorFrame->get_FrameDescription(&pFrameDescription);
	}
	// 获取帧宽度
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Width(&width);
	}
	// 获取帧高度
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Height(&height);
	}
	// 获取帧格式
	if (SUCCEEDED(hr)) {
		hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
	}
	// 获取帧真数据
	if (SUCCEEDED(hr)) {
		// 在已经是BGRA的情况下 直接获取源数据
		if (imageFormat == ColorImageFormat_Bgra) {
			hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
		}
		// 负责用自带的方法转换
		else if (m_pColorRGBX)
		{
			pBuffer = m_pColorRGBX;
			nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
			hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
		}
		else
		{
			hr = E_FAIL;
		}
	}
	if (SUCCEEDED(hr)) {
		//TODO:
		memcpy(imageRenderer->colorFrameData, pBuffer, sizeof(RGBQUAD)*cColorHeight*cColorWidth);
		imageRenderer->colorFrameArrived = true;
		//imageRenderer->showColorFrame(pBuffer);
		//imageRenderer->writeColorFrameData(pBuffer, width, height);
	}
	// 安全释放
	SafeRelease(pFrameDescription);
	SafeRelease(pColorFrame);
	SafeRelease(pCFrameRef);
	SafeRelease(pArgs);
}

void CKinect::checkDepthFrame() 
{
	if (!m_pDepthFrameReader) return;
	// 深度临帧事件参数
	IDepthFrameArrivedEventArgs* pArgs = nullptr;
	// 深度帧引用
	IDepthFrameReference* pDFrameRef = nullptr;
	// 深度帧
	IDepthFrame* pDepthFrame = nullptr;
	// 帧描述
	IFrameDescription* pFrameDescription = nullptr;
	// 深度帧宽度数据
	int width = 0;
	// 深度帧高度数据
	int height = 0;
	// 深度帧最小距离
	USHORT nDepthMinReliableDistance = 0;
	// 深度帧最大距离
	USHORT nDepthMaxDistance = 0;
	// 帧缓存大小
	UINT nBufferSize = 0;
	// 帧缓存
	UINT16 *pBuffer = NULL;
	// 获取参数
	HRESULT hr = m_pDepthFrameReader->GetFrameArrivedEventData(m_hDepthFrameArrived, &pArgs);
	// 获取引用
	if (SUCCEEDED(hr)) {
		hr = pArgs->get_FrameReference(&pDFrameRef);
	}
	// 获取深度帧
	if (SUCCEEDED(hr)) {
		hr = pDFrameRef->AcquireFrame(&pDepthFrame);
	}
	// 获取帧描述
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
	}
	// 获取帧宽度
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Width(&width);
	}
	// 获取帧高度
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Height(&height);
	}
	//获取深度最小阈值
	if (SUCCEEDED(hr))
	{
		hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
	}
	//获取深度最大阈值
	if (SUCCEEDED(hr))
	{
		// In order to see the full range of depth (including the less reliable far field depth)
		// we are setting nDepthMaxDistance to the extreme potential depth threshold
		nDepthMaxDistance = USHRT_MAX;

		// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
		//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
	}

	if (SUCCEEDED(hr))
	{
		hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
	}

	imageRenderer->showDepthFrame(m_pDepthRGBX, pBuffer, width, height, nDepthMinReliableDistance, nDepthMaxDistance);

	m_pMapper->MapColorFrameToCameraSpace(
							cDepthWidth*cDepthHeight,
							pBuffer,
							cColorWidth*cColorHeight,
							colorFramePoints
						);
	// 安全释放
	SafeRelease(pFrameDescription);
	SafeRelease(pDepthFrame);
	SafeRelease(pDFrameRef);
	SafeRelease(pArgs);
}

void CKinect::checkBodyFrame()
{
	// 骨骼临帧事件参数
	IBodyFrameArrivedEventArgs* pArgs = nullptr;
	// 骨骼帧引用
	IBodyFrameReference* pBFrameRef = nullptr;
	// 骨骼帧
	IBodyFrame* pBodyFrame = nullptr;
	// 骨骼
	IBody*  ppBody[BODY_COUNT] = { 0 };

	// 获取参数
 	HRESULT hr = m_pBodyFrameReader->GetFrameArrivedEventData(m_hBodyFrameArrived, &pArgs);
	// 获取引用
	if (SUCCEEDED(hr)) {
		hr = pArgs->get_FrameReference(&pBFrameRef);
	}
	// 获取骨骼帧
	if (SUCCEEDED(hr)) {
		hr = pBFrameRef->AcquireFrame(&pBodyFrame);
	}
	// 获取骨骼数据
	if (SUCCEEDED(hr)) {
		hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBody);
	}

	// 高清面部帧源未被跟踪时尝试更换ID
	BOOLEAN tracked = FALSE;

	// 检查是否未被跟踪
	if (SUCCEEDED(hr)) {
		hr = m_pHDFaceFrameSource->get_IsTrackingIdValid(&tracked);
	}
	//if (SUCCEEDED(hr) && tracked) {
	//	static int cnt = 0;
	//	std::cout << "tracked" << cnt << std::endl;
	//	cnt++;
	//}
	// 骨骼未被跟踪时尝试更换ID
	if (SUCCEEDED(hr) && !tracked) {
		for (int i = 0; i < BODY_COUNT; ++i) {
			hr = ppBody[i]->get_IsTracked(&tracked);
			if (SUCCEEDED(hr) && tracked) {
				UINT64 id = 0;
				if (FAILED(ppBody[i]->get_TrackingId(&id))) continue;

				m_pHDFaceFrameSource->put_TrackingId(id);
				break;
			}
		}

	}
	else if (SUCCEEDED(hr) && tracked) {
		for (int i = 0; i < BODY_COUNT; ++i) {
			hr = ppBody[i]->get_IsTracked(&tracked);
			if (SUCCEEDED(hr) && tracked) {
				UINT64 id = 0;
				//HandState leftHandState = HandState_Unknown;
				//HandState rightHandState = HandState_Unknown;

				//ppBody[i]->get_HandLeftState(&leftHandState);
				//ppBody[i]->get_HandRightState(&rightHandState);

				hr = ppBody[i]->GetJoints(_countof(imageRenderer->joints), imageRenderer->joints);
				imageRenderer->sendControlParams();
				//hr = ppBody[i]->GetJointOrientations(_countof(jointOrientation), jointOrientation);
				//if (SUCCEEDED(hr)) {
					//imageRenderer->data.kinectFaceTracked = true;
					//imageRenderer->getControlParams();

				//}
				if (FAILED(ppBody[i]->get_TrackingId(&id))) continue;

				break;
			}
		}
	}
	
	// 安全释放
	for (int i = 0; i < BODY_COUNT; ++i) SafeRelease(ppBody[i]);
	SafeRelease(pBodyFrame);
	SafeRelease(pBFrameRef);
	SafeRelease(pArgs);
}

void CKinect::checkHDFaceFrame()
{

	// 高清面部临帧事件参数
	IHighDefinitionFaceFrameArrivedEventArgs* pArgs = nullptr;
	// 高清面部帧引用
	IHighDefinitionFaceFrameReference* pHDFFrameRef = nullptr;
	// 高清面部帧
	IHighDefinitionFaceFrame* pHDFaceFrame = nullptr;

	// 获取参数
	HRESULT hr = m_pHDFaceFrameReader->GetFrameArrivedEventData(m_hHDFFrameArrived, &pArgs);
	// 获取引用
	if (SUCCEEDED(hr)) {
		hr = pArgs->get_FrameReference(&pHDFFrameRef);
	}
	// 获取骨骼帧
	if (SUCCEEDED(hr)) {
		hr = pHDFFrameRef->AcquireFrame(&pHDFaceFrame);
	}
	// 更新面部特征对齐
	if (SUCCEEDED(hr)) {
		hr = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(m_pFaceAlignment);
	}
	// 获取面部顶点
	if (SUCCEEDED(hr)) {
		hr = m_pFaceModel->CalculateVerticesForAlignment(m_pFaceAlignment, m_cFaceVerticeCount, m_pFaceVertices);
		//std::cout << m_pFaceVertices[0].X << std::endl;
	}
	// 成功
	if (SUCCEEDED(hr)) {
		for (UINT i = 0U; i < m_cFaceVerticeCount; ++i) {
			m_pMapper->MapCameraPointsToColorSpace(m_cFaceVerticeCount, m_pFaceVertices,
				imageRenderer->data.kinectFacePointsCount,
				const_cast<ColorSpacePoint*>(imageRenderer->data.kinectFacePoints)
			);
		}
	}
	// 获取面部旋转四元数
	if (SUCCEEDED(hr)) {
		hr = m_pFaceAlignment->get_FaceOrientation(&imageRenderer->data.kinectFaceRotation);
	}
	//获取头部中点
	if (SUCCEEDED(hr)) {
		hr = m_pFaceAlignment->get_HeadPivotPoint(&imageRenderer->data.headPivotPoint);
	}
	if (SUCCEEDED(hr)) {
		hr = m_pMapper->MapCameraPointToColorSpace(imageRenderer->data.headPivotPoint, &imageRenderer->data.headPivotPoint2D);
	}
	//// 获取AU数据
	//if (SUCCEEDED(hr)) {
	//	hr = m_pFaceAlignment->GetAnimationUnits(FaceShapeAnimations_Count, imageRenderer->data.au);
	//}
	// 检查跟踪
	if (SUCCEEDED(hr)) {
		imageRenderer->data.kinectFaceTracked = true;
		//imageRenderer->getControlParams();
		static int cnt = 0;
		cnt++;
		std::cout << "tracked" << cnt << "\r";
	}
	else
	{
		imageRenderer->data.kinectFaceTracked = false;
	}
	// 安全释放
	SafeRelease(pHDFaceFrame);
	SafeRelease(pHDFFrameRef);
	SafeRelease(pArgs);
}

void CKinect::StartHDFaceThread() {
	if (expired_ == false) {
		//			std::cout << "timer is currently running, please expire it first..." << std::endl;
		return;
	}
	expired_ = false;
	std::thread([this]() {
		while (!try_to_expire_) {
			if (HDFaceArrived) {
				checkHDFaceFrame();
				//std::this_thread::sleep_for(std::chrono::milliseconds(interval));
				HDFaceArrived = false;
			}
		}
		//			std::cout << "stop task..." << std::endl;
		{
			std::lock_guard<std::mutex> locker(mutex_);
			expired_ = true;
			expired_cond_.notify_one();
		}
	}).detach();
}

void CKinect::Expire() {
	if (expired_) {
		return;
	}

	if (try_to_expire_) {
		//			std::cout << "timer is trying to expire, please wait..." << std::endl;
		return;
	}
	try_to_expire_ = true;
	{
		std::unique_lock<std::mutex> locker(mutex_);
		expired_cond_.wait(locker, [this] {return expired_ == true; });
		if (expired_ == true) {
			//				std::cout << "timer expired!" << std::endl;
			try_to_expire_ = false;
		}
	}
}
