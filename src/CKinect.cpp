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

	// �����¼�
	if (m_hColorFrameArrived && m_pColorFrameReader) {
		m_pColorFrameReader->UnsubscribeFrameArrived(m_hColorFrameArrived);
		m_hColorFrameArrived = 0;
	}

	// �ͷ�HighDefinitionFaceFrameReader
	SafeRelease(m_pHDFaceFrameReader);
	// �ͷ�HighDefinitionFaceFrameSource
	SafeRelease(m_pHDFaceFrameSource);
	// �ͷ�ColorFrameReader
	SafeRelease(m_pColorFrameReader);
	// �ͷ�DepthFrameReader
	SafeRelease(m_pDepthFrameReader);
	// �ͷ�BodyFrameReader
	SafeRelease(m_pBodyFrameReader);
	// �ͷ�FaceAlignment
	SafeRelease(m_pFaceAlignment);
	// �ͷ�Mapper
	SafeRelease(m_pMapper);
	// �ͷ�Face Model
	SafeRelease(m_pFaceModel);

	//�ر�Kinect
	if (m_pKinect) {
		m_pKinect->Close();
		m_pKinect->Release();
		m_pKinect = nullptr;
	}

	// �ͷŻ���
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
	// ���ҵ�ǰĬ��Kinect

	HRESULT hr = ::GetDefaultKinectSensor(&m_pKinect);

	if (SUCCEEDED(hr)) {
		hr = m_pKinect->Open();
	}
	// ��ȡ��ɫ֡Դ
	if (SUCCEEDED(hr)) {
		m_pKinect->get_ColorFrameSource(&pColorFrameSource);
	}
	// ��ȡ��ɫ֡��ȡ��
	if (SUCCEEDED(hr)) {
		pColorFrameSource->OpenReader(&m_pColorFrameReader);
	}
	// ע����֡�¼�
	if (SUCCEEDED(hr)) {
		m_pColorFrameReader->SubscribeFrameArrived(&m_hColorFrameArrived);
	}
	// ��ȡ���֡Դ
	if (SUCCEEDED(hr)) {
		m_pKinect->get_DepthFrameSource(&pDepthFrameSource);
	}
	// ��ȡ���֡��ȡ��
	if (SUCCEEDED(hr)) {
		pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	}
	// ע����֡�¼�
	if (SUCCEEDED(hr)) {
		m_pDepthFrameReader->SubscribeFrameArrived(&m_hDepthFrameArrived);
	}
	// ��ȡ����֡Դ
	if (SUCCEEDED(hr)) {
		m_pKinect->get_BodyFrameSource(&pBodyFrameSource);
	}
	// ��ȡ����֡��ȡ��
	if (SUCCEEDED(hr)) {
		pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
	}
	// ע����֡�¼�
	if (SUCCEEDED(hr)) {
		m_pBodyFrameReader->SubscribeFrameArrived(&m_hBodyFrameArrived);
	}
	// ���������沿֡Դ
	if (SUCCEEDED(hr)) {
		hr = CreateHighDefinitionFaceFrameSource(m_pKinect, &m_pHDFaceFrameSource);
	}
	// ���������沿֡��ȡ��
	if (SUCCEEDED(hr)) {
		hr = m_pHDFaceFrameSource->OpenReader(&m_pHDFaceFrameReader);
	}
	// ע����֡�¼�
	if (SUCCEEDED(hr)) {
		hr = m_pHDFaceFrameReader->SubscribeFrameArrived(&m_hHDFFrameArrived);
	}
	// �����沿��������
	if (SUCCEEDED(hr)) {
		hr = CreateFaceAlignment(&m_pFaceAlignment);
	}
	// �����沿ģ��
	if (SUCCEEDED(hr)) {
		hr = CreateFaceModel(1.f, FaceShapeDeformations::FaceShapeDeformations_Count, imageRenderer->data.sd, &m_pFaceModel);
	}
	// ��ȡӳ����
	if (SUCCEEDED(hr)) {
		hr = m_pKinect->get_CoordinateMapper(&m_pMapper);
	}
	// ��ȡ�沿������
	if (SUCCEEDED(hr)) {
		hr = GetFaceModelVertexCount(&m_cFaceVerticeCount);
	}
	// �������㻺��
	if (SUCCEEDED(hr)) {
		m_pFaceVertices = reinterpret_cast<CameraSpacePoint*>(malloc(
			(sizeof(CameraSpacePoint) + sizeof(ColorSpacePoint)) * m_cFaceVerticeCount)
			);
		if (!m_pFaceVertices) hr = E_OUTOFMEMORY;
	}
	// �޸�����
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
	//�¼�����
	HANDLE events[] = {
		reinterpret_cast<HANDLE>(m_hColorFrameArrived),
		reinterpret_cast<HANDLE>(m_hBodyFrameArrived),
		reinterpret_cast<HANDLE>(m_hDepthFrameArrived),
		reinterpret_cast<HANDLE>(m_hHDFFrameArrived),

	};
	while (true) {
		// ��Ϣ����
		if (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
		// �����¼�
		// �¼�0: ��ɫ��֡�¼�
		events[0] = reinterpret_cast<HANDLE>(m_hColorFrameArrived);
		// �¼�1: ������֡�¼�
		events[1] = reinterpret_cast<HANDLE>(m_hBodyFrameArrived);
		// �¼�3: �����֡�¼�
		events[2] = reinterpret_cast<HANDLE>(m_hDepthFrameArrived);
		// �¼�2: �����沿��֡�¼�
		events[3] = reinterpret_cast<HANDLE>(m_hHDFFrameArrived);

		// ����¼�
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
		//// �˳�
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
	// ��ɫ��֡�¼�����
	IColorFrameArrivedEventArgs* pArgs = nullptr;
	// ��ɫ֡����
	IColorFrameReference* pCFrameRef = nullptr;
	// ��ɫ֡
	IColorFrame* pColorFrame = nullptr;
	// ֡����
	IFrameDescription* pFrameDescription = nullptr;
	// ��ɫ֡�������
	int width = 0;
	// ��ɫ֡�߶�����
	int height = 0;
	// ֡��ʽ
	ColorImageFormat imageFormat = ColorImageFormat_None;
	// ֡�����С
	UINT nBufferSize = 0;
	// ֡����
	RGBQUAD *pBuffer = nullptr;
	// ��ȡ����
	HRESULT hr = m_pColorFrameReader->GetFrameArrivedEventData(m_hColorFrameArrived, &pArgs);
	// ��ȡ����
	if (SUCCEEDED(hr)) {
		hr = pArgs->get_FrameReference(&pCFrameRef);
	}
	// ��ȡ��ɫ֡
	if (SUCCEEDED(hr)) {
		hr = pCFrameRef->AcquireFrame(&pColorFrame);
	}
	// ��ȡ֡����
	if (SUCCEEDED(hr)) {
		hr = pColorFrame->get_FrameDescription(&pFrameDescription);
	}
	// ��ȡ֡���
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Width(&width);
	}
	// ��ȡ֡�߶�
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Height(&height);
	}
	// ��ȡ֡��ʽ
	if (SUCCEEDED(hr)) {
		hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
	}
	// ��ȡ֡������
	if (SUCCEEDED(hr)) {
		// ���Ѿ���BGRA������� ֱ�ӻ�ȡԴ����
		if (imageFormat == ColorImageFormat_Bgra) {
			hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
		}
		// �������Դ��ķ���ת��
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
	// ��ȫ�ͷ�
	SafeRelease(pFrameDescription);
	SafeRelease(pColorFrame);
	SafeRelease(pCFrameRef);
	SafeRelease(pArgs);
}

void CKinect::checkDepthFrame() 
{
	if (!m_pDepthFrameReader) return;
	// �����֡�¼�����
	IDepthFrameArrivedEventArgs* pArgs = nullptr;
	// ���֡����
	IDepthFrameReference* pDFrameRef = nullptr;
	// ���֡
	IDepthFrame* pDepthFrame = nullptr;
	// ֡����
	IFrameDescription* pFrameDescription = nullptr;
	// ���֡�������
	int width = 0;
	// ���֡�߶�����
	int height = 0;
	// ���֡��С����
	USHORT nDepthMinReliableDistance = 0;
	// ���֡������
	USHORT nDepthMaxDistance = 0;
	// ֡�����С
	UINT nBufferSize = 0;
	// ֡����
	UINT16 *pBuffer = NULL;
	// ��ȡ����
	HRESULT hr = m_pDepthFrameReader->GetFrameArrivedEventData(m_hDepthFrameArrived, &pArgs);
	// ��ȡ����
	if (SUCCEEDED(hr)) {
		hr = pArgs->get_FrameReference(&pDFrameRef);
	}
	// ��ȡ���֡
	if (SUCCEEDED(hr)) {
		hr = pDFrameRef->AcquireFrame(&pDepthFrame);
	}
	// ��ȡ֡����
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
	}
	// ��ȡ֡���
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Width(&width);
	}
	// ��ȡ֡�߶�
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Height(&height);
	}
	//��ȡ�����С��ֵ
	if (SUCCEEDED(hr))
	{
		hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
	}
	//��ȡ��������ֵ
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
	// ��ȫ�ͷ�
	SafeRelease(pFrameDescription);
	SafeRelease(pDepthFrame);
	SafeRelease(pDFrameRef);
	SafeRelease(pArgs);
}

void CKinect::checkBodyFrame()
{
	// ������֡�¼�����
	IBodyFrameArrivedEventArgs* pArgs = nullptr;
	// ����֡����
	IBodyFrameReference* pBFrameRef = nullptr;
	// ����֡
	IBodyFrame* pBodyFrame = nullptr;
	// ����
	IBody*  ppBody[BODY_COUNT] = { 0 };

	// ��ȡ����
 	HRESULT hr = m_pBodyFrameReader->GetFrameArrivedEventData(m_hBodyFrameArrived, &pArgs);
	// ��ȡ����
	if (SUCCEEDED(hr)) {
		hr = pArgs->get_FrameReference(&pBFrameRef);
	}
	// ��ȡ����֡
	if (SUCCEEDED(hr)) {
		hr = pBFrameRef->AcquireFrame(&pBodyFrame);
	}
	// ��ȡ��������
	if (SUCCEEDED(hr)) {
		hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBody);
	}

	// �����沿֡Դδ������ʱ���Ը���ID
	BOOLEAN tracked = FALSE;

	// ����Ƿ�δ������
	if (SUCCEEDED(hr)) {
		hr = m_pHDFaceFrameSource->get_IsTrackingIdValid(&tracked);
	}
	//if (SUCCEEDED(hr) && tracked) {
	//	static int cnt = 0;
	//	std::cout << "tracked" << cnt << std::endl;
	//	cnt++;
	//}
	// ����δ������ʱ���Ը���ID
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
	
	// ��ȫ�ͷ�
	for (int i = 0; i < BODY_COUNT; ++i) SafeRelease(ppBody[i]);
	SafeRelease(pBodyFrame);
	SafeRelease(pBFrameRef);
	SafeRelease(pArgs);
}

void CKinect::checkHDFaceFrame()
{

	// �����沿��֡�¼�����
	IHighDefinitionFaceFrameArrivedEventArgs* pArgs = nullptr;
	// �����沿֡����
	IHighDefinitionFaceFrameReference* pHDFFrameRef = nullptr;
	// �����沿֡
	IHighDefinitionFaceFrame* pHDFaceFrame = nullptr;

	// ��ȡ����
	HRESULT hr = m_pHDFaceFrameReader->GetFrameArrivedEventData(m_hHDFFrameArrived, &pArgs);
	// ��ȡ����
	if (SUCCEEDED(hr)) {
		hr = pArgs->get_FrameReference(&pHDFFrameRef);
	}
	// ��ȡ����֡
	if (SUCCEEDED(hr)) {
		hr = pHDFFrameRef->AcquireFrame(&pHDFaceFrame);
	}
	// �����沿��������
	if (SUCCEEDED(hr)) {
		hr = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(m_pFaceAlignment);
	}
	// ��ȡ�沿����
	if (SUCCEEDED(hr)) {
		hr = m_pFaceModel->CalculateVerticesForAlignment(m_pFaceAlignment, m_cFaceVerticeCount, m_pFaceVertices);
		//std::cout << m_pFaceVertices[0].X << std::endl;
	}
	// �ɹ�
	if (SUCCEEDED(hr)) {
		for (UINT i = 0U; i < m_cFaceVerticeCount; ++i) {
			m_pMapper->MapCameraPointsToColorSpace(m_cFaceVerticeCount, m_pFaceVertices,
				imageRenderer->data.kinectFacePointsCount,
				const_cast<ColorSpacePoint*>(imageRenderer->data.kinectFacePoints)
			);
		}
	}
	// ��ȡ�沿��ת��Ԫ��
	if (SUCCEEDED(hr)) {
		hr = m_pFaceAlignment->get_FaceOrientation(&imageRenderer->data.kinectFaceRotation);
	}
	//��ȡͷ���е�
	if (SUCCEEDED(hr)) {
		hr = m_pFaceAlignment->get_HeadPivotPoint(&imageRenderer->data.headPivotPoint);
	}
	if (SUCCEEDED(hr)) {
		hr = m_pMapper->MapCameraPointToColorSpace(imageRenderer->data.headPivotPoint, &imageRenderer->data.headPivotPoint2D);
	}
	//// ��ȡAU����
	//if (SUCCEEDED(hr)) {
	//	hr = m_pFaceAlignment->GetAnimationUnits(FaceShapeAnimations_Count, imageRenderer->data.au);
	//}
	// ������
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
	// ��ȫ�ͷ�
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
