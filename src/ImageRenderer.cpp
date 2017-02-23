#include "ImageRenderer.h"

ImageRenderer::ImageRenderer() {
	//分配动态内存空间
	faceAnalyser = new CFaceAnalyser(paths);
	colorFrameData = new RGBQUAD[cColorHeight*cColorWidth];
	headRoiFilter = new CKalman;
	controlParamsFilter = new CKalman;
	message = new C_UDPSandG(8012, "127.0.0.1", 1024, UDPSend);

	faceControlParams = new float[faceControlParamsCount];
	smoothedFaceControlParams = new float[faceControlParamsCount];
	eyeState = new float[eyeParamsCount];
	controlParams = new ControlParam[controlParamsCount];
	//初始化
	headRoiFilter->init(4, 1, 0.5);
	controlParamsFilter->init(faceControlParamsCount, 1, 2);

	StartProcessingThread();
}

ImageRenderer::~ImageRenderer() {
	if (colorFrameData)
	{
		delete[] colorFrameData;
		colorFrameData = NULL;
	}

	delete faceAnalyser;
	delete headRoiFilter;
	delete colorFrameData;
	delete controlParamsFilter;
	delete message;

	delete[] faceControlParams;
	delete[] smoothedFaceControlParams;
	delete[] eyeState;

	Expire();
}

void ImageRenderer::showColorFrame(RGBQUAD* pBuffer)
{
	cv::Mat tmpFrame(cColorHeight, cColorWidth, CV_8UC4, pBuffer);
	cv::resize(tmpFrame, tmpFrame, cv::Size(cColorWidth / 2, cColorHeight / 2));

	cv::Mat tmpHeadFrame = tmpFrame(headRoi);
	if (!tmpFrame.empty()&&data.kinectFaceTracked) {
		//draw face landmarks
		faceAnalyser->drawLandmarks(tmpFrame);
		//draw gaze
		faceAnalyser->drawGaze(tmpFrame);
		//draw face box
		faceAnalyser->drawBox(tmpFrame);
	}
	cv::imshow("tmp frame", tmpFrame);
}

void ImageRenderer::showDepthFrame(RGBQUAD* m_pDepthRGBX, const UINT16 * pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	// Make sure we've received valid data
	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	{
		RGBQUAD* pRGBX = m_pDepthRGBX;
		int i = 0, j = 0;
		// end pixel is start + width*height - 1

		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
		const UINT16* pBufferM = pBuffer + (nWidth)*(nHeight / 2) + (nWidth / 2);

		while (pBuffer < pBufferEnd)
		{
			USHORT depth = *pBuffer;

			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).

			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.
			BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

			pRGBX->rgbRed = intensity;
			pRGBX->rgbGreen = intensity;
			pRGBX->rgbBlue = intensity;

			++i;
			if (i == nWidth)
			{
				i = 0;
				++j;
			}
			++pRGBX;
			++pBuffer;
		}

		// Draw the data with OpenCV

		cv::Mat DepthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
		cv::imshow("depth", DepthImage);
	}
}

void ImageRenderer::sendControlParams()
{
	smoothedFaceControlParams = controlParamsFilter->ProcessKalman(faceControlParams, faceControlParamsCount);

	for (int i = 0; i < JointType_Count; ++i) {
		controlParams[i].jointsData = joints[i];
	}

	for (int i = 0; i < faceControlParamsCount; ++i) {
		controlParams[i].value = smoothedFaceControlParams[i];
	}
	memcpy(message->buffer, controlParams, sizeof(ControlParam) * controlParamsCount);
	message->SendData(message->buffer, 1024);
}

void ImageRenderer::writeColorFrameData()
{
	if (colorFrameArrived) {
		cv::Mat tmpFrame(cColorHeight, cColorWidth, CV_8UC4, colorFrameData);
		//cv::resize(tmpFrame, colorFrame, cv::Size(cColorWidth / 2, cColorHeight / 2));
		colorFrame = tmpFrame.clone();
		colorFrameArrived = false;
	}
	
}

void ImageRenderer::getHeadROI() {
	if (data.kinectFaceTracked&&data.headPivotPoint.Z>0.5) {


		//headRoi.width = 400 / data.headPivotPoint.Z;
		headRoi.width = 300;
		headRoi.height = headRoi.width;
		headRoi.x = data.headPivotPoint2D.X - headRoi.width / 2;
		headRoi.y = data.headPivotPoint2D.Y - headRoi.height*3 / 5;
		//smoothedHeadRoi = headRoiFilter->ProcessKalman(headRoi);
		headRoi = headRoi & cv::Rect(0, 0, cColorWidth, cColorHeight);
		headFrame = colorFrame(headRoi);



		data.kinectFaceTracked = false;
	}
}

void ImageRenderer::processImage()
{
	writeColorFrameData();

	getHeadROI();

	if (!headFrame.empty())
	{
		*faceAnalyser >> headFrame;

		if (faceAnalyser->detectionSucess) {
			//draw face landmarks
			faceAnalyser->drawLandmarks(headFrame);
			//draw gaze
			faceAnalyser->drawGaze(headFrame);
			//draw face box
			faceAnalyser->drawBox(headFrame);
			//get AUs parameters
			faceAnalyser->getAUsParams(faceControlParams);
			//get head rotation
			faceAnalyser->getHeadRotation(data.headRotation);
			//get eye state
			faceAnalyser->getEyeState(eyeState);

			//process control parameters
			faceControlParams[auParamsCount] = data.kinectFaceRotation.w;
			faceControlParams[auParamsCount + 1] = data.kinectFaceRotation.x;
			faceControlParams[auParamsCount + 2] = data.kinectFaceRotation.y;
			faceControlParams[auParamsCount + 3] = data.kinectFaceRotation.z;

			for (int i = 0; i < eyeParamsCount; ++i) {
				faceControlParams[auParamsCount + headRotationParamsCount + i] = eyeState[i];
			}



			sendControlParams();


			//faceAnalyser->detectionSucess = false;
		}
		cv::imshow("head frame", headFrame);
	}
	cv::rectangle(colorFrame, headRoi, cv::Scalar(255, 0, 255));
	cv::imshow("img", colorFrame);

}


void ImageRenderer::StartProcessingThread() {
	if (expired_ == false) {
		//			std::cout << "timer is currently running, please expire it first..." << std::endl;
		return;
	}
	expired_ = false;
	std::thread([this]() {
		while (!try_to_expire_) {
			if (colorFrameArrived) {
				//cv::resize(colorFrame, colorFrame, cv::Size(960, 540));
				processImage();

				colorFrameArrived = false;

				if (cv::waitKey(1) == 27) break;
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

void ImageRenderer::Expire() {
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
