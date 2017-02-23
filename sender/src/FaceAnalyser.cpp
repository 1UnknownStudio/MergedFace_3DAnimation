#include "FaceAnalyser.h"


CFaceAnalyser::CFaceAnalyser()
{
	detParameters.track_gaze = true;
	//load model files
	detParameters.model_location = "E:/Program Files/OpenFaceLib/model/main_clnf_general.txt";
	clnfModel.Read(detParameters.model_location);
}

CFaceAnalyser::CFaceAnalyser(std::string path) :clnfModel(path)
{
	detParameters.track_gaze = true;
}

CFaceAnalyser::CFaceAnalyser(const LandmarkDetector::CLNF& model) : clnfModel(model)
{
	detParameters.track_gaze = true;
}

CFaceAnalyser::CFaceAnalyser(std::vector<std::string> paths) : clnfModel(paths[0]),
faceAnalyser(std::vector<cv::Vec3d>(), 0.7, 112, 112, paths[1], paths[2])
{
	detParameters.track_gaze = true;
}

const CFaceAnalyser& CFaceAnalyser::operator=(const CFaceAnalyser& other)
{
	if (this != &other) {
		this->clnfModel = other.clnfModel;
		this->cvLandmarks = other.cvLandmarks;
		this->detectionSucess = other.detectionSucess;
		this->detParameters = other.detParameters;
		this->landmarks = other.landmarks.clone();
		this->grayscaleImg = other.grayscaleImg.clone();
		//TODO:
	}
	return *this;
}

void CFaceAnalyser::operator >> (const cv::Mat& img)
{
	if (cx == 0 || cy == 0)
	{
		cx_undefined = true;
	}
	if (fx == 0 || fy == 0)
	{
		fx_undefined = true;
	}
	// If optical centers are not defined just use center of image
	if (cx_undefined)
	{
		cx = img.cols / 2.0f;
		cy = img.rows / 2.0f;
	}
	// Use a rough guess-timate of focal length
	if (fx_undefined)
	{
		fx = 500 * (img.cols / 640.0);
		fy = 500 * (img.rows / 480.0);

		fx = (fx + fy) / 2.0;
		fy = fx;
	}
	//convert origin image format to grayscale
	if (img.channels() == 3) {
		cv::cvtColor(img, grayscaleImg, CV_BGR2GRAY);
	}
	else if (img.channels() == 1) {
		grayscaleImg = img.clone();
	}
	else if (img.channels() == 4) {
		cv::cvtColor(img, grayscaleImg, CV_BGRA2GRAY);
	}
	//detect facial landmarks
	detectionSucess = LandmarkDetector::DetectLandmarksInVideo(grayscaleImg, clnfModel, detParameters);
	if (detectionSucess) {
		landmarks = clnfModel.detected_landmarks;
		if (detParameters.track_gaze && clnfModel.eye_model) {
			FaceAnalysis::EstimateGaze(clnfModel, gazeDirection0, fx, fy, cx, cy, true);
			FaceAnalysis::EstimateGaze(clnfModel, gazeDirection1, fx, fy, cx, cy, false);


			faceAnalyser.AddNextFrame(grayscaleImg, clnfModel, 10, false, false);

			//get AUs intensity
			getAUs();

			getLandmarks();
		}
	}
}

const std::vector<cv::Point2f>& CFaceAnalyser::getLandmarks()
{
	cvLandmarks.clear();
	int points_num = landmarks.rows / 2;
	if (points_num == 0) {
		std::cout << "no landmarks detected!" << std::endl;
		return cvLandmarks;
	}
	else {
		for (int i = 0; i < points_num; ++i) {
			cvLandmarks.push_back(cv::Point2f((float)landmarks.at<double>(i, 0), (float)landmarks.at<double>(points_num + i, 0)));
		}
		return cvLandmarks;
	}
}

void CFaceAnalyser::getAUs()
{
	auRegNames = faceAnalyser.GetAURegNames();
	std::sort(auRegNames.begin(), auRegNames.end());

	auClassNames = faceAnalyser.GetAUClassNames();
	std::sort(auClassNames.begin(), auClassNames.end());

	//get AUs intensity
	auRegs = faceAnalyser.GetCurrentAUsReg();
	ausClass = faceAnalyser.GetCurrentAUsClass();
	//get AUs names
	std::vector<std::string> auRegName = faceAnalyser.GetAURegNames();
	std::sort(auRegName.begin(), auRegName.end());


}

void CFaceAnalyser::drawLandmarks(cv::Mat& img) const
{
	for (auto p : cvLandmarks) {
		// A rough heuristic for drawn point size
		int thickness = (int)std::ceil(3.0* ((double)img.cols) / 640.0);
		int thickness_2 = (int)std::ceil(1.0* ((double)img.cols) / 640.0);

		cv::circle(img, p, 1, cv::Scalar(0, 0, 255), thickness);
		cv::circle(img, p, 1, cv::Scalar(255, 0, 0), thickness_2);
	}
}

void CFaceAnalyser::drawGaze(cv::Mat& img) const
{
	if (detParameters.track_gaze && detectionSucess && clnfModel.eye_model) {
		FaceAnalysis::DrawGaze(img, clnfModel, gazeDirection0, gazeDirection1, fx, fy, cx, cy);
		//std::cout << "left eye:" << gazeDirection0.x * 180 / M_PI << "," << gazeDirection0.y * 180 / M_PI << "," << gazeDirection0.z * 180 / M_PI << std::endl;
		//std::cout << "right eye:" << gazeDirection1.x * 180 / M_PI << "," << gazeDirection1.y * 180 / M_PI << "," << gazeDirection1.z * 180 / M_PI << std::endl;

	}
}

void CFaceAnalyser::drawBox(cv::Mat& img) const {
	// A rough heuristic for box around the face width
	int thickness = (int)std::ceil(2.0* ((double)img.cols) / 640.0);

	cv::Vec6d pose_estimate_to_draw = LandmarkDetector::GetCorrectedPoseWorld(clnfModel, fx, fy, cx, cy);

	// Draw it in reddish if uncertain, blueish if certain
	LandmarkDetector::DrawBox(img, pose_estimate_to_draw, cv::Scalar(255.0, 0, 255), thickness, fx, fy, cx, cy);
}

void CFaceAnalyser::prepareOutputFile(ofstream* file)
{
	auRegNames = faceAnalyser.GetAURegNames();
	std::sort(auRegNames.begin(), auRegNames.end());

	auClassNames = faceAnalyser.GetAUClassNames();
	std::sort(auClassNames.begin(), auClassNames.end());

	for (string regName : auRegNames)
	{
		*file << ", " << regName << "_r";
	}

	for (string className : auClassNames)
	{
		*file << ", " << className << "_c";
	}
	*file << std::endl;
}

void CFaceAnalyser::outputAUs2csv(ofstream * file)
{
	if (!file->is_open()) {
		std::cerr << "Error in opening csv file!" << std::endl;
		return;
	}

	// write out au the correct index
	for (string auName : auRegNames)
	{
		for (auto auReg : auRegs)
		{
			if (auName.compare(auReg.first) == 0)
			{
				*file << ", " << auReg.second;
				break;
			}
		}
	}
	if (auRegs.size() == 0)
	{
		for (size_t p = 0; p < faceAnalyser.GetAURegNames().size(); ++p)
		{
			*file << ", 0";
		}
	}


	// write out au the correct index
	for (string auName : auClassNames)
	{
		for (auto auClass : ausClass)
		{
			if (auName.compare(auClass.first) == 0)
			{
				*file << ", " << auClass.second;
				break;
			}
		}
	}

	if (ausClass.size() == 0)
	{
		for (size_t p = 0; p < faceAnalyser.GetAUClassNames().size(); ++p)
		{
			*file << ", 0";
		}
	}

	*file << std::endl;
}

void CFaceAnalyser::getAUsParams(float* controlParams) const
{
	if (auRegs.size() == ausCount && auRegNames.size() == ausCount) {
		// write out au the correct index
		int i = 0;
		for (std::string auName : auRegNames)
		{
			for (auto auReg : auRegs)
			{
				if (auName.compare(auReg.first) == 0)
				{
					controlParams[i] = auReg.second * 30;
					++i;
					break;
				}

			}
		}

	}
}

void CFaceAnalyser::getGazeDirection(cv::Point3f& gazeDrection, bool left) const
{
	if (detectionSucess) {
		if (left) {
			gazeDrection = gazeDirection0;
		}
		else
		{
			gazeDrection = gazeDirection1;
		}


	}
}

void CFaceAnalyser::getHeadRotation(cv::Point3f& headRotation) const
{
	if (detectionSucess) {
		cv::Vec6d pose_estimate = LandmarkDetector::GetCorrectedPoseWorld(clnfModel, fx, fy, cx, cy);
		headRotation = cv::Point3f(pose_estimate[3], pose_estimate[4], pose_estimate[5]);
	}
}

void CFaceAnalyser::getEyeState(float* eyeState) const
{
	for (auto auClass : ausClass) {
		if (auClassNames[auClassNames.size() - 1].compare(auClass.first) == 0) {
			//std::cout << auClass.first<<":"<<auClass.second << std::endl;
			eyeState[0] = auClass.second * 120;
		}
	}
	//TODO:eyeball control
}
