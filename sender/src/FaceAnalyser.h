#pragma once
#define WIN32_LEAN_AND_MEAN     // Exclude rarely-used stuff from Windows headers
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <math.h>

#include <LandmarkCoreIncludes.h>
#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>

class CFaceAnalyser
{
public:

	//default constructor
	CFaceAnalyser();

	//constructor from a model file
	explicit CFaceAnalyser(std::string path);

	//constructor from multi-models files
	explicit CFaceAnalyser(std::vector<std::string> paths);

	//copy constructor
	explicit CFaceAnalyser(const LandmarkDetector::CLNF& model);

	//assignment operator for lvalues(make a deep copy of the constructor)
	const CFaceAnalyser& operator=(const CFaceAnalyser& other);

	//process face analyser
	void operator >> (const cv::Mat& img);

	//convert landmarks to opencv point format
	const std::vector<cv::Point2f>& getLandmarks();

	//draw landmarks in image
	void drawLandmarks(cv::Mat& img) const;

	//draw gaze in image
	void drawGaze(cv::Mat& img) const;

	//draw face box in image
	void drawBox(cv::Mat& img) const;

	//write output file title and index
	void prepareOutputFile(ofstream* file);

	//output AUs intensity to csv format
	void outputAUs2csv(ofstream* file);

	//get AUs intensity sequence
	void getAUsParams(float* controlParams) const;

	//get gaze direction
	void getGazeDirection(cv::Point3f& gazeDrection,bool left) const;

	//get head rotation
	void getHeadRotation(cv::Point3f& headRotation) const;

	//get eye state
	void getEyeState(float* eyeState) const;

	bool										detectionSucess = false;
	bool										outputAUs = true;
private:
	//get AUs intensity
	void getAUs();

	//std::string								modelPath;
	LandmarkDetector::FaceModelParameters		detParameters;
	LandmarkDetector::CLNF						clnfModel;
	FaceAnalysis::FaceAnalyser					faceAnalyser;

	cv::Mat										grayscaleImg;

	cv::Mat_<double>							landmarks;
	std::vector<cv::Point2f>					cvLandmarks;

	// Grab camera parameters, if they are not defined (approximate values will be used)
	float										fx = 0, fy = 0, cx = 0, cy = 0;

	// If cx (optical axis centre) is undefined will use the image size/2 as an estimate
	bool										cx_undefined = false;
	bool										fx_undefined = false;

	// Gaze tracking, absolute gaze direction
	cv::Point3f									gazeDirection0;
	cv::Point3f									gazeDirection1;

	//AUs variable
	std::vector<std::pair<std::string, double>> auRegs;
	std::vector<std::pair<std::string, double>> ausClass;

	std::vector<std::string>					auRegNames;
	std::vector<std::string>					auClassNames;

	const int									ausCount = 14;
};