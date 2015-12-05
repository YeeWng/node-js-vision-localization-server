#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <sfm/sfm_data.hpp>
#include <sfm/sfm_data_io.hpp>
#include <openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp>
#include <openMVG/sfm/pipelines/sfm_matches_provider.hpp>

#include "AKAZEOption.h"
#include "SfMDataUtils.h"

class LocalizeEngine {
private:
	std::string mSfmDataDir;
	double mSecondTestRatio;
	int mRansacRound;
	double mRansacPrecision;
	openMVG::sfm::SfM_Data mSfmData;
	hulo::MapViewFeatTo3D mMapViewFeatTo3D;
	cv::Mat mA;

	void getLocalViews(openMVG::sfm::SfM_Data &sfm_data, const cv::Mat &A, const cv::Mat &loc,
			const double radius, std::set<openMVG::IndexT> &localViews);

	void extractAKAZESingleImg(std::string imageID, cv::Mat image, std::string outputFolder,
			const hulo::AKAZEOption &akazeOption, std::vector<std::pair<float, float>> &locFeat);

public:
	LocalizeEngine();
	LocalizeEngine(const std::string sfmDataDir, const std::string AmatFile, double secondTestRatio,
			int ransacRound, double ransacPrecision);

	std::vector<double> localize(const cv::Mat image, const std::string matchDir,
			const std::vector<double>& center=std::vector<double>(), double radius=-1.0);
};
