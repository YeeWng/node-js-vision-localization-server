#include <node.h>
#include <node_buffer.h>
#include <v8.h>
#include <uuid/uuid.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>
#include <sfm/sfm_data.hpp>
#include <sfm/sfm_data_io.hpp>
#include <openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp>
#include <openMVG/sfm/pipelines/sfm_matches_provider.hpp>

#include "LocalizeEngine.h"

using namespace std;
using namespace v8;

// TODO : all these file should be saved in database, and loaded when necessary
static const std::string SFM_DATA_DIR = "/home/ishihara/test/CMU-video-data-undistort/NSH-4F-QOLT/Output/merge_result/Output/SfM/reconstruction/global3";
static const std::string MATCH_DIR = "/home/ishihara/test/CMU-video-data-undistort/NSH-4F-QOLT/Output/merge_result/Output/matches";
static const std::string A_MAT_FILE = "/home/ishihara/test/CMU-video-data-undistort/NSH-4F-QOLT/Ref/Amat.yml";
static const std::string CAMERA_INTRINSIC_K_MAT_FILE = "/home/ishihara/test/CMU-video-data-undistort/NSH-4F-QOLT/Ref/K.yml";
static const std::string CAMERA_INTRINSIC_DIST_MAT_FILE = "/home/ishihara/test/CMU-video-data-undistort/NSH-4F-QOLT/Ref/dist.yml";

static const double SECOND_TEST_RATIO = 0.6;
static const int RANSAC_ROUND = 25;
static const double RANSAC_PRECISION = 4.0;

static const std::string TMP_DIR = "/tmp/vision-localize-server";

LocalizeEngine localizeEngine;
cv::Mat A; // map specific parameter, TODO : move to database
cv::Mat intrinsicK; // user specific parameter, TODO : move to database
cv::Mat intrinsicDist; // user speccific parameter, TODO : move to database

Local<Array> execLocalizeImage(const std::string& userID, const std::string& mapID, const cv::Mat& image,
		const std::vector<double>& center=std::vector<double>(), double radius=-1.0) {
	// TODO select 3D map for specified mapID here...
	string localizeTaskID;
	{
		uuid_t uuidValue;
		uuid_generate(uuidValue);
		char uuidChar[36];
		uuid_unparse_upper(uuidValue, uuidChar);
		localizeTaskID = string(uuidChar);
	}

	// create working directory
	string workingDir = stlplus::create_filespec(TMP_DIR, localizeTaskID);
	if (stlplus::folder_create(workingDir)) {
		cout << "successed to create working folder : " << workingDir << endl;
	} else {
		cout << "failed to create working folder : " << workingDir << endl;
		return Array::New(0);
	}
	{
		string command = "cp --remove-destination -s " + stlplus::create_filespec(MATCH_DIR, "*") + " " + workingDir;
		system(command.c_str());
	}

	// TODO select intrinsic camera parameters here for specified user here...
	cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(intrinsicK, intrinsicDist, cv::Size(image.cols, image.rows), 1.0, cv::Size(image.cols, image.rows));
	cv::Mat undistortImage;
	undistort(image, undistortImage, intrinsicK, intrinsicDist, newCameraMatrix);

	// execute localize
	vector<double> pos = localizeEngine.localize(image, workingDir, center, radius);

	// remove working directory
	if (stlplus::folder_delete(workingDir, true)) {
		cout << "successed to delete working folder : " << workingDir << endl;
	} else {
		cout << "failed to delete working folder : " << workingDir << endl;
	}

	// return result
	if (pos.size()==3) {
		Local<Array> result = Array::New(3);
		for (int i=0; i<3; i++) {
			result->Set(Number::New(i), Number::New(pos[i]));
		}

		return result;
	} else {
		return Array::New(0);
	}
}

Handle<Value> LocalizeImageBuffer(const Arguments& args) {
	HandleScope scope;

	if (args.Length() != 3 && args.Length() != 5) {
		ThrowException(
				Exception::TypeError(String::New("Wrong number of arguments")));
		return scope.Close(Undefined());
	}
	if (!args[0]->IsString() || !args[1]->IsString() || !args[2]->IsObject()) {
		ThrowException(Exception::TypeError(String::New("Wrong arguments")));
		return scope.Close(Undefined());
	}

	Local<String> userID = args[0]->ToString();
	char* userIDChar = *String::Utf8Value(userID);
	Local<String> mapID = args[1]->ToString();
	char* mapIDChar = *String::Utf8Value(mapID);
	Local<Object> imageBuffer = args[2]->ToObject();
	char* imageData    = node::Buffer::Data(imageBuffer);
	size_t imageDataLen = node::Buffer::Length(imageBuffer);
	cv::Mat image = cv::imdecode(cv::_InputArray(imageData, imageDataLen), cv::IMREAD_COLOR);
	Local<Array> result;
	if (args.Length()==3) {
		result = execLocalizeImage(std::string(userIDChar), std::string(mapIDChar), image);
	} else {
		Local<Array> center = Array::Cast(*args[3]);
		std::vector<double> centerVec;
	    for(int i = 0; i < center->Length(); i++) {
	    	centerVec.push_back(center->Get(i)->NumberValue());
	    }
	    double radius = args[4]->NumberValue();
	    result = execLocalizeImage(std::string(userIDChar), std::string(mapIDChar), image, centerVec, radius);
	}
	return scope.Close(result);
}

Handle<Value> LocalizeImagePath(const Arguments& args) {
	HandleScope scope;

	if (args.Length() != 3 && args.Length() != 5) {
		ThrowException(
				Exception::TypeError(String::New("Wrong number of arguments")));
		return scope.Close(Undefined());
	}
	if (!args[0]->IsString() || !args[1]->IsString() || !args[2]->IsString()) {
		ThrowException(Exception::TypeError(String::New("Wrong arguments")));
		return scope.Close(Undefined());
	}

	Local<String> userID = args[0]->ToString();
	char* userIDChar = *String::Utf8Value(userID);
	Local<String> mapID = args[1]->ToString();
	char* mapIDChar = *String::Utf8Value(mapID);
	Local<String> imagePath = args[2]->ToString();
	char* imagePathChar = *String::Utf8Value(imagePath);
	cv::Mat image = cv::imread(std::string(imagePathChar), cv::IMREAD_COLOR);
	Local<Array> result;
	if (args.Length() == 3) {
		result = execLocalizeImage(std::string(userIDChar), std::string(mapIDChar), image);
	} else {
		Local<Array> center = Array::Cast(*args[3]);
		std::vector<double> centerVec;
	    for(int i = 0; i < center->Length(); i++) {
	    	centerVec.push_back(center->Get(i)->NumberValue());
	    }
	    double radius = args[4]->NumberValue();
	    result = execLocalizeImage(std::string(userIDChar), std::string(mapIDChar), image, centerVec, radius);
	}
	return scope.Close(result);
}

void Init(Handle<Object> exports) {
	exports->Set(String::NewSymbol("localizeImageBuffer"),
			FunctionTemplate::New(LocalizeImageBuffer)->GetFunction());
	exports->Set(String::NewSymbol("localizeImagePath"),
			FunctionTemplate::New(LocalizeImagePath)->GetFunction());

	localizeEngine = LocalizeEngine(SFM_DATA_DIR, A_MAT_FILE, SECOND_TEST_RATIO, RANSAC_ROUND, RANSAC_PRECISION);

	{
		cv::FileStorage storage(CAMERA_INTRINSIC_K_MAT_FILE, cv::FileStorage::READ);
		storage["K"] >> intrinsicK;
		storage.release();
	}
	{
		cv::FileStorage storage(CAMERA_INTRINSIC_DIST_MAT_FILE, cv::FileStorage::READ);
		storage["dist"] >> intrinsicDist;
		storage.release();
	}
}

NODE_MODULE(localizeImage, Init)
