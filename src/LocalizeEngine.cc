#include "LocalizeEngine.h"

#include <uuid/uuid.h>
#include <omp.h>

#include <openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp>
#include <openMVG/sfm/pipelines/sfm_matches_provider.hpp>

#include "AKAZEOpenCV.h"
#include "FileUtils.h"
#include "MatchUtils.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;
using namespace openMVG::matching;

using namespace hulo;

#define MINUM_NUMBER_OF_POINT_PUTATIVE_MATCH 16
#define MINUM_NUMBER_OF_POINT_RESECTION 8
#define MINUM_NUMBER_OF_INLIER_RESECTION 10

LocalizeEngine::LocalizeEngine()
{
}

LocalizeEngine::LocalizeEngine(const std::string sfmDataDir, const std::string AmatFile,
		double secondTestRatio, int ransacRound, double ransacPrecision)
{
	mSfmDataDir = sfmDataDir;
	mSecondTestRatio = secondTestRatio;
	mRansacRound = ransacRound;
	mRansacPrecision = ransacPrecision;

	const string sSfM_data = stlplus::create_filespec(mSfmDataDir, "sfm_data", "json");
	cout << "Reading sfm_data.json file : " << sSfM_data << endl;
	if (!Load(mSfmData, sSfM_data,
			ESfM_Data(VIEWS | INTRINSICS | EXTRINSICS | STRUCTURE))) {
		cerr << endl << "The input sfm_data.json file \"" << sSfM_data << "\" cannot be read." << endl;
	}
	// get map of ViewID, FeatID to 3D point
	hulo::structureToMapViewFeatTo3D(mSfmData.GetLandmarks(), mMapViewFeatTo3D);

	cv::FileStorage storage(AmatFile, cv::FileStorage::READ);
	storage["A"] >> mA;
	storage.release();
}

void LocalizeEngine::extractAKAZESingleImg(std::string imageID, cv::Mat image, std::string outputFolder,
		const hulo::AKAZEOption &akazeOption, std::vector<std::pair<float, float>> &locFeat) {
	Ptr<AKAZE> akaze = AKAZE::create(
			AKAZE::DESCRIPTOR_MLDB, 0, akazeOption.desc_ch, akazeOption.thres,
			akazeOption.nOct, akazeOption.nOctLay);

	// write out to file
	string filebase = stlplus::create_filespec(outputFolder, imageID);
	string sFeat = filebase + ".feat";
	string sDesc = filebase + ".desc";

	// extract features
	vector<KeyPoint> kpts;
	cv::Mat desc;
	akaze->detectAndCompute(image, noArray(), kpts, desc);

	// write out
	ofstream fileFeat(sFeat);
	FileStorage fileDesc(sDesc, FileStorage::WRITE);

	if (fileFeat.is_open() && fileDesc.isOpened()) {
		// save each feature write each feature to file
		for (vector<KeyPoint>::const_iterator iterKP = kpts.begin();
				iterKP != kpts.end(); iterKP++) {
			locFeat.push_back(
					make_pair(static_cast<float>(iterKP->pt.x),
							static_cast<float>(iterKP->pt.y)));
			fileFeat << iterKP->pt.x << " " << iterKP->pt.y << " "
				<< iterKP->size << " " << iterKP->angle << endl;
		}
		fileFeat.close();

		// write descriptor to file
		if (SAVE_DESC_BINARY) {
			fileDesc.release();
			hulo::saveMatBin(sDesc, desc);
		} else {
			fileDesc << "Descriptors" << desc;
			fileDesc.release();
		}
	} else {
		cerr << "cannot open file to write features for " << filebase
				<< endl;
	}
}

void LocalizeEngine::getLocalViews(openMVG::sfm::SfM_Data &sfm_data, const cv::Mat &A, const cv::Mat &loc,
		const double radius, std::set<openMVG::IndexT> &localViews) {
	// iterate through sfm_data
	for (Views::const_iterator iter = sfm_data.views.begin();
			iter != sfm_data.views.end(); iter++) {
		bool bLocal = false;
		if (sfm_data.poses.find(iter->second->id_pose) != sfm_data.poses.end()) {
			Vec3 pose = sfm_data.poses.at(iter->second->id_pose).center();

			cv::Mat pose_h(4, 1, CV_64F);
			pose_h.at<double>(0) = pose[0];
			pose_h.at<double>(1) = pose[1];
			pose_h.at<double>(2) = pose[2];
			pose_h.at<double>(3) = 1.0;
			cv::Mat gpose_h = mA * pose_h;
			cv::Mat gpose(3, 1, CV_64F);
			gpose.at<double>(0) = gpose_h.at<double>(0);
			gpose.at<double>(0) = gpose_h.at<double>(1);
			gpose.at<double>(0) = gpose_h.at<double>(2);
			if (norm(gpose - loc) <= radius) {
				localViews.insert(iter->first);
				bLocal = true;
			}
		}
		if (bLocal) {
			sfm_data.views.erase(iter->first);
		}
	}
}

std::vector<double> LocalizeEngine::localize(const cv::Mat image, const std::string matchDir,
		const std::vector<double>& center, double radius)
{
	vector<double> pos;
	double startQuery = (double) getTickCount();

	//
	// copy original SfM_Data views to temporal SfM_Data
	// TODO :
	// This step can be avoided, but current putative matching and geometric matching implementation
	// assume both query and database views should be in the SfM_Data.
	// Currently, we need to copy SfM_Data to process multiple localization request at the same time.
	//
	SfM_Data querySfmData;
	for (Views::const_iterator iter = mSfmData.views.begin(); iter != mSfmData.views.end(); iter++) {
		querySfmData.views.insert(*iter);
	}
	for (Poses::const_iterator iter = mSfmData.poses.begin(); iter != mSfmData.poses.end(); iter++) {
		querySfmData.poses.insert(*iter);
	}
	double endReadSfmData = (double) getTickCount();

	// find restricted views if center and radius are specified
	set<IndexT> localViews;
	if (center.size()==3 && radius>0) {
		cv::Mat cenLoc(3, 1, CV_64FC1);
		cenLoc.at<double>(0) = center[0];
		cenLoc.at<double>(1) = center[1];
		cenLoc.at<double>(2) = center[2];

		cout << "select view : center = " << cenLoc << ", radius = " << radius << endl;
		getLocalViews(querySfmData, mA, cenLoc, radius, localViews);
		cout << "number of selected local views by center location : " << localViews.size() << endl;
		if (localViews.size()==0) {
			return pos;
		}
	}

	///////////////////////////////////////
	// Extract features from query image //
	///////////////////////////////////////
	cout << "Extract features from query image" << endl;
	// create and save image describer file
	string sImageDesc = stlplus::create_filespec(matchDir, "image_describer.txt");
	AKAZEOption akazeOption = AKAZEOption();
	akazeOption.read(sImageDesc);

	// generate ID of query image to store feature
	string imageID;
	{
		uuid_t uuidValue;
		uuid_generate(uuidValue);
		char uuidChar[36];
		uuid_unparse_upper(uuidValue, uuidChar);
		imageID = string(uuidChar);
	}
	int imageHeight = image.rows;
	int imageWidth = image.cols;
	cout << "image ID :" << imageID << endl;
	cout << "image size : " << image.size() << endl;

	vector<pair<float, float>> qFeatLoc;
	extractAKAZESingleImg(imageID, image, matchDir, akazeOption, qFeatLoc);
	double endFeat = (double) getTickCount();

	/////////////////////////////////////////
	// Match features to those in SfM file //
	/////////////////////////////////////////
	// add query image to sfm_data
	IndexT indQueryFile = querySfmData.views.rbegin()->second->id_view + 1; // set index of query file to (id of last view of sfm data) + 1
	cout << "query image size : " << imageWidth << " x " << imageHeight << endl;
	querySfmData.views.insert(make_pair(indQueryFile, make_shared<View>(imageID, indQueryFile, 0, 0, imageWidth, imageHeight)));
	cout << "query image index : " << indQueryFile << endl;
	cout << "query image path : " << querySfmData.views.at(indQueryFile)->s_Img_path << endl;

	// generate pairs
	vector<size_t> pairs;
	if (localViews.size()==0) {
		// use all views
		for (Views::const_iterator iter = querySfmData.views.begin(); iter != querySfmData.views.end(); iter++) {
			pairs.push_back(iter->second->id_view);
		}
	} else {
		// restrict views
		for (Views::const_iterator iter = querySfmData.views.begin(); iter != querySfmData.views.end(); iter++) {
			if (localViews.find(iter->second->id_view) != localViews.end()) {
				pairs.push_back(iter->second->id_view);
			}
		}
	}
	cout << "size pairs: " << pairs.size() << endl;

	// perform putative matching
	PairWiseMatches map_putativeMatches;
	map<pair<size_t, size_t>, map<size_t, int>> featDist;
	hulo::matchAKAZEToQuery(querySfmData, matchDir, pairs, indQueryFile, mSecondTestRatio, map_putativeMatches, featDist);
	// the indices in above image is based on vec_fileNames, not Viewf from sfm_data!!!
	double endMatch = (double) getTickCount();

	// remove putative matches with fewer points
	for (auto iterMatch = map_putativeMatches.cbegin(); iterMatch != map_putativeMatches.cend();) {
		if (iterMatch->second.size() < MINUM_NUMBER_OF_POINT_PUTATIVE_MATCH) {
			map_putativeMatches.erase(iterMatch++);
		} else {
			++iterMatch;
		}
	}
	cout << "number of putative matches : " << map_putativeMatches.size() << endl;

	// exit if found no match
	if (map_putativeMatches.size() == 0) {
		cout << "Not enough putative matches" << endl;

		cout << "Read sfm_data: " << (endReadSfmData - startQuery) / getTickFrequency() << " s\n";
		cout << "Extract feature from query image : " << (endFeat - endReadSfmData) / getTickFrequency() << " s\n";
		cout << "Putative matching : " << (endMatch - endFeat) / getTickFrequency() << " s\n";

		return pos;
	}

	//// compute geometric matches
	PairWiseMatches map_geometricMatches;
	hulo::geometricMatch(querySfmData, matchDir, map_putativeMatches,
					map_geometricMatches, mRansacRound, mRansacPrecision);

	double endGMatch = (double) getTickCount();
	cout << "number of geometric matches : " << map_geometricMatches.size() << endl;

	// exit if found no match
	if (map_geometricMatches.size() == 0) {
		cout << "Not enough geometric matches" << endl;
		return pos;
	}

	shared_ptr<Matches_Provider> matches_provider = make_shared<Matches_Provider>();
	matches_provider->_pairWise_matches = map_geometricMatches;

	// build list of tracks
	cout << "Match provider size " << matches_provider->_pairWise_matches.size() << endl;

	// intersect tracks from query image to reconstructed 3D
	// any 2D pts that have been matched to more than one 3D point will be removed
	// in matchProviderToMatchSet
	cout << "map to 3D size = " << mMapViewFeatTo3D.size() << endl;
	QFeatTo3DFeat mapFeatTo3DFeat;
	hulo::matchProviderToMatchSet(matches_provider, mMapViewFeatTo3D, mapFeatTo3DFeat, featDist);
	cout << "mapFeatTo3DFeat size = " << mapFeatTo3DFeat.size() << endl;
	// matrices for saving 2D points from query image and 3D points from reconstructed model
	openMVG::Mat pt2D(2, mapFeatTo3DFeat.size());
	openMVG::Mat pt3D(3, mapFeatTo3DFeat.size());

	// get intrinsic
	const Intrinsics::const_iterator iterIntrinsic_I = mSfmData.GetIntrinsics().find(0);
	Pinhole_Intrinsic *cam_I = dynamic_cast<Pinhole_Intrinsic*>(iterIntrinsic_I->second.get());
	Mat3 K = cam_I->K();
	cout << "K = " << K << endl;

	// copy data to matrices
	size_t cpt = 0;
	for (QFeatTo3DFeat::const_iterator iterfeatId = mapFeatTo3DFeat.begin();
			iterfeatId != mapFeatTo3DFeat.end(); ++iterfeatId, ++cpt) {

		// copy 3d location
		pt3D.col(cpt) = mSfmData.GetLandmarks().at(iterfeatId->second).X;

		// copy 2d location
		const Vec2 feat = { qFeatLoc[iterfeatId->first].first,
				qFeatLoc[iterfeatId->first].second };
		pt2D.col(cpt) = cam_I->get_ud_pixel(feat);
	}
	cout << "cpt = " << cpt << endl;

	// perform resection
	vector<size_t> vec_inliers;
	Mat34 P;
	double errorMax = numeric_limits<double>::max();
	bool bResection = false;
	if (cpt > MINUM_NUMBER_OF_POINT_RESECTION) {
		bResection = robustResection(make_pair(imageHeight, imageWidth), pt2D, pt3D, &vec_inliers, &K, &P, &errorMax);
	}

	if (!bResection || vec_inliers.size() <= MINUM_NUMBER_OF_INLIER_RESECTION) {
		cout << "Fail to estimate camera matrix" << endl;
		cout << "#inliers = " << vec_inliers.size() << endl;

		cout << "Read sfm_data: " << (endReadSfmData - startQuery) / getTickFrequency() << " s\n";
		cout << "Extract feature from query image : " << (endFeat - endReadSfmData) / getTickFrequency() << " s\n";
		cout << "Putative matching : " << (endMatch - endFeat) / getTickFrequency() << " s\n";
		cout << "Geometric matching : " << (endGMatch - endMatch) / getTickFrequency() << " s\n";

		return pos;
	}

	Mat3 K_, R_;
	Vec3 t_, t_out;
	KRt_From_P(P, &K_, &R_, &t_);
	t_out = -R_.transpose() * t_;
	cout << "#inliers = " << vec_inliers.size() << endl;
	cout << "P = " << endl << P << endl;
	cout << "K = " << endl << K_ << endl;
	cout << "R = " << endl << R_ << endl;
	cout << "t = " << endl << t_ << endl;
	cout << "-R't = " << endl << -R_.transpose() * t_ << endl;

	cv::Mat t_out_h(4, 1, CV_64F);
	t_out_h.at<double>(0) = t_out[0];
	t_out_h.at<double>(1) = t_out[1];
	t_out_h.at<double>(2) = t_out[2];
	t_out_h.at<double>(3) = 1.0;
	cv::Mat global_t_out = mA * t_out_h;
	cout << "global t = " << endl << global_t_out << endl;

	double endQuery = (double) getTickCount();
	cout << "Read sfm_data: " << (endReadSfmData - startQuery) / getTickFrequency() << " s\n";
	cout << "Extract feature from query image : " << (endFeat - endReadSfmData) / getTickFrequency() << " s\n";
	cout << "Putative matching : " << (endMatch - endFeat) / getTickFrequency() << " s\n";
	cout << "Geometric matching : " << (endGMatch - endMatch) / getTickFrequency() << " s\n";
	cout << "PnP: " << (endQuery - endGMatch) / getTickFrequency() << " s\n";
	cout << "Total time: " << (endQuery - startQuery) / getTickFrequency() << " s\n";

	cout << "complete!" << endl;

	pos.push_back(global_t_out.at<double>(0));
	pos.push_back(global_t_out.at<double>(1));
	pos.push_back(global_t_out.at<double>(2));
	return pos;
}
