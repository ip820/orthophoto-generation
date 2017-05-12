// for orthophoto generation
#include "ortho.h"
#include <stdio.h>
#include <Windows.h>
#include <WinInet.h>

#include <ctime>
#include "stdafx.h"
#include "ApxModifier.h"
#include <string.h>
#include <iostream>
#include <chrono>
////
#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

#include <Windows.h>
#include <WinInet.h>

#include <algorithm>

#define ENABLE_LOG 1
#define LOG(msg) std::cout << msg
#define LOGLN(msg) std::cout << msg << std::endl

using namespace std;
using namespace cv;
using namespace cv::detail;

vector<String> img_names;
//bool preview = false;
bool try_cuda = false;
double work_megapix = 0.3;
double seam_megapix = 0.1;
double compose_megapix = -1;
float conf_thresh = 1.f;
string features_type = "surf";
string matcher_type = "homography";
string estimator_type = "homography";
string ba_cost_func = "ray";
string ba_refine_mask = "xxxxx";
bool do_wave_correct = true;
WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;
bool save_graph = false;
std::string save_graph_to;
string warp_type = "affine";
int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
float match_conf = 0.3f;
string seam_find_type = "dp_color";
int blend_type = Blender::MULTI_BAND;
int timelapse_type = Timelapser::AS_IS;
float blend_strength = 5;
string result_name = "Stitching.jpg";
bool timelapse = false;
int range_width = -1;

typedef struct WData
{
	//string imagename;
	double world[6];
}worldData;

double GetMinValue_X(vector<worldData> wFile);
double GetMaxValue_Y(vector<worldData> wFile);

void LogFile(char* filename, int st, char* st_hms, int et, char* et_hms, int pt)
{
	FILE* pFile = NULL;
	fopen_s(&pFile, "C:\\Log_file/TimeOrthophoto.csv", "a+");
	fseek(pFile, 0, SEEK_SET);

	int nLine = 1;
	while (!feof(pFile))
	{
		if (fgetc(pFile) == 10) // 라인 끝에 도착 Line 증가
		{
			nLine++;
		}
	}
	fprintf(pFile, "%s,%.3f,'%s,%.3f,'%s,%.3f\n", filename, double(st*0.001), st_hms, double(et*0.001), et_hms, double((pt)*0.001));
	fclose(pFile);
}

int stitching()
{
	SetCurrentDirectory("C://uav_image2/");

	WIN32_FIND_DATAA find_data;
	HANDLE find_file;
	char file_name[500][500];
	int x = 0;

	//처리할 영상에 대한 영상 위치자세 데이터의 파일명 읽어오기
	find_file = ::FindFirstFile("*.png", &find_data);
	if (find_file != INVALID_HANDLE_VALUE) {
		do {
			if (find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY);
			else
			{
				char *c = find_data.cFileName;
				printf("%s \n", c);

				strcpy_s(file_name[x], c);
				printf("");
				x++;	// the number of images
			};
		} while (::FindNextFile(find_file, &find_data));

		::FindClose(find_file);
	}
	for (int imgN = 0; imgN<x; imgN++)
	{
		img_names.push_back(file_name[imgN]);//image 받기
	}

	// Check if have enough images
	int num_images = static_cast<int>(img_names.size());

	double work_scale = 1, seam_scale = 1, compose_scale = 1;
	bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;

	Ptr<FeaturesFinder> finder;
	if (features_type == "surf")
	{
#ifdef HAVE_OPENCV_XFEATURES2D
		if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
			finder = makePtr<SurfFeaturesFinderGpu>();
		else
#endif
			finder = makePtr<SurfFeaturesFinder>();
	}
	else if (features_type == "orb")
	{
		finder = makePtr<OrbFeaturesFinder>();
	}
	else
	{
		cout << "Unknown 2D features type: '" << features_type << "'.\n";
		return -1;
	}

	Mat full_img, img;
	vector<ImageFeatures> features(num_images);
	vector<Mat> images(num_images);
	vector<Size> full_img_sizes(num_images);
	double seam_work_aspect = 1;

	for (int i = 0; i < num_images; ++i)
	{
		full_img = imread(img_names[i]);
		full_img_sizes[i] = full_img.size();

		if (full_img.empty())
		{
			LOGLN("Can't open image " << img_names[i]);
			return -1;
		}
		if (work_megapix < 0)
		{
			img = full_img;
			work_scale = 1;
			is_work_scale_set = true;
		}
		else
		{
			if (!is_work_scale_set)
			{
				work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
				is_work_scale_set = true;
			}
			resize(full_img, img, Size(), work_scale, work_scale);
		}
		if (!is_seam_scale_set)
		{
			seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
			seam_work_aspect = seam_scale / work_scale;
			is_seam_scale_set = true;
		}

		(*finder)(img, features[i]);
		features[i].img_idx = i;
		LOGLN("Features in image #" << i + 1 << ": " << features[i].keypoints.size());

		resize(full_img, img, Size(), seam_scale, seam_scale);
		images[i] = img.clone();
	}

	finder->collectGarbage();
	full_img.release();
	img.release();

	LOG("Pairwise matching");
#
	vector<MatchesInfo> pairwise_matches;
	Ptr<FeaturesMatcher> matcher;
	if (matcher_type == "affine")
		matcher = makePtr<AffineBestOf2NearestMatcher>(false, try_cuda, match_conf);
	else if (range_width == -1)
		matcher = makePtr<BestOf2NearestMatcher>(try_cuda, match_conf);
	else
		matcher = makePtr<BestOf2NearestRangeMatcher>(range_width, try_cuda, match_conf);

	(*matcher)(features, pairwise_matches);
	matcher->collectGarbage();

	// Check if we should save matches graph
	if (save_graph)
	{
		LOGLN("Saving matches graph...");
		ofstream f(save_graph_to.c_str());
		f << matchesGraphAsString(img_names, pairwise_matches, conf_thresh);
	}

	// Leave only images we are sure are from the same panorama
	vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
	vector<Mat> img_subset;
	vector<String> img_names_subset;
	vector<Size> full_img_sizes_subset;
	for (size_t i = 0; i < indices.size(); ++i)
	{
		img_names_subset.push_back(img_names[indices[i]]);
		img_subset.push_back(images[indices[i]]);
		full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
	}

	images = img_subset;
	img_names = img_names_subset;
	full_img_sizes = full_img_sizes_subset;

	// Check if we still have enough images
	num_images = static_cast<int>(img_names.size());
	if (num_images < 2)
	{
		LOGLN("Need more images");
		return -1;
	}

	Ptr<Estimator> estimator;
	if (estimator_type == "affine")
		estimator = makePtr<AffineBasedEstimator>();
	else
		estimator = makePtr<HomographyBasedEstimator>();

	vector<CameraParams> cameras;
	if (!(*estimator)(features, pairwise_matches, cameras))
	{
		cout << "Homography estimation failed.\n";
		return -1;
	}

	for (size_t i = 0; i < cameras.size(); ++i)
	{
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R;
		LOGLN("Initial camera intrinsics #" << indices[i] + 1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R);
	}

	Ptr<detail::BundleAdjusterBase> adjuster;
	if (ba_cost_func == "reproj") adjuster = makePtr<detail::BundleAdjusterReproj>();
	else if (ba_cost_func == "ray") adjuster = makePtr<detail::BundleAdjusterRay>();
	else if (ba_cost_func == "affine") adjuster = makePtr<detail::BundleAdjusterAffinePartial>();
	else if (ba_cost_func == "no") adjuster = makePtr<NoBundleAdjuster>();
	else
	{
		cout << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
		return -1;
	}
	adjuster->setConfThresh(conf_thresh);
	Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
	if (ba_refine_mask[0] == 'x') refine_mask(0, 0) = 1;
	if (ba_refine_mask[1] == 'x') refine_mask(0, 1) = 1;
	if (ba_refine_mask[2] == 'x') refine_mask(0, 2) = 1;
	if (ba_refine_mask[3] == 'x') refine_mask(1, 1) = 1;
	if (ba_refine_mask[4] == 'x') refine_mask(1, 2) = 1;
	adjuster->setRefinementMask(refine_mask);
	if (!(*adjuster)(features, pairwise_matches, cameras))
	{
		cout << "Camera parameters adjusting failed.\n";
		return -1;
	}

	// Find median focal length

	vector<double> focals;
	for (size_t i = 0; i < cameras.size(); ++i)
	{
		LOGLN("Camera #" << indices[i] + 1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R);
		focals.push_back(cameras[i].focal);
	}

	sort(focals.begin(), focals.end());
	float warped_image_scale;
	if (focals.size() % 2 == 1)
		warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
	else
		warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

	if (do_wave_correct)
	{
		vector<Mat> rmats;
		for (size_t i = 0; i < cameras.size(); ++i)
			rmats.push_back(cameras[i].R.clone());
		waveCorrect(rmats, wave_correct);
		for (size_t i = 0; i < cameras.size(); ++i)
			cameras[i].R = rmats[i];
	}

	LOGLN("Warping images (auxiliary)... ");


	vector<Point> corners(num_images);
	vector<UMat> masks_warped(num_images);
	vector<UMat> images_warped(num_images);
	vector<Size> sizes(num_images);
	vector<UMat> masks(num_images);

	// Preapre images masks
	for (int i = 0; i < num_images; ++i)
	{
		masks[i].create(images[i].size(), CV_8U);
		masks[i].setTo(Scalar::all(255));
	}

	// Warp images and their masks

	Ptr<WarperCreator> warper_creator;
#ifdef HAVE_OPENCV_CUDAWARPING
	if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
	{
		if (warp_type == "plane")
			warper_creator = makePtr<cv::PlaneWarperGpu>();
		else if (warp_type == "cylindrical")
			warper_creator = makePtr<cv::CylindricalWarperGpu>();
		else if (warp_type == "spherical")
			warper_creator = makePtr<cv::SphericalWarperGpu>();
	}
	else
#endif
	{
		if (warp_type == "plane")
			warper_creator = makePtr<cv::PlaneWarper>();
		else if (warp_type == "affine")
			warper_creator = makePtr<cv::AffineWarper>();
		else if (warp_type == "cylindrical")
			warper_creator = makePtr<cv::CylindricalWarper>();
		else if (warp_type == "spherical")
			warper_creator = makePtr<cv::SphericalWarper>();
		else if (warp_type == "fisheye")
			warper_creator = makePtr<cv::FisheyeWarper>();
		else if (warp_type == "stereographic")
			warper_creator = makePtr<cv::StereographicWarper>();
		else if (warp_type == "compressedPlaneA2B1")
			warper_creator = makePtr<cv::CompressedRectilinearWarper>(2.0f, 1.0f);
		else if (warp_type == "compressedPlaneA1.5B1")
			warper_creator = makePtr<cv::CompressedRectilinearWarper>(1.5f, 1.0f);
		else if (warp_type == "compressedPlanePortraitA2B1")
			warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(2.0f, 1.0f);
		else if (warp_type == "compressedPlanePortraitA1.5B1")
			warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(1.5f, 1.0f);
		else if (warp_type == "paniniA2B1")
			warper_creator = makePtr<cv::PaniniWarper>(2.0f, 1.0f);
		else if (warp_type == "paniniA1.5B1")
			warper_creator = makePtr<cv::PaniniWarper>(1.5f, 1.0f);
		else if (warp_type == "paniniPortraitA2B1")
			warper_creator = makePtr<cv::PaniniPortraitWarper>(2.0f, 1.0f);
		else if (warp_type == "paniniPortraitA1.5B1")
			warper_creator = makePtr<cv::PaniniPortraitWarper>(1.5f, 1.0f);
		else if (warp_type == "mercator")
			warper_creator = makePtr<cv::MercatorWarper>();
		else if (warp_type == "transverseMercator")
			warper_creator = makePtr<cv::TransverseMercatorWarper>();
	}

	if (!warper_creator)
	{
		cout << "Can't create the following warper '" << warp_type << "'\n";
		return 1;
	}

	Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

	for (int i = 0; i < num_images; ++i)
	{
		Mat_<float> K;
		cameras[i].K().convertTo(K, CV_32F);
		float swa = (float)seam_work_aspect;
		K(0, 0) *= swa; K(0, 2) *= swa;
		K(1, 1) *= swa; K(1, 2) *= swa;

		corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
		sizes[i] = images_warped[i].size();

		warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
	}

	vector<UMat> images_warped_f(num_images);
	for (int i = 0; i < num_images; ++i)
		images_warped[i].convertTo(images_warped_f[i], CV_32F);

	Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
	compensator->feed(corners, images_warped, masks_warped);

	Ptr<SeamFinder> seam_finder;
	if (seam_find_type == "no")
		seam_finder = makePtr<detail::NoSeamFinder>();
	else if (seam_find_type == "voronoi")
		seam_finder = makePtr<detail::VoronoiSeamFinder>();
	else if (seam_find_type == "gc_color")
	{
#ifdef HAVE_OPENCV_CUDALEGACY
		if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
			seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR);
		else
#endif
			seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
	}
	else if (seam_find_type == "gc_colorgrad")
	{
#ifdef HAVE_OPENCV_CUDALEGACY
		if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
			seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
		else
#endif
			seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
	}
	else if (seam_find_type == "dp_color")
		seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR);
	else if (seam_find_type == "dp_colorgrad")
		seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
	if (!seam_finder)
	{
		cout << "Can't create the following seam finder '" << seam_find_type << "'\n";
		return 1;
	}

	seam_finder->find(images_warped_f, corners, masks_warped);

	// Release unused memory
	images.clear();
	images_warped.clear();
	images_warped_f.clear();
	masks.clear();

	LOGLN("Compositing...");


	Mat img_warped, img_warped_s;
	Mat dilated_mask, seam_mask, mask, mask_warped;
	Ptr<Blender> blender;
	Ptr<Timelapser> timelapser;
	//double compose_seam_aspect = 1;
	double compose_work_aspect = 1;

	for (int img_idx = 0; img_idx < num_images; ++img_idx)
	{
		LOGLN("Compositing image #" << indices[img_idx] + 1);

		// Read image and resize it if necessary
		full_img = imread(img_names[img_idx]);
		if (!is_compose_scale_set)
		{
			if (compose_megapix > 0)
				compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / full_img.size().area()));
			is_compose_scale_set = true;

			// Compute relative scales
			//compose_seam_aspect = compose_scale / seam_scale;
			compose_work_aspect = compose_scale / work_scale;

			// Update warped image scale
			warped_image_scale *= static_cast<float>(compose_work_aspect);
			warper = warper_creator->create(warped_image_scale);

			// Update corners and sizes
			for (int i = 0; i < num_images; ++i)
			{
				// Update intrinsics
				cameras[i].focal *= compose_work_aspect;
				cameras[i].ppx *= compose_work_aspect;
				cameras[i].ppy *= compose_work_aspect;

				// Update corner and size
				Size sz = full_img_sizes[i];
				if (std::abs(compose_scale - 1) > 1e-1)
				{
					sz.width = cvRound(full_img_sizes[i].width * compose_scale);
					sz.height = cvRound(full_img_sizes[i].height * compose_scale);
				}

				Mat K;
				cameras[i].K().convertTo(K, CV_32F);
				Rect roi = warper->warpRoi(sz, K, cameras[i].R);
				corners[i] = roi.tl();
				sizes[i] = roi.size();
			}
		}
		if (abs(compose_scale - 1) > 1e-1)
			resize(full_img, img, Size(), compose_scale, compose_scale);
		else
			img = full_img;
		full_img.release();
		Size img_size = img.size();

		Mat K;
		cameras[img_idx].K().convertTo(K, CV_32F);

		// Warp the current image
		warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);

		// Warp the current image mask
		mask.create(img_size, CV_8U);
		mask.setTo(Scalar::all(255));
		warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

		// Compensate exposure
		compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

		img_warped.convertTo(img_warped_s, CV_16S);
		img_warped.release();
		img.release();
		mask.release();

		dilate(masks_warped[img_idx], dilated_mask, Mat());
		resize(dilated_mask, seam_mask, mask_warped.size());
		mask_warped = seam_mask & mask_warped;

		if (!blender && !timelapse)
		{
			blender = Blender::createDefault(blend_type, try_cuda);
			Size dst_sz = resultRoi(corners, sizes).size();
			float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
			if (blend_width < 1.f)
				blender = Blender::createDefault(Blender::NO, try_cuda);
			else if (blend_type == Blender::MULTI_BAND)
			{
				MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
				mb->setNumBands(static_cast<int>(ceil(log(blend_width) / log(2.)) - 1.));
				LOGLN("Multi-band blender, number of bands: " << mb->numBands());
			}
			else if (blend_type == Blender::FEATHER)
			{
				FeatherBlender* fb = dynamic_cast<FeatherBlender*>(blender.get());
				fb->setSharpness(1.f / blend_width);
				LOGLN("Feather blender, sharpness: " << fb->sharpness());
			}
			blender->prepare(corners, sizes);
		}
		else if (!timelapser && timelapse)
		{
			timelapser = Timelapser::createDefault(timelapse_type);
			timelapser->initialize(corners, sizes);
		}

		// Blend the current image
		if (timelapse)
		{
			timelapser->process(img_warped_s, Mat::ones(img_warped_s.size(), CV_8UC1), corners[img_idx]);
			String fixedFileName;
			size_t pos_s = String(img_names[img_idx]).find_last_of("/\\");
			if (pos_s == String::npos)
			{
				fixedFileName = "fixed_" + img_names[img_idx];
			}
			else
			{
				fixedFileName = "fixed_" + String(img_names[img_idx]).substr(pos_s + 1, String(img_names[img_idx]).length() - pos_s);
			}
			imwrite(fixedFileName, timelapser->getDst());
		}
		else
		{
			blender->feed(img_warped_s, mask_warped, corners[img_idx]);
		}
	}

	if (!timelapse)
	{
		Mat result, result_mask;
		blender->blend(result, result_mask);

		imwrite(result_name, result);
	}

	return 0;
}

void apx_generation()
{
	SetCurrentDirectoryA("C://uav_image2/");
	WIN32_FIND_DATAA find_data;
	HANDLE find_file;
	char file_name2[500][500];
	int y;
	y = 0;
	//처리할 영상에 대한 영상 위치자세 데이터의 파일명 읽어오기

	// Count the number of world files
	find_file = ::FindFirstFileA("*.pgw", &find_data);
	if (find_file != INVALID_HANDLE_VALUE) {
		do {
			if (find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY);
			else
			{
				char *c = find_data.cFileName;
				printf("%s \n", c);

				strcpy_s(file_name2[y], c);
				printf("");
				y++;
			};
		} while (::FindNextFileA(find_file, &find_data));

		::FindClose(find_file);
	}

	// Read the world files
	vector<worldData> wfile;
	worldData w;
	
	for (int i=0;i<y;i++)
	{
		char* Worldname = file_name2[i];
		ifstream inFile(Worldname);		

		if (inFile) {
			//wfile.clear();
			
			while (inFile >> w.world[0] >> w.world[1] >> w.world[2] >> w.world[3] >> w.world[4] >> w.world[5])
				wfile.push_back(w);
			//in.clear();
		}
	}

	
	for (int i = 0; i < y; i++) {
		cout << wfile[i].world[4] << endl;
	}
	
	double worldX = GetMinValue_X(wfile);	// Get the min vaules of X
	double worldY = GetMaxValue_Y(wfile);	// Get the max vaules of Y
	
	// Output the world file
	ofstream outFile("result.jgw");
	outFile << wfile[0].world[0] << endl << 0 << endl << 0 << endl << wfile[0].world[3] << endl << worldX << endl << worldY;
	outFile.close();		
}

void main() {

	//SetCurrentDirectoryA("C://uav_image/");

	//char file_name[500][500];
	//int x;
	//WIN32_FIND_DATAA find_data;
	//HANDLE find_file;

	//char sttime_hms[30], edtime_hms[30];
	//std::time_t tNow = std::time(NULL);
	//std::tm tmHMS;
	//localtime_s(&tmHMS, &tNow);
	//tmHMS.tm_hour = 0;
	//tmHMS.tm_min = 0;
	//tmHMS.tm_sec = 0;

	//std::time_t theDay000 = std::mktime(&tmHMS);

	//x = 0;
	//	//처리할 영상에 대한 영상 위치자세 데이터의 파일명 읽어오기
	//	find_file = ::FindFirstFileA("*.txt", &find_data);
	//	if (find_file != INVALID_HANDLE_VALUE) {
	//		do {

	//			if (find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY);
	//			else
	//			{
	//				char *c = find_data.cFileName;
	//				printf("%s \n", c);

	//				strcpy_s(file_name[x], c);
	//				printf("");
	//				x++;
	//			};
	//		} while (::FindNextFileA(find_file, &find_data));

	//		::FindClose(find_file);
	//	}

	//	if (x == 0)
	//	{
	//		printf("파일이 없습니다.");
	//		exit(0);
	//	}
	//	
	//	for (int i = 0; i < x; i++)
	//	{
	//		std::chrono::system_clock::time_point tpDay000 = std::chrono::system_clock::from_time_t(theDay000);
	//		std::chrono::system_clock::time_point tpStart = std::chrono::system_clock::now();
	//		int stms = std::chrono::duration_cast<std::chrono::milliseconds>(tpStart - tpDay000).count();
	//		int sthour = std::chrono::duration_cast<std::chrono::hours>(tpStart - tpDay000).count();
	//		int stminute = std::chrono::duration_cast<std::chrono::minutes>(tpStart - tpDay000).count();
	//		int stsecond = std::chrono::duration_cast<std::chrono::seconds>(tpStart - tpDay000).count();
	//		sprintf_s(sttime_hms, "%d:%d:%.3lf", sthour, int(stminute - sthour * 60), double(stsecond - sthour * 3600 - (stminute - sthour * 60) * 60 + (stms*0.001 - stsecond)));
	//		printf("%s\n", sttime_hms);				

	//		char* EOname = file_name[i];

	//		if ( ortho(EOname) )//txt 파일을 넘겨 받아 영상 처리
	//		{					
	//			//처리된 txt 파일 저장 경로 변경
	//			char Dpath[500] = "C://uav_image/Done/";
	//			strcat_s(Dpath, file_name[i]);
	//			rename(file_name[i], Dpath);
	//		}

	//		std::chrono::system_clock::time_point tpEnd = std::chrono::system_clock::now();
	//		int edms = std::chrono::duration_cast<std::chrono::milliseconds>(tpEnd - tpDay000).count();
	//		int edhour = std::chrono::duration_cast<std::chrono::hours>(tpEnd - tpDay000).count();
	//		int edminute = std::chrono::duration_cast<std::chrono::minutes>(tpEnd - tpDay000).count();
	//		int edsecond = std::chrono::duration_cast<std::chrono::seconds>(tpEnd - tpDay000).count();
	//		int processtime = std::chrono::duration_cast<std::chrono::milliseconds>(tpEnd - tpStart).count();
	//		sprintf_s(edtime_hms, "%d:%d:%.3lf", edhour, int(edminute - edhour * 60), double(edsecond - edhour * 3600 - (edminute - edhour * 60) * 60 + (edms*0.001 - edsecond)));
	//		printf("%s\n", edtime_hms);
	//		LogFile(EOname, stms, sttime_hms, edms, edtime_hms, processtime);
	//	}


	stitching();
	apx_generation();
}

double GetMinValue_X(vector<worldData> wFile) {
	double min = wFile[0].world[4];		// X

	for (int i = 0; i < wFile.size(); i++) {
		if (wFile[i].world[4] < min)
			min = wFile[i].world[4];
	}

	return min;
}

double GetMaxValue_Y(vector<worldData> wFile) {
	double max = wFile[0].world[5];		// Y

	for (int i = 0; i < wFile.size(); i++) {
		if (wFile[i].world[4] < max)
			max = wFile[i].world[5];
	}

	return max;
}