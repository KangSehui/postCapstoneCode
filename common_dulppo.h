#pragma once

#include "global.h"

double depth_level_2_Z(unsigned char d);
double depth_level_2_Z_s(unsigned short d);
double depth_level_2_Z_s(unsigned short d, int camera);
unsigned char Z_2_depth_level(double z);
unsigned short Z_2_depth_level_s(double z);

void file_print_init(int ppc_mode, int shape_of_voxel, int voxel_res);
void file_print_sim(int cnt, double occlustion_pattern_value, double cor_coeff);
void file_print_data(int frame, int registered_size, int plen_size, vector<vector<float>> dev_pointnum,
	vector<float> point_num_per_color, vector<int> num_holes, vector<float> psnrs_y_p, vector<float> psnrs_y_h, vector<vector<float>> &dev_pointnum_avr);
void file_print_dev(vector<vector<float>> dev_pointnum_avr, int plen_size);
void file_print_end();

double dequantization_short(unsigned short norm, double Min, double Max);
unsigned short quantization_short(double denorm, double Min, double Max);
void projection_UVZ_2_XY_PC(Matrix4d projMatrix, double u, double v, double z, double *x, double *y);
double MVG(Matrix3d K, Matrix3d R, Matrix3Xd t, int x, int y, double Z, double *X, double *Y);
bool confirm_point(
	int camera,
	PointXYZRGB p,
	vector<Mat> color_imgs);
bool confirm_point_modified(
	int camera,
	PointXYZRGB p,
	Mat color_img, Mat depth_img, int th, int d_th);

//Mat cvt_yuv2bgr(string name, int frame, int type);
Mat cvt_yuv2bgr(
	string name,
	int frame,
	int type,
	bool is_yuv=true);

PointCloud<PointXYZRGB>::Ptr make_PC(
	int camera,
	Mat color_img,
	Mat depth_img,
	vector<Mat> color_imgs
	);

vector<PointCloud<PointXYZRGB>::Ptr> get_PC_of_every_camera(
	int frame,
	vector<vector<string>> color_names,
	vector<vector<string>> depth_names,
	vector<Mat>& color_imgs,
	vector<Mat>& depth_imgs
	);

vector<PointCloud<PointXYZRGB>::Ptr> get_PC_of_every_camera(
	int frame,
	vector<string> color_names_,
	vector<string> depth_names_,
	vector<Mat> &color_imgs,
	vector<Mat> &depth_imgs
	);

PointCloud<PointXYZRGB>::Ptr make_registered_PC(vector<PointCloud<PointXYZRGB>::Ptr> pointclouds);
void find_min_max(PointCloud<PointXYZRGB>::Ptr source_PC, vector<float> &min, vector<float> &max);
void find_min_max(vector<PPC> source_PC, vector<float> &min, vector<float> &max);
void find_min_max(PointCloud<PointXYZRGB>::Ptr source_PC, vector<double>& min, vector<double>& max);
void find_min_max(vector<PPC> source_PC, vector<double>& min, vector<double>& max);
void test_view_PC(
	PointCloud<PointXYZRGB>::Ptr pointcloud1,
	PointCloud<PointXYZRGB>::Ptr pointcloud2,
	PointCloud<PointXYZRGB>::Ptr pointcloud3);
void view_PC(PointCloud<PointXYZRGB>::Ptr pointcloud);
void view_PC(PointCloud<PointXYZRGB>::Ptr pointcloud1, PointCloud<PointXYZRGB>::Ptr pointcloud2);
void projection(PointCloud<PointXYZRGB>::Ptr pointcloud, vector<int> num_p, int camera, int camera_order, Mat& img, Mat& depthimg);
double det(double mat[3][3]);
void back_projection(PointCloud<PointXYZRGB>::Ptr pointcloud, int camera, Mat &img, int nNeighbor);
double projection_XYZ_2_UV(Matrix4d projMatrix, double x, double y, double z, int& u, int& v);
double find_point_dist(double w, int camera);
void cvProjectPoints2_new(const CvMat* objectPoints,
	const CvMat* r_vec,
	const CvMat* t_vec,
	const CvMat* A,
	const CvMat* distCoeffs,
	CvMat* imagePoints,
	double aspectRatio,
	int camera,
	vector<Point3f> RGB,
	Mat &depthimg,
	Mat &projectimg);

//PointCloud<PointXYZRGB>::Ptr filtering(PointCloud<PointXYZRGB>::Ptr original_PC);
void RGB_dev(vector<PPC> PPC, vector<vector<float>> &dev_pointnum_percent, vector<float> &point_num_per_color);
void HSV_dev(vector<PPC> PPC, vector<vector<float>>& dev_pointnum_percent, vector<float>& point_num_per_color);
void YUV_dev(vector<PPC> PPC, vector<vector<float>>& dev_pointnum_percent, vector<float>& point_num_per_color);

void printPSNRWithoutBlackPixel(vector<Mat> orig_imgs, vector<Mat> proj_imgs, vector<float>& psnrs_y, vector<float>& psnrs_u, vector<float>& psnrs_v, vector<int>& num_holes);
void printPSNRWithBlackPixel(vector<Mat> orig_imgs, vector<Mat> proj_imgs, vector<float>& psnrs_y, vector<float>& psnrs_u, vector<float>& psnrs_v);

double pearsoncoeff(vector<double> X, vector<double> Y);
vector<double> operator*(vector<double> a, vector<double> b);
vector<double> operator-(vector<double> a, double b);
double stdev(vector<double> nums);
double sqsum(vector<double> a);
double mean(vector<double> a);
double sum(vector<double> a);
double correlationCoefficient(vector<double> X, vector<double> Y, int n);
double cal_color_sim(vector<double> X, vector<double> Y);
int combination(int n, int r);
Matrix4d compute_projection_matrices(int cam_num);

PointCloud<PointXYZRGB>::Ptr make_incremental_PC(vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	vector<char> camera_order,
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	vector<int> &num_p, int th, int d_th);

PointCloud<PointXYZRGB>::Ptr quantization(PointCloud<PointXYZRGB>::Ptr pointcloud);
void make_proj_img_vec(
	PointCloud<PointXYZRGB>::Ptr pointcloud,
	vector<Mat>& proj_img_vec,
	vector<Mat>& depth_value_img_vec,
	vector<int> num_p);

Mat find_hole(
	Mat depthimg,
	int window_size);

Mat make_filled_image(
	Mat depthimg,
	Mat colorimg,
	Mat hole_image,
	int window_size);

void hole_filling(
	vector<Mat> depthimgs,
	vector<Mat> colorimgs,
	vector<Mat>& filled_imgs);

void printPSNR(
	vector<Mat> orig_imgs,
	vector<Mat> proj_imgs);