#include "global.h"
#define _CRT_SECURE_NO_WARNINGS

ofstream fout_cs;
ofstream fout_os;
ofstream fout_data;
ofstream fout_dev;

void file_print_init(int ppc_mode, int shape_of_voxel, int voxel_res) {

	int cnt = 0;

	string name_mode;
	if (mode == 0) name_mode = "ballet";
	else if (mode == 1) name_mode = "fencing";
	else if (mode == 2) name_mode = "intel";
	else if (mode == 3) name_mode = "tech";

	string shape, name_ppc;
	if (shape_of_voxel == 1) shape = "frustum";
	else shape = "cube";

	if (ppc_mode == 2) name_ppc = "incre";
	else name_ppc = "batch";

	string name_cs = "output\\" + name_mode + "_" + to_string(voxel_res) + "_" + shape + "_" + name_ppc + "_cs.csv";
	string name_os = "output\\" + name_mode + "_" + to_string(voxel_res) + "_" + shape + "_" + name_ppc + "_os.csv";
	string name_data = "output\\" + name_mode + "_" + to_string(voxel_res) + "_" + shape + "_" + name_ppc + "_data.csv";
	string name_dev = "output\\" + name_mode + "_" + to_string(voxel_res) + "_" + shape + "_" + name_ppc + "_dev.csv";


	fout_cs.open(name_cs, ios::out);
	fout_os.open(name_os, ios::out);
	fout_data.open(name_data, ios::out);
	fout_dev.open(name_dev, ios::out);
	
	fout_data << "frame,PC,PPC,hole num";
	for (int i = 0; i < total_num_cameras; i++)
		fout_data << ",cam" << i;
	
	fout_data << ",PSNR p";
	for (int i = 0; i < total_num_cameras; i++)
		fout_data << ",cam" << i;
	
	fout_data << ",PSNR h";
	for (int i = 0; i < total_num_cameras; i++)
		fout_data << ",cam" << i;
	
	fout_data << "\n";
	
	
	fout_dev << "frame,";
	for (int i = 0; i < total_num_cameras; i++)
		fout_dev << i + 1 << "개,0,1,2,3,4,5,6,7,";

	fout_dev << "\n";
}

void file_print_sim(int cnt, double occlustion_pattern_value, double cor_coeff) {
	fout_os << occlustion_pattern_value << ",";
	if (cnt % 1000 == 999)
		fout_os << endl;

	fout_cs << cor_coeff << ",";
	if (cnt % 1000 == 999)
		fout_cs << endl;
}

void file_print_data(int frame, int registered_size, int plen_size, vector<vector<float>> dev_pointnum,
	vector<float> point_num_per_color, vector<int> num_holes, vector<float> psnrs_y_p, vector<float> psnrs_y_h, vector<vector<float>> &dev_pointnum_avr) {

	fout_data << frame << ",";
	fout_dev << frame << ",";

	fout_data << registered_size << ",";

	fout_data << plen_size << ",";
	
	for (int i = 0; i < total_num_cameras; i++) {
		for (int j = 0; j < 8; j++) {
			dev_pointnum_avr[i][j] += dev_pointnum[i][j];
		}
		fout_dev << "," << dev_pointnum[i][0]
			<< "," << dev_pointnum[i][1]
			<< "," << dev_pointnum[i][2]
			<< "," << dev_pointnum[i][3]
			<< "," << dev_pointnum[i][4]
			<< "," << dev_pointnum[i][5]
			<< "," << dev_pointnum[i][6]
			<< "," << dev_pointnum[i][7] << ",";
	}
	fout_dev << "\n";

	for (int i = 0; i < total_num_cameras; i++)
		fout_data << "," << num_holes[i];

	fout_data << ",";
	
	for (int i = 0; i < total_num_cameras; i++)
		fout_data << "," << psnrs_y_p[i];
	
	fout_data << ",";
	
	for (int i = 0; i < total_num_cameras; i++)
		fout_data << "," << psnrs_y_h[i];
	
	fout_data << "\n";
}

void file_print_dev(vector<vector<float>> dev_pointnum_avr, int plen_size) {
	for (int i = 0; i < total_num_cameras; i++) {
		float num = 0;
		for (int j = 0; j < 8; j++) {

			dev_pointnum_avr[i][j] /= 10;
			num += dev_pointnum_avr[i][j];
		}

		fout_dev << int(num) <<"("<< round(num/ plen_size*100 * 10)/10.0 << "%),";

		for (int j = 0; j < 8; j++) {
			dev_pointnum_avr[i][j] = round((dev_pointnum_avr[i][j] / int(num)) * 100 * 100) / 100.0;
			fout_dev << dev_pointnum_avr[i][j]<< "/";
		}
		fout_dev << "\n";
	}
}

void file_print_end() {

	fout_cs.close();
	fout_os.close();
	fout_data.close();
	fout_dev.close();
}

double dequantization_short(unsigned short norm, double min, double max)
{
	double denorm;

	denorm = ((double)norm / 65535.0) * (max - min) + min;

	return denorm;
}

double depth_level_2_Z(unsigned char d)
{
	double z;

	z = 1.0 / ((d / 255.0) * (1.0 / MinZ - 1.0 / MaxZ) + 1.0 / MaxZ);

	return z;
}

double depth_level_2_Z_s(unsigned short d)
{
	double z;

	z = 1.0 / ((d / 65535.0) * (1.0 / MinZ - 1.0 / MaxZ) + 1.0 / MaxZ);

	return z;
}

unsigned char Z_2_depth_level(double z)
{
	unsigned char d;

	d = (1.0 / z - 1.0 / MaxZ) / (1.0 / MinZ - 1.0 / MaxZ) * 255.0;

	return d;
}

unsigned short Z_2_depth_level_s(double z)
{
	unsigned short d;

	d = (1.0 / z - 1.0 / MaxZ) / (1.0 / MinZ - 1.0 / MaxZ) * 65535.0;

	return d;
}

double depth_level_2_Z_s(unsigned short d, int camera)
{
	double z;

	z = 1.0 / ((d / 65535.0) * (1.0 / tech_minmaxZ[camera][0] - 1.0 / tech_minmaxZ[camera][1]) + 1.0 / tech_minmaxZ[camera][1]);

	return z;
}

unsigned short quantization_short(double denorm, double min, double max)
{
	unsigned short norm;

	norm = ((denorm - min) * 65535.0) / (max - min);
	
	return norm;
}

void projection_UVZ_2_XY_PC(
	Matrix4d projMatrix,
	double u,
	double v,
	double z,
	double *x,
	double *y)
{
	double c0, c1, c2;

	c0 = z * projMatrix(0, 2) + projMatrix(0, 3);
	c1 = z * projMatrix(1, 2) + projMatrix(1, 3);
	c2 = z * projMatrix(2, 2) + projMatrix(2, 3);

	v = (double)_height - v - 1.0;

	*y = u * (c1 * projMatrix(2, 0) - projMatrix(1, 0) * c2)
		+ v * (c2 * projMatrix(0, 0) - projMatrix(2, 0) * c0)
		+ projMatrix(1, 0) * c0
		- c1 * projMatrix(0, 0);

	*y /= v * (projMatrix(2, 0) * projMatrix(0, 1) - projMatrix(2, 1) * projMatrix(0, 0))
		+ u * (projMatrix(1, 0) * projMatrix(2, 1) - projMatrix(1, 1) * projMatrix(2, 0))
		+ projMatrix(0, 0) * projMatrix(1, 1)
		- projMatrix(1, 0) * projMatrix(0, 1);

	*x = (*y)*(projMatrix(0, 1) - projMatrix(2, 1) * u) + c0 - c2 * u;

	*x /= projMatrix(2, 0) * u - projMatrix(0, 0);
}


double projection_XYZ_2_UV(
	Matrix4d projMatrix,
	double x,
	double y,
	double z,
	int& u,
	int& v)
{
	double u_, v_;
	double w;
	u_ = projMatrix(0, 0) * x + projMatrix(0, 1) * y + projMatrix(0, 2) * z + projMatrix(0, 3);
	v_ = projMatrix(1, 0) * x + projMatrix(1, 1) * y + projMatrix(1, 2) * z + projMatrix(1, 3);
	w = projMatrix(2, 0) * x + projMatrix(2, 1) * y + projMatrix(2, 2) * z + projMatrix(2, 3);


	u_ /= w;
	v_ /= w;
	if (mode == MSR3DVideo_Ballet) {
		v_ = _height - v_ - 1.0;
	}

	u = cvRound(u_);
	v = cvRound(v_);

	return w;
}

double MVG(
	Matrix3d K,
	Matrix3d R_wc,
	Matrix3Xd t_wc,
	int x,
	int y,
	double Z,
	double *X,
	double *Y)
{
	double X_cam = (x - K(0, 2)) * (Z / K(0, 0));
	double Y_cam = (y - K(1, 2)) * (Z / K(1, 1));

	//cam coord
	Matrix3Xd C_cam(3, 1);
	C_cam(0, 0) = X_cam;
	C_cam(1, 0) = Y_cam;
	C_cam(2, 0) = Z;

	//assuming R, t as matrix world to cam
	Matrix3Xd C_world(3, 1);
	C_world = R_wc.inverse() * (C_cam - t_wc);
	*X = C_world(0, 0);
	*Y = C_world(1, 0);

	return C_world(2, 0);
}

bool confirm_point_modified(
	int camera,
	PointXYZRGB p,
	Mat color_img, Mat depth_img, int th, int d_th)
{
	int u, v;

	int Y_sub, U_sub, V_sub, D_sub;

	projection_XYZ_2_UV(m_CalibParams[camera].m_ProjMatrix, p.x, p.y, p.z, u, v);

	Mat p_color(1, 1, CV_8UC3), color(1, 1, CV_8UC3);
	Vec3b depth, p_depth;
	Vec3s depth_s, p_depth_s;

	p_color.at<Vec3b>(0, 0)[0] = p.b;
	p_color.at<Vec3b>(0, 0)[1] = p.g;
	p_color.at<Vec3b>(0, 0)[2] = p.r;

	color.at<Vec3b>(0, 0)[0] = color_img.at<Vec3b>(v, u)[0];
	color.at<Vec3b>(0, 0)[1] = color_img.at<Vec3b>(v, u)[1];
	color.at<Vec3b>(0, 0)[2] = color_img.at<Vec3b>(v, u)[2];

	cvtColor(p_color, p_color, CV_BGR2YUV);
	cvtColor(color, color, CV_BGR2YUV);

	Y_sub = abs(int(color.at<Vec3b>(0, 0)[0] - p_color.at<Vec3b>(0, 0)[0]));
	U_sub = abs(int(color.at<Vec3b>(0, 0)[1] - p_color.at<Vec3b>(0, 0)[1]));
	V_sub = abs(int(color.at<Vec3b>(0, 0)[2] - p_color.at<Vec3b>(0, 0)[2]));

	if (mode == 0) {
		depth = depth_img.at<Vec3b>(v, u);
		p_depth = Z_2_depth_level(p.z);
		D_sub = abs(depth[0] - p_depth[0]);
	}
	else {
		depth_s = depth_img.at<Vec3s>(v, u);
		p_depth_s = Z_2_depth_level_s(p.z);
		D_sub = abs(depth_s[0] - p_depth_s[0]);
	}
	
	//cout << depth_s << " " << p_depth_s << endl;

	if (Y_sub < th && U_sub < th && V_sub < th && D_sub < d_th) return true; //true일때 차이값이 작음
	return false;//false일때 차이값이 큼 
}

bool confirm_point(
	int camera,
	PointXYZRGB p,
	vector<Mat> color_imgs)
{
	int u, v;

	int Y_sub, U_sub, V_sub;

	vector<char> camera_order;
	if (mode == 0) camera_order = { 4,3,5,2,6,1,7,0 };
	else camera_order = { 5,4,6,3,7,2,8,1,9,0 };

	for (int i = 0; i < total_num_cameras; i++)
	{
		int cam_num = (int)camera_order[i];

		if (camera == cam_num) continue;

		projection_XYZ_2_UV(m_CalibParams[cam_num].m_ProjMatrix, p.x, p.y, -p.z, u, v);

		Mat p_one(1, 1, CV_8UC3), color_img_one(1, 1, CV_8UC3);
		p_one.at<Vec3b>(0, 0)[0] = p.b;
		p_one.at<Vec3b>(0, 0)[1] = p.g;
		p_one.at<Vec3b>(0, 0)[2] = p.r;

		if (u < 0 || v < 0 || u >= _width || v >= _height) return true;

		color_img_one.at<Vec3b>(0, 0)[0] = color_imgs[cam_num].at<Vec3b>(v, u)[0];
		color_img_one.at<Vec3b>(0, 0)[1] = color_imgs[cam_num].at<Vec3b>(v, u)[1];
		color_img_one.at<Vec3b>(0, 0)[2] = color_imgs[cam_num].at<Vec3b>(v, u)[2];

		cvtColor(p_one, p_one, CV_BGR2YUV);
		cvtColor(color_img_one, color_img_one, CV_BGR2YUV);

		Y_sub = abs(int(color_img_one.at<Vec3b>(0, 0)[0] - p_one.at<Vec3b>(0, 0)[0]));
		U_sub = abs(int(color_img_one.at<Vec3b>(0, 0)[1] - p_one.at<Vec3b>(0, 0)[1]));
		V_sub = abs(int(color_img_one.at<Vec3b>(0, 0)[2] - p_one.at<Vec3b>(0, 0)[2]));

		//cout << U << " " << V << endl;

		//20
		if (U_sub < 0 && V_sub <0) return true; //true일때 차이값이 작음
	}
	return false;//false일때 차이값이 큼 
}

Mat cvt_yuv2bgr(
	string name,
	int frame,
	int type,
	bool is_yuv)
{
	int numOfPixel = _width * _height;

	//  file I/O
	FILE* fp = fopen(name.c_str(), "rb");
	if (fp == NULL) {
		perror("encoder file open fail");
		exit(0);
	}

	if (!is_yuv) {
		short* buffer = new short[numOfPixel];
		Mat img(_height, _width, CV_16UC3);

		for (int i = 0; i < frame; i++)
			fread(buffer, 2, numOfPixel, fp);

		// byte data 읽어오기
		fread(buffer, 2, numOfPixel, fp);

		// y 채널 Mat로 만들기
		for (int y = 0; y < _height; y++)
			for (int x = 0; x < _width; x++) {
				img.at<Vec3s>(y, x)[0] = buffer[y * _width + x];
				img.at<Vec3s>(y, x)[1] = buffer[y * _width + x];
				img.at<Vec3s>(y, x)[2] = buffer[y * _width + x];
			}

		delete[] buffer;

		return img;
	}

	if (type == 8) {
		char* buffer_y = new char[numOfPixel];
		char* buffer_u = new char[numOfPixel / 4];
		char* buffer_v = new char[numOfPixel / 4];

		Mat img_y(_height, _width, CV_8UC1);
		Mat img_u(_height / 2, _width / 2, CV_8UC1);
		Mat img_v(_height / 2, _width / 2, CV_8UC1);
		Mat img_yuv[3];
		Mat img;

		for (int i = 0; i < frame; i++) {
			fread(buffer_y, 1, numOfPixel, fp);
			fread(buffer_u, 1, numOfPixel / 4, fp);
			fread(buffer_v, 1, numOfPixel / 4, fp);
		}

		// byte data 읽어오기
		fread(buffer_y, 1, numOfPixel, fp);
		fread(buffer_u, 1, numOfPixel / 4, fp);
		fread(buffer_v, 1, numOfPixel / 4, fp);

		// y 채널 Mat로 만들기
		for (int y = 0; y < _height; y++) {
			for (int x = 0; x < _width; x++) {
				img_y.at<uchar>(y, x) = buffer_y[y * _width + x];
			}
		}

		// u, v 채널 Mat로 만들기
		for (int y = 0; y < _height / 2; y++) {
			for (int x = 0; x < _width / 2; x++) {
				img_u.at<uchar>(y, x) = buffer_u[y * (_width / 2) + x];
				img_v.at<uchar>(y, x) = buffer_v[y * (_width / 2) + x];
			}
		}

		// 3개 짜리 Mat array에 각 채널 Mat 할당.
		// u, v 채널은 resize 수행.
		img_yuv[0] = img_y;
		resize(img_u, img_yuv[1], Size(_width, _height));
		resize(img_v, img_yuv[2], Size(_width, _height));

		// 하나의 다중 채널 Mat로 만들기.
		merge(img_yuv, 3, img);

		// 색공간 변환.
		cvtColor(img, img, CV_YUV2BGR);

		delete[] buffer_y;
		delete[] buffer_u;
		delete[] buffer_v;

		// 여기서 10bits 변환하면 될듯

		return img;
	}
	else if (type == 10) {
		short* buffer_y = new short[numOfPixel];
		short* buffer_u = new short[numOfPixel / 4];
		short* buffer_v = new short[numOfPixel / 4];

		Mat img_y(_height, _width, CV_8U);
		Mat img_u(_height / 2, _width / 2, CV_8U);
		Mat img_v(_height / 2, _width / 2, CV_8U);
		Mat img_yuv[3];
		Mat img;

		for (int i = 0; i < frame; i++) {
			fread(buffer_y, 2, numOfPixel, fp);
			fread(buffer_u, 2, numOfPixel / 4, fp);
			fread(buffer_v, 2, numOfPixel / 4, fp);
		}

		// byte data 읽어오기
		fread(buffer_y, 2, numOfPixel, fp);
		fread(buffer_u, 2, numOfPixel / 4, fp);
		fread(buffer_v, 2, numOfPixel / 4, fp);

		// y 채널 Mat로 만들기
		for (int y = 0; y < _height; y++) {
			for (int x = 0; x < _width; x++) {
				img_y.at<uchar>(y, x) = uchar(buffer_y[y * _width + x] * 255.0 / 1023.0);
			}
		}

		// u, v 채널 Mat로 만들기
		for (int y = 0; y < _height / 2; y++) {
			for (int x = 0; x < _width / 2; x++) {
				img_u.at<uchar>(y, x) = uchar(buffer_u[y * (_width / 2) + x] * 255.0 / 1023.0);
				img_v.at<uchar>(y, x) = uchar(buffer_v[y * (_width / 2) + x] * 255.0 / 1023.0);
			}
		}

		// 3개 짜리 Mat array에 각 채널 Mat 할당.
		// u, v 채널은 resize 수행.
		img_yuv[0] = img_y;
		resize(img_u, img_yuv[1], Size(_width, _height));
		resize(img_v, img_yuv[2], Size(_width, _height));

		// 하나의 다중 채널 Mat로 만들기.
		merge(img_yuv, 3, img);

		// 색공간 변환.
		cvtColor(img, img, CV_YUV2BGR);

		delete[] buffer_y;
		delete[] buffer_u;
		delete[] buffer_v;

		return img;
	}
	else {
		short* buffer_y = new short[numOfPixel];
		short* buffer_u = new short[numOfPixel / 4];
		short* buffer_v = new short[numOfPixel / 4];

		Mat img_y(_height, _width, CV_16UC1);
		Mat img_u(_height / 2, _width / 2, CV_16UC1);
		Mat img_v(_height / 2, _width / 2, CV_16UC1);
		Mat img_yuv[3];
		Mat img;

		for (int i = 0; i < frame; i++) {
			fread(buffer_y, 2, numOfPixel, fp);
			fread(buffer_u, 2, numOfPixel / 4, fp);
			fread(buffer_v, 2, numOfPixel / 4, fp);
		}

		// byte data 읽어오기
		fread(buffer_y, 2, numOfPixel, fp);
		fread(buffer_u, 2, numOfPixel / 4, fp);
		fread(buffer_v, 2, numOfPixel / 4, fp);

		// y 채널 Mat로 만들기
		for (int y = 0; y < _height; y++) {
			for (int x = 0; x < _width; x++) {
				img_y.at<ushort>(y, x) = buffer_y[y * _width + x];
			}
		}

		// u, v 채널 Mat로 만들기
		for (int y = 0; y < _height / 2; y++) {
			for (int x = 0; x < _width / 2; x++) {
				img_u.at<ushort>(y, x) = buffer_u[y * (_width / 2) + x];
				img_v.at<ushort>(y, x) = buffer_v[y * (_width / 2) + x];
			}
		}

		// 3개 짜리 Mat array에 각 채널 Mat 할당.
		// u, v 채널은 resize 수행.
		img_yuv[0] = img_y;
		resize(img_u, img_yuv[1], Size(_width, _height));
		resize(img_v, img_yuv[2], Size(_width, _height));

		// 하나의 다중 채널 Mat로 만들기.
		merge(img_yuv, 3, img);

		// 색공간 변환.
		cvtColor(img, img, CV_YUV2BGR);

		delete[] buffer_y;
		delete[] buffer_u;
		delete[] buffer_v;

		return img;
	}
}

PointCloud<PointXYZRGB>::Ptr make_PC(
	int camera,
	Mat color_img,
	Mat depth_img,
	vector<Mat> color_imgs)
{
	PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);

	for (int y = 0; y < _height; y++) {
		for (int x = 0; x < _width; x++)
		{
			bool is_good_point = 0;

			Vec3b d, color;
			Vec3s d_s;
			double Z=0.0, X = 0.0, Y = 0.0;

			switch (mode) {
			case 0:
				d = depth_img.at<Vec3b>(y, x);
				color = color_img.at<Vec3b>(y, x);

				//Z = denormalization_c(d[0], MinZ, MaxZ);
				Z = depth_level_2_Z(d[0]);
				projection_UVZ_2_XY_PC(m_CalibParams[camera].m_ProjMatrix, x, y, Z, &X, &Y);

				//Z *= (-1);//change
				break;

			case 1:
			case 2:
			case 3:
				d_s = depth_img.at<Vec3s>(y, x);
				color = color_img.at<Vec3b>(y, x);

				
				//Z = denormalization_s(d_s[0], MinZ, MaxZ);
				Z = depth_level_2_Z_s(d_s[0]);
				//imshow("depth img",depth_img);
				//waitKey(0);

				Z = MVG(m_CalibParams[camera].m_K, m_CalibParams[camera].m_RotMatrix, m_CalibParams[camera].m_Trans, x, y, Z, &X, &Y);
				break;
			}
			PointXYZRGB p;

			p.x = X;
			p.y = Y;
			p.z = Z;

			p.b = (float)color[0];
			p.g = (float)color[1];
			p.r = (float)color[2];

			pointcloud->points.push_back(p);

			//is_good_point = confirm_point(camera, p, color_imgs);

			//if (is_good_point)
			//	pointcloud->points.push_back(p);
			//else continue;

		}
	}
	return pointcloud;
}

vector<PointCloud<PointXYZRGB>::Ptr> get_PC_of_every_camera(
	int frame,
	vector<vector<string>> color_names,
	vector<vector<string>> depth_names,
	vector<Mat> &color_imgs,
	vector<Mat> &depth_imgs)
{
	vector<Mat> imgs(total_num_cameras);
	vector<Mat> imgs2(total_num_cameras);

	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds(total_num_cameras);

	for (int camera = 0; camera < total_num_cameras; camera++)
	{
		string folder_path = path + "/cam" + to_string(camera) + "/";
		Mat color_img, depth_img;

		color_img = imread(folder_path + color_names[camera][frame]);
		depth_img = imread(folder_path + depth_names[camera][frame]);

		//cvtColor(color_img, color_img, CV_BGR2YUV);

		imgs[camera] = color_img;
		imgs2[camera] = depth_img;
	}

	for (int camera = 0; camera < total_num_cameras; camera++)
	{
		PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);
		pointcloud = make_PC(camera, imgs[camera], imgs2[camera], color_imgs);
		
		pointclouds[camera] = pointcloud;
	}

	color_imgs = imgs;
	depth_imgs = imgs2;

	return pointclouds;
}

vector<PointCloud<PointXYZRGB>::Ptr> get_PC_of_every_camera(
	int frame,
	vector<string> color_names_,
	vector<string> depth_names_,
	vector<Mat> &color_imgs,
	vector<Mat> &depth_imgs)
{
	vector<Mat> imgs(total_num_cameras);
	vector<Mat> imgs2(total_num_cameras);

	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds(total_num_cameras);

	for (int camera = 0; camera < total_num_cameras; camera++)
	{
		Mat color_img, depth_img;

		switch (mode) {
		case Poznan_Fencing:
			color_img = cvt_yuv2bgr(path + "\\" + color_names_[camera], frame, color_bits, 1);
			if (depth_bits == 8) depth_img = cvt_yuv2bgr(path + "\\" + depth_names_[camera], frame, depth_bits, 1);
			else depth_img = cvt_yuv2bgr(path + "\\" + depth_names_[camera], frame, depth_bits, 0);

			break;

		case Intel_Kermit:
		case Technicolor_Painter:
			color_img = cvt_yuv2bgr(path + "\\" + color_names_[camera], frame, color_bits, 1);
			depth_img = cvt_yuv2bgr(path + "\\" + depth_names_[camera], frame, depth_bits, 1);


			//imshow("color img", color_img);
			//imshow("depth img", depth_img);
			//waitKey(0);
			//imwrite(color_names_[camera]+".png", color_img);
			//imwrite(depth_names_[camera] + ".png", depth_img);
			break;
		}
		//cvtColor(color_img, color_img, CV_BGR2YUV);

		imgs[camera] = color_img;
		imgs2[camera] = depth_img;
	}
	for (int camera = 0; camera < total_num_cameras; camera++)
	{
		PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);
		pointcloud = make_PC(camera, imgs[camera], imgs2[camera], color_imgs);
		pointclouds[camera] = pointcloud;
	}

	color_imgs = imgs;
	depth_imgs = imgs2;

	return pointclouds;
}

PointCloud<PointXYZRGB>::Ptr make_registered_PC(
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds)
{

	PointCloud<PointXYZRGB>::Ptr registered_PC(new PointCloud<PointXYZRGB>);
	for (int camera = 0; camera < temp_num_cameras; camera++)
		for (int point_idx = 0; point_idx < pointclouds[camera]->points.size(); point_idx++)
			registered_PC->points.push_back(pointclouds[camera]->points[point_idx]);

	return registered_PC;
}

void find_min_max(
	PointCloud<PointXYZRGB>::Ptr source_PC,
	vector<float> &min,
	vector<float> &max)
{
	for (int p = 0; p < 3; p++)
	{
		min[p] = FLT_MAX;

		max[p] = -FLT_MAX;
	}

	for (int i = 0; i < source_PC->points.size(); i++)
	{
		if (source_PC->points[i].x < min[0]) min[0] = source_PC->points[i].x;
		if (source_PC->points[i].y < min[1]) min[1] = source_PC->points[i].y;
		if (source_PC->points[i].z < min[2]) min[2] = source_PC->points[i].z;

		if (source_PC->points[i].x > max[0]) max[0] = source_PC->points[i].x;
		if (source_PC->points[i].y > max[1]) max[1] = source_PC->points[i].y;
		if (source_PC->points[i].z > max[2]) max[2] = source_PC->points[i].z;
	}
}

void find_min_max(
	vector<PPC> source_PC,
	vector<float> &min,
	vector<float> &max)
{
	for (int p = 0; p < 3; p++)
	{
		min[p] = FLT_MAX;

		max[p] = -FLT_MAX;
	}

	for (int i = 0; i < source_PC.size(); i++)
	{
		if (source_PC[i].geometry[0] < min[0]) min[0] = source_PC[i].geometry[0];
		if (source_PC[i].geometry[1] < min[1]) min[1] = source_PC[i].geometry[1];
		if (source_PC[i].geometry[2] < min[2]) min[2] = source_PC[i].geometry[2];

		if (source_PC[i].geometry[0] > max[0]) max[0] = source_PC[i].geometry[0];
		if (source_PC[i].geometry[1] > max[1]) max[1] = source_PC[i].geometry[1];
		if (source_PC[i].geometry[2] > max[2]) max[2] = source_PC[i].geometry[2];
	}
}

void find_min_max(
	PointCloud<PointXYZRGB>::Ptr source_PC,
	vector<double>& min,
	vector<double>& max)
{
	for (int p = 0; p < 3; p++)
	{
		min[p] = FLT_MAX;

		max[p] = -FLT_MAX;
	}

	for (int i = 0; i < source_PC->points.size(); i++)
	{
		if (source_PC->points[i].x < min[0]) min[0] = source_PC->points[i].x;
		if (source_PC->points[i].y < min[1]) min[1] = source_PC->points[i].y;
		if (source_PC->points[i].z < min[2]) min[2] = source_PC->points[i].z;

		if (source_PC->points[i].x > max[0]) max[0] = source_PC->points[i].x;
		if (source_PC->points[i].y > max[1]) max[1] = source_PC->points[i].y;
		if (source_PC->points[i].z > max[2]) max[2] = source_PC->points[i].z;
	}
}

void find_min_max(
	vector<PPC> source_PC,
	vector<double>& min,
	vector<double>& max)
{
	for (int p = 0; p < 3; p++)
	{
		min[p] = FLT_MAX;

		max[p] = -FLT_MAX;
	}

	for (int i = 0; i < source_PC.size(); i++)
	{
		if (source_PC[i].geometry[0] < min[0]) min[0] = source_PC[i].geometry[0];
		if (source_PC[i].geometry[1] < min[1]) min[1] = source_PC[i].geometry[1];
		if (source_PC[i].geometry[2] < min[2]) min[2] = source_PC[i].geometry[2];

		if (source_PC[i].geometry[0] > max[0]) max[0] = source_PC[i].geometry[0];
		if (source_PC[i].geometry[1] > max[1]) max[1] = source_PC[i].geometry[1];
		if (source_PC[i].geometry[2] > max[2]) max[2] = source_PC[i].geometry[2];
	}
}

void test_view_PC(
	PointCloud<PointXYZRGB>::Ptr pointcloud1,
	PointCloud<PointXYZRGB>::Ptr pointcloud2,
	PointCloud<PointXYZRGB>::Ptr pointcloud3)
{
	int v1 = 0;

	PCLVisualizer viewer("PC viewer demo");
	viewer.setSize(1280, 1000);
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
	viewer.addCoordinateSystem(5.0);
	viewer.setBackgroundColor(255, 255, 255);

	PointCloudColorHandlerRGBField<pcl::PointXYZRGB > rgb_handler(pointcloud1);
	PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(pointcloud2, 255, 0, 0);
	PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue(pointcloud3, 0, 0, 255);

	viewer.addPointCloud(pointcloud2, red, "view_3", v1);
	viewer.addPointCloud(pointcloud3, blue, "view_5", v1);
	viewer.addPointCloud(pointcloud1, rgb_handler, "view_ref", v1);

	while (!viewer.wasStopped()) viewer.spinOnce();
}

void view_PC(PointCloud<PointXYZRGB>::Ptr pointcloud)
{
	int v1 = 0;

	PCLVisualizer viewer("PC viewer demo");
	viewer.setSize(1280, 1000);
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
	viewer.addCoordinateSystem(5.0);

	PointCloudColorHandlerRGBField<pcl::PointXYZRGB > rgb_handler(pointcloud);
	viewer.addPointCloud(pointcloud, rgb_handler, "result", v1);
	while (!viewer.wasStopped()) viewer.spinOnce();
}

void view_PC(PointCloud<PointXYZRGB>::Ptr pointcloud1, PointCloud<PointXYZRGB>::Ptr pointcloud2) {
	{
		int v1 = 0;

		PCLVisualizer viewer("PC viewer demo");
		viewer.setSize(1280, 1000);
		viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
		viewer.addCoordinateSystem(5.0);
		//Matrix4f extrinsic;
		//for (int i = 0; i < 3; i++)
		//   for (int j = 0; j < 3; j++)
		//      extrinsic(i, j) = (float)m_CalibParams[0].m_RotMatrix(i, j);

		//for (int i = 0; i < 3; i++)
		//   extrinsic(i, 3) = (float)m_CalibParams[0].m_Trans(i, 0);

		//extrinsic(3, 0) = 0;
		//extrinsic(3, 1) = 0;
		//extrinsic(3, 2) = 0;
		//extrinsic(3, 3) = 1;

		//cout << "extrinsic : "<<extrinsic << endl;

		//viewer.setCameraParameters(m_CalibParams[0].m_K, extrinsic);

		PointCloudColorHandlerRGBField<pcl::PointXYZRGB > rgb_handler1(pointcloud1);
		viewer.addPointCloud(pointcloud1, rgb_handler1, "result", v1);
		PointCloudColorHandlerRGBField<pcl::PointXYZRGB > rgb_handler2(pointcloud2);
		viewer.addPointCloud(pointcloud2, rgb_handler2, "result2", v1);

		while (!viewer.wasStopped()) viewer.spinOnce();
	}

}

double det(double mat[3][3])
{
	double D = 0;

	D = mat[0][0] * ((mat[1][1] * mat[2][2]) - (mat[2][1] * mat[1][2]))
		- mat[0][1] * (mat[1][0] * mat[2][2] - mat[2][0] * mat[1][2])
		+ mat[0][2] * (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]);

	return D;
}

double find_point_dist(
	double w,
	int camera)
{
	double numerator, denominator, dist = 0;
	double M[3][3];


	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			M[i][j] = m_CalibParams[camera].m_ProjMatrix(i, j);

	for (int i = 0; i < 3; i++)
		denominator = denominator + (M[2][i] * M[2][i]);

	denominator = sqrt(denominator);

	numerator = det(M);


	// sign
	if (numerator < 0) numerator = -1;

	else if (numerator == 0) numerator = 0;

	else numerator = 1;

	numerator = numerator * w;

	if (denominator == 0) cerr << "error" << endl;

	else dist = (numerator / denominator);

	return dist;
}

void projection(PointCloud<PointXYZRGB>::Ptr pointcloud, vector<int> num_p,int camera, int camera_order, Mat &img, Mat &depthimg)
{
	PointCloud<PointXYZRGB>::iterator cloudit;
	PointCloud<PointXYZRGB>::iterator cloudend;

	double X;
	double Y;
	double Z;
	int u;
	int v;

	double dist;
	double w;

	int cnt = 0;
	int cnt_exception = 0;

	Mat temp_img(_height, _width, CV_64F, -1);

	for (cloudit = pointcloud->points.begin(), cloudend = pointcloud->points.end(); cloudit < cloudend; cloudit++) {
		//if (camera_order != 0 && cnt < num_p[camera_order - 1]) {
		//	cnt++;
		//	continue;
		//}
		X = cloudit->x;
		Y = cloudit->y;
		Z = cloudit->z;

		//Z = -Z;

		w = projection_XYZ_2_UV(
			m_CalibParams[camera].m_ProjMatrix,
			X,
			Y,
			Z,
			u,
			v);

		if ((u < 0) || (v < 0) || (u > _width - 1) || (v > _height - 1)) {
			cnt_exception++;
			cnt++;
			continue;
		}

		dist = find_point_dist(w, camera);
		temp_img.at<double>(v, u) = dist;

		//if (depthimg.at<double>(v, u) == -1)
		//	depthimg.at<double>(v, u) = dist;
		//else
		//{
		//	if (dist < depthimg.at<double>(v, u))
		//		depthimg.at<double>(v, u) = dist; 

		//	else continue;
		//}

		img.at<Vec3b>(v, u)[0] = ushort(cloudit->b);
		img.at<Vec3b>(v, u)[1] = ushort(cloudit->g);
		img.at<Vec3b>(v, u)[2] = ushort(cloudit->r);

		cnt++;

		if (cnt == num_p[camera_order]) {
			break;
		}
	}
	//cout << camera_order << "  " << cnt << "  exception"<<cnt_exception<< endl;

	depthimg = temp_img;
}

void back_projection(
	PointCloud<PointXYZRGB>::Ptr pointcloud,
	int camera,
	Mat &img,
	int nNeighbor)
{
	if (nNeighbor != 4 && nNeighbor != 8 && nNeighbor != 12 && nNeighbor != 20 && nNeighbor != 24) {
		cerr << "Wrong neighbor number!" << endl;
		exit(0);
	}

	vector<vector<vector<PointXYZRGB>>> point_storage(img.rows, vector<vector<PointXYZRGB>>(img.cols));
	vector<vector<vector<double>>> point_dist_storage(img.rows, vector<vector<double>>(img.cols));

	Matrix<double, 3, 1> camera_center;
	Matrix<double, 4, 1> world_2d_;
	Matrix<double, 4, 4> RT;

	Mat geo_mat(Size(img.cols, img.rows), CV_64FC3);

	Vec3d temp1, temp2;

	double X;
	double Y;
	double Z;
	int u;
	int v;

	double dist;
	double w;

	PointCloud<PointXYZRGB>::iterator cloudit;
	PointCloud<PointXYZRGB>::iterator cloudend;

	for (cloudit = pointcloud->points.begin(), cloudend = pointcloud->points.end(); cloudit < cloudend; cloudit++)
	{
		PointXYZRGB temp_point;

		X = cloudit->x;
		Y = cloudit->y;
		Z = cloudit->z;

		//fencing ballet
		//if (mode == MSR3DVideo_Ballet) Z = -Z; //change

		w = projection_XYZ_2_UV(
			m_CalibParams[camera].m_ProjMatrix,
			X,
			Y,
			Z,
			u,
			v);

		dist = find_point_dist(w, camera);

		//cout << w << " " << dist << endl;

		if ((u < 0) || (v < 0) || (u > _width - 1) || (v > _height - 1)) continue;

		temp_point.x = cloudit->x;
		temp_point.y = cloudit->y;
		temp_point.z = cloudit->z;
		temp_point.r = cloudit->r;
		temp_point.g = cloudit->g;
		temp_point.b = cloudit->b;

		point_storage[v][u].push_back(temp_point);
		point_dist_storage[v][u].push_back(dist);
	}

	///////////////////////////////////////////////////////////////////// BACK PROJECTION
	camera_center = -(m_CalibParams[camera].m_RotMatrix.transpose())*m_CalibParams[camera].m_Trans;

	double cx = m_CalibParams[camera].m_K(2);
	double cy = m_CalibParams[camera].m_K(5);
	double focal_length = m_CalibParams[camera].m_K(0);

	RT << m_CalibParams[camera].m_RotMatrix(0, 0), m_CalibParams[camera].m_RotMatrix(0, 1), m_CalibParams[camera].m_RotMatrix(0, 2), m_CalibParams[camera].m_Trans(0),
		m_CalibParams[camera].m_RotMatrix(1, 0), m_CalibParams[camera].m_RotMatrix(1, 1), m_CalibParams[camera].m_RotMatrix(1, 2), m_CalibParams[camera].m_Trans(1),
		m_CalibParams[camera].m_RotMatrix(2, 0), m_CalibParams[camera].m_RotMatrix(2, 1), m_CalibParams[camera].m_RotMatrix(2, 2), m_CalibParams[camera].m_Trans(2),
		0, 0, 0, 1;

	for (int i = 0; i < img.cols; i++) {
		for (int j = 0; j < img.rows; j++) {
			world_2d_ << (j - cx), (i - cy), focal_length, 1;
			world_2d_ = RT.inverse() * world_2d_;

			geo_mat.at<Vec3d>(j, i)[0] = world_2d_(0);
			geo_mat.at<Vec3d>(j, i)[1] = world_2d_(1);
			geo_mat.at<Vec3d>(j, i)[2] = world_2d_(2);
		}
	}

	for (int i = 0; i < img.cols; i++) {
		for (int j = 0; j < img.rows; j++) {
			double minTanAngle = DBL_MAX;
			double minDepth = DBL_MAX;

			Vec3b temp_color;

			temp2[0] = geo_mat.at<Vec3d>(j, i)[0] - camera_center(0);
			temp2[1] = geo_mat.at<Vec3d>(j, i)[1] - camera_center(1);
			temp2[2] = geo_mat.at<Vec3d>(j, i)[2] - camera_center(2);

			if (point_storage[j][i].size() > 0) {
				vector<PointXYZRGB> point_vec = point_storage[j][i];
				for (int k = 0; k < point_vec.size(); k++) {
					PointXYZRGB pt = point_vec[k];
					double dist = point_dist_storage[j][i][k];

					if (minDepth > dist) {
						minDepth = dist;
						temp_color[0] = pt.b;
						temp_color[1] = pt.g;
						temp_color[2] = pt.r;
					}
				}
			}
			else {
				if (nNeighbor >= 4) {
					if (j > 0) {
						vector<PointXYZRGB> point_vec = point_storage[j - 1][i];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 1][i][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (i > 0) {
						vector<PointXYZRGB> point_vec = point_storage[j][i - 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j][i - 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j < img.rows - 1) {
						vector<PointXYZRGB> point_vec = point_storage[j + 1][i];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 1][i][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (i < img.cols - 1) {
						vector<PointXYZRGB> point_vec = point_storage[j][i + 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j][i + 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}
				}

				if (nNeighbor >= 8) {
					if (j > 0 && i > 0) {
						vector<PointXYZRGB> point_vec = point_storage[j - 1][i - 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 1][i - 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j > 0 && i < img.cols - 1) {
						vector<PointXYZRGB> point_vec = point_storage[j - 1][i + 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 1][i + 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j < img.rows - 1 && i > 0) {
						vector<PointXYZRGB> point_vec = point_storage[j + 1][i - 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 1][i - 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j < img.rows - 1 && i < img.cols - 1) {
						vector<PointXYZRGB> point_vec = point_storage[j + 1][i + 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 1][i + 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}
				}

				if (nNeighbor >= 12) {
					if (j > 1) {
						vector<PointXYZRGB> point_vec = point_storage[j - 2][i];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 2][i][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (i > 1) {
						vector<PointXYZRGB> point_vec = point_storage[j][i - 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j][i - 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j < img.rows - 2) {
						vector<PointXYZRGB> point_vec = point_storage[j + 2][i];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 2][i][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (i < img.cols - 2) {
						vector<PointXYZRGB> point_vec = point_storage[j][i + 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j][i + 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}
				}

				if (nNeighbor >= 20) {
					if (j > 1 && i > 0) {
						vector<PointXYZRGB> point_vec = point_storage[j - 2][i - 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 2][i - 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j > 1 && i < img.cols - 1) {
						vector<PointXYZRGB> point_vec = point_storage[j - 2][i + 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 2][i + 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (i < img.cols - 2 && j > 0) {
						vector<PointXYZRGB> point_vec = point_storage[j - 1][i + 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 1][i + 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (i < img.cols - 2 && j < img.rows - 1) {
						vector<PointXYZRGB> point_vec = point_storage[j + 1][i + 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 1][i + 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j < img.rows - 2 && i < img.cols - 1) {
						vector<PointXYZRGB> point_vec = point_storage[j + 2][i + 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 2][i + 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j < img.rows - 2 && i > 0) {
						vector<PointXYZRGB> point_vec = point_storage[j + 2][i - 1];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 2][i - 1][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (i > 1 && j < img.rows - 1) {
						vector<PointXYZRGB> point_vec = point_storage[j + 1][i - 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 1][i - 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (i > 1 && j > 0) {
						vector<PointXYZRGB> point_vec = point_storage[j - 1][i - 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 1][i - 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}
				}
				if (nNeighbor >= 24) {
					if (j > 1 && i > 1) {
						vector<PointXYZRGB> point_vec = point_storage[j - 2][i - 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 2][i - 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j > 1 && i < img.cols - 2) {
						vector<PointXYZRGB> point_vec = point_storage[j - 2][i + 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j - 2][i + 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j < img.rows - 2 && i < img.cols - 2) {
						vector<PointXYZRGB> point_vec = point_storage[j + 2][i + 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 2][i + 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}

					if (j < img.rows - 2 && i > 1) {
						vector<PointXYZRGB> point_vec = point_storage[j + 2][i - 2];
						for (int k = 0; k < point_vec.size(); k++) {
							PointXYZRGB pt = point_vec[k];
							double dist = point_dist_storage[j + 2][i - 2][k];

							temp1[0] = pt.x - camera_center(0);
							temp1[1] = pt.y - camera_center(1);
							temp1[2] = pt.z - camera_center(2);

							// double dist = norm(temp1.cross(temp2)) / norm(temp2);
							double tanAngle = abs(norm(temp1.cross(temp2)) / temp1.dot(temp2));

							if (minTanAngle > tanAngle) {
								minTanAngle = tanAngle;
								temp_color[0] = pt.b;
								temp_color[1] = pt.g;
								temp_color[2] = pt.r;
							}
						}
					}
				}
			}
			img.at<Vec3b>(j, i) = temp_color;
		}
	}
}

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
	Mat &projectimg)
{
	Ptr<CvMat> matM, _m;

	int i, count;
	const CvPoint3D64f* M;
	CvPoint2D64f* m;
	double r[3], R[9], dRdr[27], t[3], a[9], k[14] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 }, fx, fy, cx, cy;

	Matx33d matTilt = Matx33d::eye();
	Matx33d dMatTiltdTauX(0, 0, 0, 0, 0, 0, 0, -1, 0);
	Matx33d dMatTiltdTauY(0, 0, 0, 0, 0, 0, 1, 0, 0);
	CvMat _r, _t, _a = cvMat(3, 3, CV_64F, a), _k;

	CvMat matR = cvMat(3, 3, CV_64F, R), _dRdr = cvMat(3, 9, CV_64F, dRdr);

	bool fixedAspectRatio = aspectRatio > FLT_EPSILON;


	int total = objectPoints->rows * objectPoints->cols * CV_MAT_CN(objectPoints->type);
	if (total % 3 != 0)
	{
		//we have stopped support of homogeneous coordinates because it cause ambiguity in interpretation of the input data
		CV_Error(CV_StsBadArg, "Homogeneous coordinates are not supported");
	}
	count = total / 3;


	if (CV_IS_CONT_MAT(objectPoints->type) &&
		(CV_MAT_DEPTH(objectPoints->type) == CV_32F || CV_MAT_DEPTH(objectPoints->type) == CV_64F) &&
		((objectPoints->rows == 1 && CV_MAT_CN(objectPoints->type) == 3) ||
		(objectPoints->rows == count && CV_MAT_CN(objectPoints->type)*objectPoints->cols == 3) ||
			(objectPoints->rows == 3 && CV_MAT_CN(objectPoints->type) == 1 && objectPoints->cols == count)))
	{
		matM.reset(cvCreateMat(objectPoints->rows, objectPoints->cols, CV_MAKETYPE(CV_64F, CV_MAT_CN(objectPoints->type))));
		cvConvert(objectPoints, matM);
	}



	if (CV_IS_CONT_MAT(imagePoints->type) &&
		(CV_MAT_DEPTH(imagePoints->type) == CV_32F || CV_MAT_DEPTH(imagePoints->type) == CV_64F) &&
		((imagePoints->rows == 1 && CV_MAT_CN(imagePoints->type) == 2) ||
		(imagePoints->rows == count && CV_MAT_CN(imagePoints->type)*imagePoints->cols == 2) ||
			(imagePoints->rows == 2 && CV_MAT_CN(imagePoints->type) == 1 && imagePoints->cols == count)))
	{
		_m.reset(cvCreateMat(imagePoints->rows, imagePoints->cols, CV_MAKETYPE(CV_64F, CV_MAT_CN(imagePoints->type))));
		cvConvert(imagePoints, _m);
	}


	M = (CvPoint3D64f*)matM->data.db;
	m = (CvPoint2D64f*)_m->data.db;


	///////////여기서 R 생성 /////////////
	if (r_vec->rows == 3 && r_vec->cols == 3)
	{
		_r = cvMat(3, 1, CV_64FC1, r);
		cvRodrigues2(r_vec, &_r);
		cvRodrigues2(&_r, &matR, &_dRdr);
		cvCopy(r_vec, &matR);
	}
	else
	{
		_r = cvMat(r_vec->rows, r_vec->cols, CV_MAKETYPE(CV_64F, CV_MAT_CN(r_vec->type)), r);
		cvConvert(r_vec, &_r);
		cvRodrigues2(&_r, &matR, &_dRdr);
	}
	///////////////////////////////

	_t = cvMat(t_vec->rows, t_vec->cols, CV_MAKETYPE(CV_64F, CV_MAT_CN(t_vec->type)), t);
	cvConvert(t_vec, &_t);


	cvConvert(A, &_a);
	fx = a[0]; fy = a[4];
	cx = a[2]; cy = a[5];

	if (fixedAspectRatio)
		fx = fy * aspectRatio;

	double dist;

	/**/
	CvPoint3D64f* M_new = (CvPoint3D64f*)matM->data.db;
	double *dists;
	dists = (double *)malloc(sizeof(double)*count);
	char *indices;
	indices = (char *)malloc(sizeof(char)*count);

	double max = 0;
	int max_index = 0;

	double proj31 = (a[6] * R[0]) + (a[7] * R[3]) + (a[8] * R[6]);
	double proj32 = (a[6] * R[1]) + (a[7] * R[4]) + (a[8] * R[7]);
	double proj33 = (a[6] * R[2]) + (a[7] * R[5]) + (a[8] * R[8]);
	double proj34 = (a[6] * t[0]) + (a[7] * t[1]) + (a[8] * t[2]);

	vector<Point2f> _2p;

	int count1 = 0, count2 = 0;

	for (i = 0; i < count; i++)
	{

		double X = M[i].x, Y = M[i].y, Z = M[i].z;
		double x = R[0] * X + R[1] * Y + R[2] * Z + t[0];
		double y = R[3] * X + R[4] * Y + R[5] * Z + t[1];
		double z = R[6] * X + R[7] * Y + R[8] * Z + t[2];
		double r2, r4, r6, a1, a2, a3, cdist, icdist2;
		double xd, yd, xd0, yd0, invProj;

		Vec3d vecTilt;
		Vec3d dVecTilt;
		Matx22d dMatTilt;
		Vec2d dXdYd;

		z = z ? 1. / z : 1;
		x *= z; y *= z;

		r2 = x * x + y * y;
		r4 = r2 * r2;
		r6 = r4 * r2;
		a1 = 2 * x*y;
		a2 = r2 + 2 * x*x;
		a3 = r2 + 2 * y*y;
		cdist = 1 + k[0] * r2 + k[1] * r4 + k[4] * r6;


		icdist2 = 1. / (1 + k[5] * r2 + k[6] * r4 + k[7] * r6);
		xd0 = x * cdist*icdist2 + k[2] * a1 + k[3] * a2 + k[8] * r2 + k[9] * r4;
		yd0 = y * cdist*icdist2 + k[2] * a3 + k[3] * a1 + k[10] * r2 + k[11] * r4;

		// additional distortion by projecting onto a tilt plane
		vecTilt = matTilt * Vec3d(xd0, yd0, 1);
		invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;

		xd = invProj * vecTilt(0);
		yd = invProj * vecTilt(1);


		double w = (proj31 * X) + (proj32 * Y) + (proj33 * Z) + proj34;

		dist = find_point_dist(w, camera);

		double m_x = xd * fx + cx;
		double m_y = yd * fy + cy;

		if (((int)m_x < 0) || ((int)m_y < 0) || ((int)m_x > _width - 1) || ((int)m_y > _height - 1)) continue;

		if (depthimg.at<double>((int)m_y, (int)m_x) == -1)
			depthimg.at<double>((int)m_y, (int)m_x) = dist;

		else
		{
			if (dist < depthimg.at<double>((int)m_y, (int)m_x))
				depthimg.at<double>((int)m_y, (int)m_x) = dist;

			else continue;
		}

		count2++;

		Point2i point;
		m_y = _height - m_y - 1;
		point.x = (int)m_x;
		point.y = (int)m_y;

		_2p.push_back(point);
		m[i].x = (int)m_x;
		m[i].y = (int)m_y;


		projectimg.at<Vec3b>(point.y, point.x)[0] = (uchar)RGB[i].x;
		projectimg.at<Vec3b>(point.y, point.x)[1] = (uchar)RGB[i].y;
		projectimg.at<Vec3b>(point.y, point.x)[2] = (uchar)RGB[i].z;

	}


	if (_m != imagePoints)
		cvConvert(_m, imagePoints);

}

void YUV_dev(vector<PPC> PPC, vector<vector<float>>& dev_pointnum_percent, vector<float>& point_num_per_color)
{
	vector<vector<float>> temp_dev(total_num_cameras, vector<float>(8, 0));
	vector<float> temp_point_num(total_num_cameras, 0);

	for (int point_num = 0; point_num < PPC.size(); point_num++) {
		float avr_u = 0, avr_v = 0;
		int cam_number = 0;
		bool first_value = 0;
		int prev_u = 0, prev_v = 0;
		for (int cam = 0; cam < total_num_cameras; cam++) {

			Mat yuv(1, 1, CV_8UC3); // BGR = YUV
			yuv.at<Vec3b>(0, 0)[0] = PPC[point_num].color[cam][2];
			yuv.at<Vec3b>(0, 0)[1] = PPC[point_num].color[cam][1];
			yuv.at<Vec3b>(0, 0)[2] = PPC[point_num].color[cam][0];
			//cvtColor(yuv, yuv, CV_BGR2YUV);

			if (PPC[point_num].occlusion_pattern[cam] == true && !first_value) {
				first_value = true;
				prev_u = (int)yuv.at<Vec3b>(0, 0)[1];
				prev_v = (int)yuv.at<Vec3b>(0, 0)[2];
				continue;
			}
			else if (PPC[point_num].occlusion_pattern[cam] == true && first_value) {
				cam_number++;
				avr_u += abs(prev_u - (int)yuv.at<Vec3b>(0, 0)[1]);
				avr_v += abs(prev_v - (int)yuv.at<Vec3b>(0, 0)[2]);
				prev_u = (int)yuv.at<Vec3b>(0, 0)[1];
				prev_v = (int)yuv.at<Vec3b>(0, 0)[2];
			}
		}
		cam_number++;
		temp_point_num[cam_number - 1] += 1;

		if (cam_number == 1) {
			temp_dev[cam_number - 1][0] += 1;
			continue;
		}

		float avr_dev = (avr_u + avr_v) / ((cam_number - 1) * 2.0);
		//cout << avr_dev << "\t" << avr_u << "\t" << avr_v << "\t" << cam_number << endl;

		if (avr_dev >= 0 && avr_dev <= 1) temp_dev[cam_number - 1][0] += 1;
		else if (avr_dev >= 1 && avr_dev <= 2) temp_dev[cam_number - 1][1] += 1;
		else if (avr_dev >= 2 && avr_dev <= 3) temp_dev[cam_number - 1][2] += 1;
		else if (avr_dev >= 3 && avr_dev <= 4) temp_dev[cam_number - 1][3] += 1;
		else if (avr_dev >= 4 && avr_dev <= 5) temp_dev[cam_number - 1][4] += 1;
		else if (avr_dev >= 5 && avr_dev <= 6) temp_dev[cam_number - 1][5] += 1;
		else if (avr_dev >= 6 && avr_dev <= 7) temp_dev[cam_number - 1][6] += 1;
		else if (avr_dev >= 7 && avr_dev < 8) temp_dev[cam_number - 1][7] += 1;
	}

	dev_pointnum_percent = temp_dev;
	point_num_per_color = temp_point_num;
}

void HSV_dev(vector<PPC> PPC, vector<vector<float>>& dev_pointnum_percent, vector<float>& point_num_per_color)
{
	for (int point_num = 0; point_num < PPC.size(); point_num++) {
		float avr_h = 0, avr_s = 0, avr_v = 0;
		float avr_h_2 = 0, avr_s_2 = 0, avr_v_2 = 0;
		int cam_number = 0;
		for (int cam = 0; cam < total_num_cameras; cam++) {

			Mat hsv(1, 1, CV_8UC3);
			hsv.at<Vec3b>(0, 0)[0] = PPC[point_num].color[cam][2];
			hsv.at<Vec3b>(0, 0)[1] = PPC[point_num].color[cam][1];
			hsv.at<Vec3b>(0, 0)[2] = PPC[point_num].color[cam][0];
			cvtColor(hsv, hsv, CV_BGR2HSV);

			if (PPC[point_num].occlusion_pattern[cam] == true) {
				cam_number++;
				avr_h += (float)hsv.at<Vec3b>(0, 0)[0];
				avr_s += (float)hsv.at<Vec3b>(0, 0)[1];
				avr_v += (float)hsv.at<Vec3b>(0, 0)[2];
				avr_h_2 += float((float)hsv.at<Vec3b>(0, 0)[0] * (float)hsv.at<Vec3b>(0, 0)[0]);
				avr_s_2 += float((float)hsv.at<Vec3b>(0, 0)[1] * (float)hsv.at<Vec3b>(0, 0)[1]);
				avr_v_2 += float((float)hsv.at<Vec3b>(0, 0)[2] * (float)hsv.at<Vec3b>(0, 0)[2]);
			}
		}
		point_num_per_color[cam_number - 1] += 1;

		avr_h /= cam_number;
		avr_s /= cam_number;
		avr_v /= cam_number;
		avr_h_2 /= cam_number;
		avr_s_2 /= cam_number;
		avr_v_2 /= cam_number;

		float dev_h = 0, dev_s = 0, dev_v = 0;


		dev_h = sqrt(avr_h_2 - avr_h * avr_h);
		dev_s = sqrt(avr_s_2 - avr_s * avr_s);
		dev_v = sqrt(avr_v_2 - avr_v * avr_v);

		float avr_dev = (dev_h + dev_s + dev_v) / 3.0;


		if (avr_dev >= 0 && avr_dev < 5) {
			dev_pointnum_percent[cam_number - 1][0] += 1;
		}
		else if (avr_dev >= 5 && avr_dev < 10) {
			dev_pointnum_percent[cam_number - 1][1] += 1;
		}
		else if (avr_dev >= 10 && avr_dev < 15) {
			dev_pointnum_percent[cam_number - 1][2] += 1;
		}
		else if (avr_dev >= 15 && avr_dev < 20) {
			dev_pointnum_percent[cam_number - 1][3] += 1;
		}
		else if (avr_dev >= 20) {
			dev_pointnum_percent[cam_number - 1][4] += 1;
		}

	}

	//for (int cam = 0; cam < total_num_cameras; cam++) {
	//   for (int i = 0; i < 5; i++) {
	//      dev_pointnum_percent[cam][i] = dev_pointnum_percent[cam][i] / (float)point_num_per_color[cam] * 100;
	//   }
	//}


	//int total = 0;

	////fout << "COLOR N개의 point 개수 :::::::::::::::::" << endl;

	//for (int cam = 0; cam < total_num_cameras; cam++) {
	//   cout << cam + 1 << "개: " << point_num_per_color[cam] << "(" << point_num_per_color[cam] / PPC.size() * 100 << "%)" << endl;
	//   //fout << cam + 1 << "개: " << point_num_per_color[cam] << "(" << point_num_per_color[cam] / PPC.size() * 100 << "%)" << endl;
	//   //point_num_per_color[cam] = point_num_per_color[cam] / PPC.size() * 100;
	//   //cout << cam + 1 << "개: " << point_num_per_color[cam] << "%" << endl;
	//}

	//for (int cam = 0; cam < total_num_cameras; cam++) {
	//   cout << cam + 1 << "개: " << dev_pointnum_percent[cam][0] << "/" << dev_pointnum_percent[cam][1] << "/" << dev_pointnum_percent[cam][2] << "/" << dev_pointnum_percent[cam][3] << "/" << dev_pointnum_percent[cam][4] << endl;
	//   //fout << cam + 1 << "개: " << dev_pointnum_percent[cam][0] << "/" << dev_pointnum_percent[cam][1] << "/" << dev_pointnum_percent[cam][2] << "/" << dev_pointnum_percent[cam][3] << "/" << dev_pointnum_percent[cam][4] << endl;
	//}
}

void RGB_dev(vector<PPC> PPC, vector<vector<float>>& dev_pointnum_percent, vector<float>& point_num_per_color)
{
	for (int point_num = 0; point_num < PPC.size(); point_num++) {

		float avr_r = 0, avr_g = 0, avr_b = 0;
		float avr_r_2 = 0, avr_g_2 = 0, avr_b_2 = 0;
		int cam_number = 0;
		for (int cam = 0; cam < total_num_cameras; cam++) {
			Mat hsv(1, 1, CV_8UC3);
			hsv.at<Vec3b>(0, 0)[0] = PPC[point_num].color[cam][2];
			hsv.at<Vec3b>(0, 0)[1] = PPC[point_num].color[cam][1];
			hsv.at<Vec3b>(0, 0)[2] = PPC[point_num].color[cam][0];
			cvtColor(hsv, hsv, CV_BGR2HSV);




			if (PPC[point_num].occlusion_pattern[cam] == true) {
				cam_number++;
				avr_r += (float)PPC[point_num].color[cam][0];
				avr_g += (float)PPC[point_num].color[cam][1];
				avr_b += (float)PPC[point_num].color[cam][2];
				avr_r_2 += float(PPC[point_num].color[cam][0] * PPC[point_num].color[cam][0]);
				avr_g_2 += float(PPC[point_num].color[cam][1] * PPC[point_num].color[cam][1]);
				avr_b_2 += float(PPC[point_num].color[cam][2] * PPC[point_num].color[cam][2]);
			}
		}
		point_num_per_color[cam_number - 1] += 1;

		avr_r /= cam_number;
		avr_g /= cam_number;
		avr_b /= cam_number;
		avr_r_2 /= cam_number;
		avr_g_2 /= cam_number;
		avr_b_2 /= cam_number;

		float dev_r = 0, dev_g = 0, dev_b = 0;


		dev_r = sqrt(avr_r_2 - avr_r * avr_r);
		dev_g = sqrt(avr_g_2 - avr_g * avr_g);
		dev_b = sqrt(avr_b_2 - avr_b * avr_b);

		float avr_dev = (dev_r + dev_g + dev_b) / 3.0;
		if (avr_dev >= 0 && avr_dev < 5) {
			dev_pointnum_percent[cam_number - 1][0] += 1;
		}
		else if (avr_dev >= 5 && avr_dev < 10) {
			dev_pointnum_percent[cam_number - 1][1] += 1;
		}
		else if (avr_dev >= 10 && avr_dev < 15) {
			dev_pointnum_percent[cam_number - 1][2] += 1;
		}
		else if (avr_dev >= 15 && avr_dev < 20) {
			dev_pointnum_percent[cam_number - 1][3] += 1;
		}
		else if (avr_dev >= 20) {
			dev_pointnum_percent[cam_number - 1][4] += 1;
		}
	}

	for (int cam = 0; cam < total_num_cameras; cam++) {
		for (int i = 0; i < 5; i++) {
			dev_pointnum_percent[cam][i] = dev_pointnum_percent[cam][i] / (float)point_num_per_color[cam] * 100;
		}
	}


	int total = 0;

	//fout << "COLOR N개의 point 개수 :::::::::::::::::" << endl;

	for (int cam = 0; cam < total_num_cameras; cam++) {
		cout << cam + 1 << "개: " << point_num_per_color[cam] << "(" << point_num_per_color[cam] / PPC.size() * 100 << "%)" << endl;
		//fout << cam + 1 << "개: " << point_num_per_color[cam] << "(" << point_num_per_color[cam] / PPC.size() * 100 << "%)" << endl;
		//point_num_per_color[cam] = point_num_per_color[cam] / PPC.size() * 100;
		//cout << cam + 1 << "개: " << point_num_per_color[cam] << "%" << endl;
	}

	for (int cam = 0; cam < total_num_cameras; cam++) {
		cout << cam + 1 << "개: " << dev_pointnum_percent[cam][0] << "/" << dev_pointnum_percent[cam][1] << "/" << dev_pointnum_percent[cam][2] << "/" << dev_pointnum_percent[cam][3] << "/" << dev_pointnum_percent[cam][4] << endl;
		//fout << cam + 1 << "개: " << dev_pointnum_percent[cam][0] << "/" << dev_pointnum_percent[cam][1] << "/" << dev_pointnum_percent[cam][2] << "/" << dev_pointnum_percent[cam][3] << "/" << dev_pointnum_percent[cam][4] << endl;
	}
}

void printPSNRWithoutBlackPixel(
	vector<Mat> orig_imgs,
	vector<Mat> proj_imgs,
	vector<float>& psnrs_y,
	vector<float>& psnrs_u,
	vector<float>& psnrs_v,
	vector<int> &num_holes)
{
	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
	{
		double mse = 0, sum = 0;
		int cnt = 0;
		float tmp = 0;

		Mat yuv_orig[3];
		Mat yuv_proj[3];

		split(orig_imgs[cam_num], yuv_orig);
		split(proj_imgs[cam_num], yuv_proj);

		int n = 0;

		for (int v = 0; v < _height; v++)
			for (int u = 0; u < _width; u++) {

				if (yuv_proj[0].at<uchar>(v, u) == 0 && yuv_proj[1].at<uchar>(v, u) == 0 && yuv_proj[2].at<uchar>(v, u) == 0)
					n++;

				else {
					tmp = yuv_orig[0].at<uchar>(v, u) - yuv_proj[0].at<uchar>(v, u);
					cnt++;
					sum += tmp * tmp;

					tmp = yuv_orig[1].at<uchar>(v, u) - yuv_proj[1].at<uchar>(v, u);
					cnt++;
					sum += tmp * tmp;

					tmp = yuv_orig[2].at<uchar>(v, u) - yuv_proj[2].at<uchar>(v, u);
					cnt++;
					sum += tmp * tmp;
				}
			}

		mse = sum / cnt;
		float psnr = 10 * log10(255 * 255 / mse);

		//mse_y = sum_y / cnt_y;
		//psnr_y = 10 * log10(255 * 255 / mse_y);
		////psnr_y = 20 * log10(255 / sqrt(mse_y));
		//
		//mse_u = sum_u / cnt_u;
		//psnr_u = 10 * log10(255 * 255 / mse_u);
		////psnr_u = 20 * log10(255 / sqrt(mse_u));
		//
		//mse_v = sum_v / cnt_v;
		//psnr_v = 10 * log10(255 * 255 / mse_v);
		////psnr_v = 20 * log10(255 / sqrt(mse_v));


		num_holes.push_back(n);
		//num_holes.push_back(n);
		psnrs_y.push_back(psnr);
		//psnrs_u.push_back(psnr_u);
		//psnrs_v.push_back(psnr_v);
	}

	cout << "num of holes ::::::::::::::::" << endl;

	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
		cout << "cam" << cam_num << " : " << num_holes[cam_num] << endl;
	}

	cout << "PSNR without black pixel ::::::::::::::::::" << endl;

	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
		cout << "cam" << cam_num << " : " << psnrs_y[cam_num] << endl;
	}

	//cout << "U channel :::::::::::::::" << endl;
	//fout << "U channel :::::::::::::::" << endl;
	//for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
	//	cout << "cam" << cam_num << " : " << psnrs_u[cam_num] << endl;
	//	fout << "cam" << cam_num << " : " << psnrs_u[cam_num] << endl;
	//}
	//
	//cout << "V channel :::::::::::::::" << endl;
	//fout << "V channel :::::::::::::::" << endl;
	//for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
	//	cout << "cam" << cam_num << " : " << psnrs_v[cam_num] << endl;
	//	fout << "cam" << cam_num << " : " << psnrs_v[cam_num] << endl;
	//}
}

void printPSNRWithBlackPixel(
	vector<Mat> orig_imgs,
	vector<Mat> proj_imgs,
	vector<float>& psnrs_y,
	vector<float>& psnrs_u,
	vector<float>& psnrs_v)
{
	float avgPSNR = 0.0;
	float avgNumofPixel = 0.0;

	vector<float> PSNR_vec;
	vector<int> hole_num_vec;

	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
	{
		double mse = 0, sum = 0;
		int cnt = 0;
		float tmp = 0;

		Mat yuv_orig[3];
		Mat yuv_proj[3];

		split(orig_imgs[cam_num], yuv_orig);
		split(proj_imgs[cam_num], yuv_proj);

		int n = 0;

		for (int v = 0; v < _height; v++)
			for (int u = 0; u < _width; u++) {

				if (yuv_proj[0].at<uchar>(v, u) == 0 && yuv_proj[1].at<uchar>(v, u) == 0 && yuv_proj[2].at<uchar>(v, u) == 0) {
					n++;
				}

				tmp = yuv_orig[0].at<uchar>(v, u) - yuv_proj[0].at<uchar>(v, u);
				cnt++;
				sum += tmp * tmp;

				tmp = yuv_orig[1].at<uchar>(v, u) - yuv_proj[1].at<uchar>(v, u);
				cnt++;
				sum += tmp * tmp;

				tmp = yuv_orig[2].at<uchar>(v, u) - yuv_proj[2].at<uchar>(v, u);
				cnt++;
				sum += tmp * tmp;
			}

		mse = sum / cnt;
		float psnr = 10 * log10(255 * 255 / mse);


		PSNR_vec.push_back(psnr);
		hole_num_vec.push_back(n);

		avgPSNR += psnr;
		avgNumofPixel += n;

		psnrs_y.push_back(psnr);
	}
	cout << "PSNR with black pixel ::::::::::::::::::" << endl;

	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
		cout << "cam" << cam_num << " : " << psnrs_y[cam_num] << endl;
	}

	avgPSNR /= total_num_cameras;
	avgNumofPixel /= total_num_cameras;
}

double sum(vector<double> a)
{
	double s = 0;
	for (int i = 0; i < a.size(); i++)
	{
		s += a[i];
	}
	return s;
}

double mean(vector<double> a)
{
	return sum(a) / a.size();
}

double sqsum(vector<double> a)
{
	double s = 0;
	for (int i = 0; i < a.size(); i++)
	{
		s += pow(a[i], 2);
	}
	return s;
}

double stdev(vector<double> nums)
{
	double N = nums.size();
	return pow(sqsum(nums) / N - pow(sum(nums) / N, 2), 0.5);
}

vector<double> operator-(vector<double> a, double b)
{
	vector<double> retvect;
	for (int i = 0; i < a.size(); i++)
	{
		retvect.push_back(a[i] - b);
	}
	return retvect;
}

vector<double> operator*(vector<double> a, vector<double> b)
{
	vector<double> retvect;
	for (int i = 0; i < a.size(); i++)
	{
		retvect.push_back(a[i] * b[i]);
	}
	return retvect;
}

double pearsoncoeff(vector<double> X, vector<double> Y)
{
	return sum((X - mean(X)) * (Y - mean(Y))) / (X.size() * stdev(X) * stdev(Y));
}

double correlationCoefficient(vector<double> X, vector<double> Y, int n)
{

	double sum_X = 0, sum_Y = 0, sum_XY = 0;
	double squareSum_X = 0, squareSum_Y = 0;

	for (int i = 0; i < n; i++)
	{
		// sum of elements of array X. 
		sum_X = sum_X + X[i];

		// sum of elements of array Y. 
		sum_Y = sum_Y + Y[i];

		// sum of X[i] * Y[i]. 
		sum_XY = sum_XY + X[i] * Y[i];

		// sum of square of array elements. 
		squareSum_X = squareSum_X + X[i] * X[i];
		squareSum_Y = squareSum_Y + Y[i] * Y[i];
	}

	// use formula for calculating correlation coefficient. 
	double corr = (double)(n * sum_XY - sum_X * sum_Y)
		/ sqrt((n * squareSum_X - sum_X * sum_X)
			* (n * squareSum_Y - sum_Y * sum_Y));

	double temp = sqrt((n * squareSum_X - sum_X * sum_X)
		* (n * squareSum_Y - sum_Y * sum_Y));

	/*cout << "분모 ::\t " << temp << endl;
	cout << "squareSum_X ::\t" << squareSum_X << endl;
	cout << "squareSum_Y ::\t" << squareSum_Y << endl;
	cout << "sum_X ::\t" << sum_X << endl;
	cout << "sum_X ::\t" << sum_Y << endl;
	cout << "N ::\t " << n << endl;*/

	return corr;
}

double cal_color_sim(vector<double> X, vector<double> Y)
{
	double abs_sum = 0;
	double result = 0;
	for (int i = 0; i < X.size(); i++) {
		abs_sum = abs(X[i] - Y[i]);
	}

	result = 1 - (abs_sum / 256);
	return result;
}

int combination(int n, int r) {
	if (n == r || r == 0) return 1;
	else return combination(n - 1, r - 1) + combination(n - 1, r);
}

// frustum voxelization 시 RT로 projection matrix를 계산하는 함수
Matrix4d compute_projection_matrices(int cam_num)
{
	Matrix3Xd camRT(3, 4);
	Matrix4d camP;

	// The extrinsic matrix
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			camRT(i, j) = m_CalibParams[cam_num].m_RotMatrix(i, j);

	for (int i = 0; i < 3; i++)
		camRT(i, 3) = m_CalibParams[cam_num].m_Trans(i, 0);

	// Multiply the intrinsic matrix by the extrinsic matrix to find our projection matrix
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 4; j++) {
			camP(i, j) = 0.0;

			for (int k = 0; k < 3; k++)
				camP(i, j) += m_CalibParams[cam_num].m_K(i, k) * camRT(k, j);
		}

	camP(3, 0) = 0.0;
	camP(3, 1) = 0.0;
	camP(3, 2) = 0.0;
	camP(3, 3) = 1.0;

	return camP;
}

PointCloud<PointXYZRGB>::Ptr make_incremental_PC(vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	vector<char> camera_order,
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	vector<int> &num_p, 
	int th, int d_th) {

	PointCloud<PointXYZRGB>::Ptr incremental_PC(new PointCloud<PointXYZRGB>);

	vector<int> temp_num_p;
	incremental_PC = pointclouds[(int)camera_order[0]];
	temp_num_p.push_back(incremental_PC->points.size());

	int u, v;
	int num_pt = incremental_PC->points.size();
	//cout << num_pt << endl;

	for (int i = 1; i < temp_num_cameras; i++) {
		int cam = (int)camera_order[i];

		Mat confirm_img(_height, _width, CV_32S, -1);
		//Mat test_img(_height, _width, CV_8UC1, Scalar::all(0));
		//Mat test_img2(_height, _width, CV_8UC1, Scalar::all(0));

		for (int p_num = 0; p_num < incremental_PC->points.size(); p_num++) {

			PointXYZRGB p = incremental_PC->points[p_num];
			projection_XYZ_2_UV(m_CalibParams[cam].m_ProjMatrix, p.x, p.y, p.z, u, v);

			if ((u < 0) || (v < 0) || (u > _width - 1) || (v > _height - 1)) continue;

			confirm_img.at<int>(v, u) = p_num;
			//test_img.at<uchar>(v, u) = 255;
		}

		double X = 0., Y = 0., Z;
		Vec3b d;
		Vec3s d_s;
		for (int v = 0; v < _height; v++) {
			for (int u = 0; u < _width; u++) {

				Vec3b color = color_imgs[cam].at<Vec3b>(v, u);
				if (confirm_img.at<int>(v, u) == -1) {

					if (mode == Poznan_Fencing) {

						d_s = depth_imgs[cam].at<Vec3s>(v, u);
						Z = depth_level_2_Z_s(d_s[0]);
						Z = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z, &X, &Y);
					}
					else if (mode == MSR3DVideo_Ballet) {

						d = depth_imgs[cam].at<Vec3b>(v, u);
						Z = depth_level_2_Z(d[0]);
						projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y);
					}

					PointXYZRGB p;

					p.x = X;
					p.y = Y;
					p.z = Z;
					p.r = color[2];
					p.g = color[1];
					p.b = color[0];

					incremental_PC->points.push_back(p);
					num_pt++;
				}
				else {
					int p_num = confirm_img.at<int>(v, u);
					if (confirm_point_modified(cam, incremental_PC->points[p_num], color_imgs[cam], depth_imgs[cam], th, d_th)) {
						//true면 차이작, false면 차이큼
						//test_img2.at<uchar>(v, u) = 255;
						continue;
					}
					else {
						if (mode == Poznan_Fencing) {

							d_s = depth_imgs[cam].at<Vec3s>(v, u);
							Z = depth_level_2_Z_s(d_s[0]);
							Z = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z, &X, &Y);
						}
						else if (mode == MSR3DVideo_Ballet) {

							d = depth_imgs[cam].at<Vec3b>(v, u);
							Z = depth_level_2_Z(d[0]);
							projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y);
						}

						PointXYZRGB p;

						p.x = X;
						p.y = Y;
						p.z = Z;
						p.r = color[2];
						p.g = color[1];
						p.b = color[0];

						incremental_PC->points.push_back(p);
						num_pt++;
					}
				}
			}
		}

		temp_num_p.push_back(num_pt);
		//cout << num_pt << endl;
	}
	
	num_p = temp_num_p;

	return incremental_PC;
}

PointCloud<PointXYZRGB>::Ptr quantization(PointCloud<PointXYZRGB>::Ptr pointcloud) {

	PointCloud<PointXYZRGB>::Ptr quantized_PC(new PointCloud<PointXYZRGB>);

	vector<float> min(3), max(3);

	find_min_max(pointcloud, min, max);

	for (int point_idx = 0; point_idx < pointcloud->points.size(); point_idx++)
	{
		PointXYZRGB p;
		p.r = pointcloud->points[point_idx].r;
		p.g = pointcloud->points[point_idx].g;
		p.b = pointcloud->points[point_idx].b;
		p.x = (pointcloud->points[point_idx].x - min[0]) / (max[0] - min[0]) * 2048;
		p.y = (pointcloud->points[point_idx].y - min[1]) / (max[1] - min[1]) * 2048;
		p.z = (pointcloud->points[point_idx].z - min[2]) / (max[2] - min[2]) * 2048;

		quantized_PC->points.push_back(p);
	}

	return quantized_PC;
}

void make_proj_img_vec(
	PointCloud<PointXYZRGB>::Ptr pointcloud,
	vector<Mat>& proj_img_vec,
	vector<Mat>& depth_value_img_vec,
	vector<int> num_p)
{
	vector<char> camera_order;
	if (mode == 0) camera_order = { 4,3,5,2,6,1,7,0 };
	else camera_order = { 5,4,6,3,7,2,8,1,9,0 };

	for (int i = 0; i < total_num_cameras; i++)
	{
		Mat color_img(_height, _width, CV_8UC3, Scalar(0));

		Mat depth_value_img(_height, _width, CV_64F, -1);

		projection(pointcloud, num_p, camera_order[i], i, color_img, depth_value_img);

		proj_img_vec[camera_order[i]] = color_img;

		depth_value_img_vec[camera_order[i]] = depth_value_img;
	}
}

Mat find_hole(
	Mat depthimg,
	int window_size)
{
	double avr_depth_value = 0;
	double ratio = 0.25;

	for (int i = 0; i < depthimg.rows; i++)
		for (int j = 0; j < depthimg.cols; j++)
			avr_depth_value += depthimg.at<double>(i, j);

	avr_depth_value = avr_depth_value / (_width * _height);

	Mat hole_image(_height, _width, CV_32F, -1);

	for (int rownum = 0; rownum < _height; rownum++) {
		for (int colnum = 0; colnum < _width; colnum++) {

			double center_pix_value = depthimg.at<double>(rownum, colnum);

			if (center_pix_value == -1)
			{
				hole_image.at<float>(rownum, colnum) = 0;
				continue;
			}
			int different_pixel = 0;
			int out_of_range_pixel = 0;

			for (int h = rownum - window_size; h <= rownum + window_size; h++)
				for (int w = colnum - window_size; w <= colnum + window_size; w++) {
					if (h < 0 || w < 0 || h >= _height || w >= _width) {
						out_of_range_pixel++;
						continue;
					}
					if (abs(depthimg.at<double>(h, w) - center_pix_value) > avr_depth_value* ratio)
						different_pixel++;
				}
			
			if (different_pixel < ceil((pow((2 * window_size + 1), 2) - out_of_range_pixel) / 2))
				hole_image.at<float>(rownum, colnum) = 1;
			else 
				hole_image.at<float>(rownum, colnum) = 0;
		}
	}

	return hole_image;
}

Mat make_filled_image(
	Mat depthimg,
	Mat colorimg,
	Mat hole_image,
	int window_size)
{
	Mat filled_image(_height, _width, CV_8UC3, Scalar::all(0));

	for (int rownum = 0; rownum < _height; rownum++)
	{
		for (int colnum = 0; colnum < _width; colnum++)
		{
			bool is_not_hole = hole_image.at<float>(rownum, colnum);
			if (is_not_hole)
			{
				filled_image.at<Vec3b>(rownum, colnum)[0] = colorimg.at<Vec3b>(rownum, colnum)[0];
				filled_image.at<Vec3b>(rownum, colnum)[1] = colorimg.at<Vec3b>(rownum, colnum)[1];
				filled_image.at<Vec3b>(rownum, colnum)[2] = colorimg.at<Vec3b>(rownum, colnum)[2];
				continue;
			}
			else
			{
				vector<ushort> pixel_sum(3);
				int pixel_count = 0;

				for (int h = rownum - window_size; h <= rownum + window_size; h++)
				{
					for (int w = colnum - window_size; w <= colnum + window_size; w++)
					{
						if (h < 0 || w < 0 || h >= _height || w >= _width) continue;
						else if (hole_image.at<float>(h, w) == 0) continue;
						else
						{
							pixel_sum[0] += colorimg.at<Vec3b>(h, w)[0];
							pixel_sum[1] += colorimg.at<Vec3b>(h, w)[1];
							pixel_sum[2] += colorimg.at<Vec3b>(h, w)[2];
							pixel_count++;
						}
					}
				}
				if (pixel_count == 0)
				{
					filled_image.at<Vec3b>(rownum, colnum)[0] = 255;
					filled_image.at<Vec3b>(rownum, colnum)[1] = 255;
					filled_image.at<Vec3b>(rownum, colnum)[2] = 255;
					//continue;
				}
				else
				{
					filled_image.at<Vec3b>(rownum, colnum)[0] = uchar(pixel_sum[0] / pixel_count);
					filled_image.at<Vec3b>(rownum, colnum)[1] = uchar(pixel_sum[1] / pixel_count);
					filled_image.at<Vec3b>(rownum, colnum)[2] = uchar(pixel_sum[2] / pixel_count);
				}
			}
		}
	}
	return filled_image;
}

void hole_filling(
	vector<Mat> depthimgs,
	vector<Mat> colorimgs,
	vector<Mat>& filled_imgs)
{
	Mat hole_image;

	int window_size = 4;
	for (int num = 0; num < total_num_cameras; num++) {

		hole_image = find_hole(depthimgs[num], window_size);

		filled_imgs[num] = make_filled_image(depthimgs[num], colorimgs[num], hole_image, window_size);
	}
}

void printPSNR(
	vector<Mat> orig_imgs,
	vector<Mat> proj_imgs)
{
	
	for (int i = 0; i < total_num_cameras; i++)
	{
		float mse, psnr, tmp;
		float sum = 0;
		int cnt = 0;

		cvtColor(orig_imgs[i], orig_imgs[i], COLOR_BGR2GRAY);
		cvtColor(proj_imgs[i], proj_imgs[i], COLOR_BGR2GRAY);

		int n = 0;
		for (int v = 0; v < _height; v++)
			for (int u = 0; u < _width; u++)
			{
				if (proj_imgs[i].at<uchar>(v, u) == 0) continue;

				tmp = orig_imgs[i].at<uchar>(v, u) - proj_imgs[i].at<uchar>(v, u);
				cnt++;
				sum += tmp * tmp;
			}

		mse = sum / cnt;
		psnr = 10 * log10(255 * 255 / mse);
		cout << /*"cam" << i << " = " <<*/ psnr << endl;
		if (psnr == INFINITY) continue;
		PSNRs[i] += psnr;
		//writeFile << psnr << endl;
	}
	//writeFile << endl;
}