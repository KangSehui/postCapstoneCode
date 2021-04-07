//   map + filtered minmax + color balancing
#if 1
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include "opencv.hpp"
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/eigen.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/image_depth.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <math.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <map>
#include <list>

using namespace pcl;
using namespace std;
using namespace io;
using namespace cv;
using namespace Eigen;
using namespace visualization;

typedef struct {

	Matrix3d m_K;

	Matrix3d m_RotMatrix;

	Matrix3Xd m_Trans;

	Matrix4d m_ProjMatrix;

} CalibStruct;

int _width, _height, camera_idx_reference, total_num_cameras, total_num_frames;

double MinZ, MaxZ;

string path;

vector<CalibStruct> m_CalibParams;

Vec3b color_rate[8], standard_rate;

void set_parameters()
{
	cout << "mode 0: Ballet data" << endl;

	cout << "--> Enter the data index: ";



	int mode = 0;

	//cin >> mode;

	cout << mode << endl;

	switch (mode)
	{
	case 0:
		_width = 1024;

		_height = 768;

		camera_idx_reference = 4;

		MinZ = 42.0;

		MaxZ = 130.0;

		path = "C:\\Users\\ÇÑÇö¹Î\\Desktop\\pcl_build\\MSR3DVideo-Ballet";

		break;



	default:
		cerr << "Wrong mode!!!" << endl;

		exit(0);
	}
}

void set_color_data(int frame, vector<vector<string>> color_names)
{
	string folder_path;

	Mat color_imgs[8];

	Mat color_roi[8];

	int w1 = 20, h1 = 20, w2 = 100, h2 = 100;

	int roi_size = (w2 - w1) * (h2 - h1);

	Rect roi(w1, h1, w2, h2);

	float color_sum[3];

	for (int camera = 0; camera < 8; camera++)
	{
		folder_path = path + "/cam" + to_string(camera) + "/";

		color_imgs[camera] = imread(folder_path + color_names[camera][frame]);


		color_roi[camera] = color_imgs[camera](roi).clone();

		color_sum[0] = 0; color_sum[1] = 0; color_sum[2] = 0;

		for (int w = 0; w < w2 - w1; w++)

			for (int h = 0; h < h2 - h1; h++)
			{
				color_sum[0] += (float)color_roi[camera].at<Vec3b>(h, w)[0];

				color_sum[1] += (float)color_roi[camera].at<Vec3b>(h, w)[1];

				color_sum[2] += (float)color_roi[camera].at<Vec3b>(h, w)[2];
			}


		color_rate[camera][0] = (unsigned char)(color_sum[0] / roi_size);

		color_rate[camera][1] = (unsigned char)(color_sum[1] / roi_size);

		color_rate[camera][2] = (unsigned char)(color_sum[2] / roi_size);

		cout << "color rate " << camera << " " << (int)color_rate[camera][0] << " " << (int)color_rate[camera][1] << " " << (int)color_rate[camera][2] << endl;
	}

	standard_rate[0] = color_rate[camera_idx_reference][0];

	standard_rate[1] = color_rate[camera_idx_reference][1];

	standard_rate[2] = color_rate[camera_idx_reference][2];
}

void set_color_data() {
	color_rate[0][0] = 143;
	color_rate[0][1] = 142;
	color_rate[0][2] = 111;

	color_rate[1][0] = 148;
	color_rate[1][1] = 146;
	color_rate[1][2] = 108;

	color_rate[2][0] = 136;
	color_rate[2][1] = 137;
	color_rate[2][2] = 103;

	color_rate[3][0] = 141;
	color_rate[3][1] = 141;
	color_rate[3][2] = 103;

	color_rate[4][0] = 134;
	color_rate[4][1] = 134;
	color_rate[4][2] = 98;

	color_rate[5][0] = 136;
	color_rate[5][1] = 136;
	color_rate[5][2] = 100;

	color_rate[6][0] = 138;
	color_rate[6][1] = 136;
	color_rate[6][2] = 98;

	color_rate[7][0] = 141;
	color_rate[7][1] = 127;
	color_rate[7][2] = 116;

	standard_rate[0] = color_rate[camera_idx_reference][0];
	standard_rate[1] = color_rate[camera_idx_reference][1];
	standard_rate[2] = color_rate[camera_idx_reference][2];
}

void get_num_camera_N_frame(
	int& total_num_cameras,
	int& total_num_frames)
{
	//   get cameras
	string camera_path;

	camera_path = path + "\\*";



	intptr_t camera_handle;

	struct _finddata_t camera_fd;

	camera_handle = _findfirst(camera_path.c_str(), &camera_fd);

	int cameras = 0;

	while (_findnext(camera_handle, &camera_fd) == 0) cameras++;

	_findclose(camera_handle);



	total_num_cameras = cameras - 2;



	//   get frames
	string frame_path;

	frame_path = path + "\\cam0\\*.jpg";



	intptr_t frame_handle;

	struct _finddata_t frame_fd;

	frame_handle = _findfirst(frame_path.c_str(), &frame_fd);

	int frames = 0;

	do frames++;
	while (_findnext(frame_handle, &frame_fd) == 0);

	_findclose(frame_handle);



	total_num_frames = frames;
}

void load_matrix_data()
{

	string matrix_path;

	matrix_path = path + "\\*.txt";



	intptr_t matrix_handle;

	struct _finddata_t matrix_fd;

	if ((matrix_handle = _findfirst(matrix_path.c_str(), &matrix_fd)) == -1L)

		cout << "No file in directory!" << endl;



	string matrixfile;

	matrixfile = path + "\\" + matrix_fd.name;

	ifstream openFile(matrixfile);

	if (!openFile.is_open())
	{
		cerr << "Failed to open " << endl;

		exit(EXIT_FAILURE);
	}



	double col0, col1, col2, col3;

	int row_count = 0;

	int camera_idx = 0;

	string buffer;

	vector<CalibStruct> temp_CalibParams(total_num_cameras);



	Matrix3Xd temp(3, 1);

	temp << 0, 0, 0;

	for (int camera_idx = 0; camera_idx < total_num_cameras; camera_idx++)

		temp_CalibParams[camera_idx].m_Trans = temp;



	while (!openFile.eof())
	{
		getline(openFile, buffer);



		//   get intrinsics
		while (openFile >> col0 >> col1 >> col2)
		{
			temp_CalibParams[camera_idx].m_K(row_count, 0) = col0;

			temp_CalibParams[camera_idx].m_K(row_count, 1) = col1;

			temp_CalibParams[camera_idx].m_K(row_count, 2) = col2;

			if (row_count > 1) {

				row_count = 0;

				break;
			}

			row_count++;
		}



		//   skip distortion coefficient
		getline(openFile, buffer);

		getline(openFile, buffer);



		//   get extrinsics
		while (openFile >> col0 >> col1 >> col2 >> col3)
		{
			temp_CalibParams[camera_idx].m_RotMatrix(row_count, 0) = col0;

			temp_CalibParams[camera_idx].m_RotMatrix(row_count, 1) = col1;

			temp_CalibParams[camera_idx].m_RotMatrix(row_count, 2) = col2;

			temp_CalibParams[camera_idx].m_Trans(row_count, 0) = col3;

			if (row_count > 1)
			{
				row_count = 0;

				break;
			}

			row_count++;
		}



		getline(openFile, buffer);

		getline(openFile, buffer);



		camera_idx++;
	}



	openFile.close();

	_findclose(matrix_handle);



	m_CalibParams = temp_CalibParams;
}

void compute_projection_matrices()
{
	Matrix3d inMat;

	Matrix3Xd exMat(3, 4);

	for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
	{
		// The intrinsic matrix
		inMat = m_CalibParams[cam_idx].m_K;



		// The extrinsic matrix
		for (int i = 0; i < 3; i++)

			for (int j = 0; j < 3; j++)

				exMat(i, j) = m_CalibParams[cam_idx].m_RotMatrix(i, j);

		for (int i = 0; i < 3; i++)

			exMat(i, 3) = m_CalibParams[cam_idx].m_Trans(i, 0);



		// Multiply the intrinsic matrix by the extrinsic matrix to find our projection matrix
		for (int i = 0; i < 3; i++)

			for (int j = 0; j < 4; j++)
			{
				m_CalibParams[cam_idx].m_ProjMatrix(i, j) = 0.0;

				for (int k = 0; k < 3; k++)

					m_CalibParams[cam_idx].m_ProjMatrix(i, j) += inMat(i, k) * exMat(k, j);
			}

		m_CalibParams[cam_idx].m_ProjMatrix(3, 0) = 0.0;

		m_CalibParams[cam_idx].m_ProjMatrix(3, 1) = 0.0;

		m_CalibParams[cam_idx].m_ProjMatrix(3, 2) = 0.0;

		m_CalibParams[cam_idx].m_ProjMatrix(3, 3) = 1.0;
	}
}

void load_file_name(
	vector<vector<string>>& color_names,
	vector<vector<string>>& depth_names)
{
	string cam_path;

	cam_path = path + "\\cam";

	intptr_t color_handle, depth_handle;

	struct _finddata_t color_fd, depth_fd;

	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
	{
		string color_path = cam_path + to_string(cam_num) + "\\*.jpg";

		string depth_path = cam_path + to_string(cam_num) + "\\*.png";

		color_handle = _findfirst(color_path.c_str(), &color_fd);

		depth_handle = _findfirst(depth_path.c_str(), &depth_fd);

		for (int frame_num = 0; frame_num < total_num_frames; frame_num++)
		{
			color_names[cam_num][frame_num] = color_fd.name;

			depth_names[cam_num][frame_num] = depth_fd.name;

			_findnext(color_handle, &color_fd);

			_findnext(depth_handle, &depth_fd);
		}
	}

	_findclose(color_handle);

	_findclose(depth_handle);
}

double depth_level_2_Z(unsigned char d)
{
	double z;

	z = 1.0 / ((d / 255.0) * (1.0 / MinZ - 1.0 / MaxZ) + 1.0 / MaxZ);

	return z;
}

void projection_UVZ_2_XY(
	Matrix4d projMatrix,
	double u,
	double v,
	double z,
	double* x,
	double* y)
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

	*x = (*y) * (projMatrix(0, 1) - projMatrix(2, 1) * u) + c0 - c2 * u;

	*x /= projMatrix(2, 0) * u - projMatrix(0, 0);
}

double projection_XYZ_2_UV(
	Matrix4d projMatrix,
	double x,
	double y,
	double z,
	int* u,
	int* v)
{
	double w;

	*u = projMatrix(0, 0) * x
		+ projMatrix(0, 1) * y
		+ projMatrix(0, 2) * z
		+ projMatrix(0, 3);

	*v = projMatrix(1, 0) * x
		+ projMatrix(1, 1) * y
		+ projMatrix(1, 2) * z
		+ projMatrix(1, 3);

	w = projMatrix(2, 0) * x
		+ projMatrix(2, 1) * y
		+ projMatrix(2, 2) * z
		+ projMatrix(2, 3);

	*u /= w;

	*v /= w;

	*v = _height - *v - 1.0;

	return w;
}

bool confirm_point(
	int camera,
	PointXYZRGB p,
	vector<Mat> color_imgs)
{
	int u, v;

	int blue, green, red;

	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
	{
		if (camera == cam_num) continue;

		projection_XYZ_2_UV(m_CalibParams[cam_num].m_ProjMatrix, p.x, p.y, -p.z, &u, &v);



		if (u < 0 || v < 0 || u >= _width || v >= _height) return true;

		blue = abs((int)color_imgs[cam_num].at<Vec3b>(v, u)[0] - (int)p.b);

		green = abs((int)color_imgs[cam_num].at<Vec3b>(v, u)[1] - (int)p.g);

		red = abs((int)color_imgs[cam_num].at<Vec3b>(v, u)[2] - (int)p.r);



		if (blue + green + red < 20) return true;
	}

	return false;
}


void color_balancing(
	int camera,
	Vec3b original,
	Vec3b& edited)
{
	float b = (float)original[0] / (float)color_rate[camera][0] * (float)standard_rate[0];

	float g = (float)original[1] / (float)color_rate[camera][1] * (float)standard_rate[1];

	float r = (float)original[2] / (float)color_rate[camera][2] * (float)standard_rate[2];

	edited[0] = (unsigned char)b;

	edited[1] = (unsigned char)g;

	edited[2] = (unsigned char)r;

	//cout << b << "\t" << g << "\t" << r << "\n";
}

PointCloud<PointXYZRGB>::Ptr make_PC(
	int camera,
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs)
{
	PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);

	bool is_good_point = 0;

	for (int y = 1; y < _height - 1; y++)

		for (int x = 1; x < _width - 1; x++)
		{
			Vec3b d = depth_imgs[camera].at<Vec3b>(y, x);

			double Z = depth_level_2_Z(d[0]);

			double X = 0.0;

			double Y = 0.0;

			projection_UVZ_2_XY(m_CalibParams[camera].m_ProjMatrix, x, y, Z, &X, &Y);



			PointXYZRGB p;

			p.x = X;

			p.y = Y;

			p.z = -Z;



			Vec3b color;

			color = color_imgs[camera].at<Vec3b>(y, x);
			//color_balancing(camera, color_imgs[camera].at<Vec3b>(y, x), color);

			p.b = (float)color[0];

			p.g = (float)color[1];

			p.r = (float)color[2];

			//cout << (float)color[0] << "\t" << (float)color[1] << "\t" << (float)color[2] << endl;


			is_good_point = confirm_point(camera, p, color_imgs);

			if (is_good_point)
				pointcloud->points.push_back(p);

			else continue;
		}

	return pointcloud;
}


vector<PointCloud<PointXYZRGB>::Ptr> get_PC_of_every_camera(
	int frame,
	vector<vector<string>> color_names,
	vector<vector<string>> depth_names,
	vector<Mat>& color_imgs)
{
	vector<Mat> imgs(total_num_cameras);

	vector<Mat> imgs2(total_num_cameras);

	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds(total_num_cameras);

	for (int camera = 0; camera < total_num_cameras; camera++)
	{
		string folder_path = path + "/cam" + to_string(camera) + "/";

		Mat color_img = imread(folder_path + color_names[camera][frame]);

		imgs[camera] = color_img;

		Mat depth_img = imread(folder_path + depth_names[camera][frame]);

		imgs2[camera] = depth_img;
	}

	for (int camera = 0; camera < total_num_cameras; camera++)
	{
		PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);

		pointcloud = make_PC(camera, imgs, imgs2);

		pointclouds[camera] = pointcloud;
	}

	color_imgs = imgs;

	return pointclouds;
}

PointCloud<PointXYZRGB>::Ptr make_registered_PC(
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds)
{
	PointCloud<PointXYZRGB>::Ptr registered_PC(new PointCloud<PointXYZRGB>);

	for (int camera = 0; camera < total_num_cameras; camera++)

		for (int point_idx = 0; point_idx < pointclouds[camera]->points.size(); point_idx++)

			registered_PC->points.push_back(pointclouds[camera]->points[point_idx]);

	return registered_PC;
}

PointCloud<PointXYZRGB>::Ptr filtering(PointCloud<PointXYZRGB>::Ptr original_PC)
{
	PointCloud<PointXYZRGB>::Ptr filtered_PC(new PointCloud<PointXYZRGB>);
	StatisticalOutlierRemoval<PointXYZRGB> sor;

	sor.setInputCloud(original_PC);

	sor.setMeanK(50);

	sor.setStddevMulThresh(1.0);

	sor.filter(*filtered_PC);

	return filtered_PC;


}


void view_PC(
	PointCloud<PointXYZRGB>::Ptr pointcloud)
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


void find_min_max(
	PointCloud<PointXYZRGB>::Ptr source_PC,
	vector<float>& min,
	vector<float>& max)
{

	for (int p = 0; p < 3; p++)
	{
		min[p] = 100000000000000;

		max[p] = -100000000000000;
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

map<int, list<PointXYZRGB>> make_3dGrid_map(
	PointCloud<PointXYZRGB>::Ptr registered_PC,
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	vector<char> camera_order,
	int voxel_div_num) {

	vector<float> min(3), max(3);

	PointCloud<PointXYZRGB>::Ptr filtered_PC;

	filtered_PC = filtering(registered_PC);

	cout << "registered PC size:\t" << registered_PC->points.size() << endl;
	cout << "filtered PC size:\t" << filtered_PC->points.size() << endl;

	//view_PC(filtered_PC);


	find_min_max(filtered_PC, min, max);

	cout << min[0] << "\t" << min[1] << "\t" << min[2] << endl;

	cout << max[0] << "\t" << max[1] << "\t" << max[2] << endl;



	float x_size = max[0] - min[0];

	float y_size = max[1] - min[1];

	float z_size = max[2] - min[2];

	int x_voxel_num = voxel_div_num;

	int y_voxel_num = voxel_div_num;

	int z_voxel_num = voxel_div_num;

	int total_voxel_num = x_voxel_num * y_voxel_num * z_voxel_num;

	//vector<vector<PointXYZRGB>> cube(total_voxel_num); 
	map<int, list<PointXYZRGB>> m_cube;


	vector<char> voxel_index(total_voxel_num, -1);

	int x_voxel_index, y_voxel_index, z_voxel_index;

	int cube_index;

	int order = 0;

	while (order < camera_order.size()) {

		for (int point_idx = 0; point_idx < pointclouds[(int)camera_order[order]]->points.size(); point_idx++)
		{

			if (pointclouds[(int)camera_order[order]]->points[point_idx].x < min[0] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].y < min[1] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].z < min[2] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].x > max[0] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].y > max[1] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].z > max[2])
				continue;

			x_voxel_index = (int)floor((pointclouds[(int)camera_order[order]]->points[point_idx].x - min[0]) / x_size * ((float)x_voxel_num - 1));

			y_voxel_index = (int)floor((pointclouds[(int)camera_order[order]]->points[point_idx].y - min[1]) / y_size * ((float)y_voxel_num - 1));

			z_voxel_index = (int)floor((pointclouds[(int)camera_order[order]]->points[point_idx].z - min[2]) / z_size * ((float)z_voxel_num - 1));

			cube_index = x_voxel_index * (y_voxel_num * z_voxel_num) + y_voxel_index * z_voxel_num + z_voxel_index;

			if ((int)voxel_index[cube_index] == -1) {

				PointXYZRGB _point;

				_point = pointclouds[(int)camera_order[order]]->points[point_idx];
				list<PointXYZRGB> PList;

				PList.push_back(_point);

				//cube[cube_index].push_back(_point);
				m_cube.insert(make_pair(cube_index, PList));
				//mm.insert(make_pair(cube_index, 1));

				voxel_index[cube_index] = camera_order[order];

			}

			else if (voxel_index[cube_index] == camera_order[order]) {

				PointXYZRGB _point;

				_point = pointclouds[(int)camera_order[order]]->points[point_idx];


				m_cube.find(cube_index)->second.push_back(_point);

				//cube[cube_index].push_back(_point);

			}

			else continue;

		}

		order++;

	}
	return m_cube;
}

map<int, list<PointXYZRGB>> make_3dGrid_map_(
	PointCloud<PointXYZRGB>::Ptr registered_PC,
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	vector<char> camera_order,
	int voxel_div_num) {

	vector<float> min(3), max(3);

	PointCloud<PointXYZRGB>::Ptr filtered_PC;

	filtered_PC = filtering(registered_PC);

	cout << "registered PC size:\t" << registered_PC->points.size() << endl;
	cout << "filtered PC size:\t" << filtered_PC->points.size() << endl;

	//view_PC(filtered_PC);


	find_min_max(filtered_PC, min, max);

	cout << min[0] << "\t" << min[1] << "\t" << min[2] << endl;

	cout << max[0] << "\t" << max[1] << "\t" << max[2] << endl;



	float x_size = max[0] - min[0];

	float y_size = max[1] - min[1];

	float z_size = max[2] - min[2];

	int x_voxel_num = voxel_div_num;

	int y_voxel_num = voxel_div_num;

	int z_voxel_num = voxel_div_num;

	int total_voxel_num = x_voxel_num * y_voxel_num * z_voxel_num;

	//vector<vector<PointXYZRGB>> cube(total_voxel_num); 
	map<int, list<PointXYZRGB>> m_cube;


	vector<char> voxel_index(total_voxel_num, -1);

	int x_voxel_index, y_voxel_index, z_voxel_index;

	int cube_index;

	int order = 0;

	while (order < total_num_cameras) {

		for (int point_idx = 0; point_idx < pointclouds[(int)camera_order[order]]->points.size(); point_idx++)
		{

			if (pointclouds[(int)camera_order[order]]->points[point_idx].x < min[0] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].y < min[1] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].z < min[2] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].x > max[0] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].y > max[1] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].z > max[2])
				continue;

			x_voxel_index = (int)floor((pointclouds[(int)camera_order[order]]->points[point_idx].x - min[0]) / x_size * ((float)x_voxel_num - 1));

			y_voxel_index = (int)floor((pointclouds[(int)camera_order[order]]->points[point_idx].y - min[1]) / y_size * ((float)y_voxel_num - 1));

			z_voxel_index = (int)floor((pointclouds[(int)camera_order[order]]->points[point_idx].z - min[2]) / z_size * ((float)z_voxel_num - 1));

			cube_index = x_voxel_index * (y_voxel_num * z_voxel_num) + y_voxel_index * z_voxel_num + z_voxel_index;



			if ((int)voxel_index[cube_index] == -1) {

				PointXYZRGB _point;

				_point = pointclouds[(int)camera_order[order]]->points[point_idx];
				list<PointXYZRGB> PList;

				PList.push_back(_point);


				//cube[cube_index].push_back(_point);
				m_cube.insert(make_pair(cube_index, PList));
				//mm.insert(make_pair(cube_index, 1));

				voxel_index[cube_index] = camera_order[order];

			}

			else if (voxel_index[cube_index] != -1) {

				PointXYZRGB _point;

				_point = pointclouds[(int)camera_order[order]]->points[point_idx];

				m_cube.find(cube_index)->second.push_back(_point);
			}

		}
		order++;
	}

	return m_cube;
}

PointXYZRGB merge_points(
	PointXYZRGB p1,
	PointXYZRGB p2)
{
	PointXYZRGB merged;


	merged.x = (p1.x + p2.x) / 2;

	merged.y = (p1.y + p2.y) / 2;

	merged.z = (p1.z + p2.z) / 2;


	merged.r = (p1.r + p2.r) / 2;

	merged.g = (p1.g + p2.g) / 2;

	merged.b = (p1.b + p2.b) / 2;



	return merged;
}

int find_two_points_rgbsum(PointXYZRGB p1, PointXYZRGB p2) {

	int temp_rgbsum;

	temp_rgbsum = abs(p1.r - p2.r) + abs(p1.g - p2.g) + abs(p1.b - p2.b);


	return(temp_rgbsum);

}

void merge_VOXEL(vector<PointXYZRGB>& voxel, int standard_num) {

	vector<PointXYZRGB> temp;
	int p1 = -1, p2 = -1;

	while (1) {

		for (int k = 0; k < voxel.size(); k++) {

			for (int l = k + 1; l < voxel.size(); l++)
			{

				if (find_two_points_rgbsum(voxel[k], voxel[l]) < standard_num) {

					p1 = k;
					p2 = l;

				}
				else continue;

			}
		}

		if (p1 == -1 || p2 == -1) break;

		/*      temp.push_back(merge_points(voxel[p1], voxel[p2]));

			for (int i = 0; i < voxel.size(); i++) {

			  if (i != p1 && i != p2) {

			   temp.push_back(voxel[i]);

			  }

			}
		*/

		for (int i = 0; i < voxel.size(); i++) {

			if (i != p2) {

				temp.push_back(voxel[i]);

			}

		}


		voxel = temp;

		temp.clear();
		p1 = -1;
		p2 = -1;

	}

}

void merge_CUBE(
	vector<vector<PointXYZRGB>>& cube, int standard_num)
{
	for (int i = 0; i < cube.size(); i++) {

		merge_VOXEL(cube[i], standard_num);

	}

}

void view_PC(
	PointCloud<PointXYZRGB>::Ptr pointcloud1,
	PointCloud<PointXYZRGB>::Ptr pointcloud2)
{
	int v1 = 0;

	PCLVisualizer viewer("PC viewer demo");

	viewer.setSize(1280, 1000);

	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);

	viewer.addCoordinateSystem(5.0);



	PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(pointcloud1, 0, 0, 255);

	PointCloudColorHandlerCustom<pcl::PointXYZRGB> green(pointcloud2, 0, 255, 0);

	viewer.addPointCloud(pointcloud2, green, "original", v1);

	viewer.addPointCloud(pointcloud1, red, "merged", v1);

	while (!viewer.wasStopped()) viewer.spinOnce();
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
	double numerator, denominator, dist;



	double M[3][3];

	for (int i = 0; i < 3; i++)

		for (int j = 0; j < 3; j++)

			M[i][j] = m_CalibParams[camera].m_ProjMatrix(i, j);

	for (int i = 0; i < 3; i++)

		denominator = denominator + (M[2][i] * M[2][i]);

	denominator = sqrt(denominator);



	numerator = det(M);

	if (numerator < 0) numerator = -1;

	else if (numerator == 0) numerator = 0;

	else numerator = 1;

	numerator = numerator * w;



	if (denominator == 0) cerr << "error" << endl;

	else dist = (numerator / denominator);



	return dist;

}


void projection(
	PointCloud<PointXYZRGB>::Ptr pointcloud,
	int camera,
	Mat& img,
	Mat& depthimg)
{
	PointCloud<PointXYZRGB>::iterator cloudit;

	PointCloud<PointXYZRGB>::iterator cloudend;

	double X;
	double Y;
	double Z;
	int* u;
	int* v;

	int _u;
	int _v;

	double dist;
	double w;


	for (cloudit = pointcloud->points.begin(), cloudend = pointcloud->points.end(); cloudit < cloudend; cloudit++) {

		X = cloudit->x;
		Y = cloudit->y;
		Z = cloudit->z;

		Z = -Z;

		w = projection_XYZ_2_UV(
			m_CalibParams[camera].m_ProjMatrix,
			X,
			Y,
			Z,
			u,
			v);

		dist = find_point_dist(w, camera);

		if ((*u < 0) || (*v < 0) || (*u > _width - 1) || (*v > _height - 1)) continue;

		_u = (int)*u;

		_v = (int)*v;

		if (depthimg.at<double>(_v, _u) == -1)

			depthimg.at<double>(_v, _u) = dist;


		else
		{
			if (dist < depthimg.at<double>(_v, _u))
				depthimg.at<double>(_v, _u) = dist;

			else continue;

		}


		img.at<Vec3b>(_v, _u)[0] = cloudit->b;

		img.at<Vec3b>(_v, _u)[1] = cloudit->g;

		img.at<Vec3b>(_v, _u)[2] = cloudit->r;
	}

}

void make_proj_img_vec(
	PointCloud<PointXYZRGB>::Ptr pointcloud,
	vector<Mat>& proj_img_vec,
	vector<Mat>& depth_value_img_vec)
{
	for (int i = 0; i < total_num_cameras; i++)
	{
		Mat color_img(_height, _width, CV_8UC3, Scalar(0));

		Mat depth_value_img(_height, _width, CV_64F, -1);

		projection(pointcloud, i, color_img, depth_value_img);

		proj_img_vec[i] = color_img;

		depth_value_img_vec[i] = depth_value_img;
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

	for (int rownum = 0; rownum < _height; rownum++)
	{
		for (int colnum = 0; colnum < _width; colnum++)
		{
			double center_pix_value;

			center_pix_value = depthimg.at<double>(rownum, colnum);

			if (center_pix_value == -1)
			{
				hole_image.at<float>(rownum, colnum) = 0;

				continue;
			}



			int different_pixel = 0;

			int out_of_range_pixel = 0;

			for (int h = rownum - window_size; h <= rownum + window_size; h++)

				for (int w = colnum - window_size; w <= colnum + window_size; w++)
				{
					if (h < 0 || w < 0 || h >= _height || w >= _width)
					{
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
	cout << "end" << endl;
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
		cout << "cam" << i << " = " << psnr << endl;
	}
}



int main()
{
	int scaling = 1;

	set_parameters();

	get_num_camera_N_frame(total_num_cameras, total_num_frames);

	load_matrix_data();

	compute_projection_matrices();

	set_color_data();


	vector<vector<string>> color_names(total_num_cameras, vector<string>(total_num_frames));

	vector<vector<string>> depth_names(total_num_cameras, vector<string>(total_num_frames));

	load_file_name(color_names, depth_names);



	Mat blank(_height, _width, CV_8UC3, Scalar::all(0));

	Mat depth_value_img(_height, _width, CV_64F, -1);



	vector<Mat> color_imgs(total_num_cameras, blank);

	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds(total_num_cameras);

	PointCloud<PointXYZRGB>::Ptr registered_PC(new PointCloud<PointXYZRGB>);


	int voxel_div_num = 512;


	vector<char> camera_order = { 4,3,5,2,6,1,7,0 };

	PointCloud<PointXYZRGB>::Ptr merged_PC(new PointCloud<PointXYZRGB>);



	for (int frame = 0; frame < total_num_frames; frame++)
	{
		cout << "\n----------------------------<frame " << frame << ">---------------------------\n" << endl;

		pointclouds = get_PC_of_every_camera(frame, color_names, depth_names, color_imgs);

		registered_PC = make_registered_PC(pointclouds);

		map<int, list<PointXYZRGB>> cube_map;


		cube_map = make_3dGrid_map(registered_PC, pointclouds, camera_order, voxel_div_num);


		for (auto voxelit = cube_map.begin(); voxelit != cube_map.end(); voxelit++)
			for (list<PointXYZRGB>::iterator pointit = (voxelit->second).begin(); pointit != (voxelit->second).end(); pointit++)
				merged_PC->points.push_back(*pointit);

		cout << endl << "merged points num : " << merged_PC->points.size() << endl;

		view_PC(merged_PC);

		vector<Mat> projection_imgs(total_num_cameras, blank);

		vector<Mat> filled_imgs(total_num_cameras, blank);

		vector<Mat> depth_value_imgs(total_num_cameras, depth_value_img);

		make_proj_img_vec(merged_PC, projection_imgs, depth_value_imgs);

		hole_filling(depth_value_imgs, projection_imgs, filled_imgs);

		printPSNR(color_imgs, projection_imgs);

		cout << "=============================" << endl;

		printPSNR(color_imgs, filled_imgs);

		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
		{
			imshow("original", color_imgs[cam_num]);

			imshow("projection", projection_imgs[cam_num]);

			imshow("hole filling", filled_imgs[cam_num]);

			string image_name = "img_data/mvc_co_hf_frame" + to_string(frame) + "_cam" + to_string(cam_num) + ".png";

			//imwrite(image_name, filled_imgs[cam_num]);

			waitKey(0);
		}

		//printPSNR(color_imgs, filled_imgs);

		color_imgs.clear();

		pointclouds.clear();

		registered_PC.reset(new PointCloud<PointXYZRGB>);

		merged_PC.reset(new PointCloud<PointXYZRGB>);
	}

	system("pause");

	return 0;
}
#endif