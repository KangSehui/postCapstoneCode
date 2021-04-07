#pragma once

#define BOOST_TYPEOF_EMULATION

#define MSR3DVideo_Ballet 0
#define Poznan_Fencing 1
#define Intel_Kermit 2
#define Technicolor_Painter 3

#include "stdio.h"
#include "stdlib.h"
#include <windows.h>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <math.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <list>
#include <pcl/registration/icp.h>
#include <ctime>
#include <pcl/features/normal_3d.h>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include <rapidjson/filereadstream.h>
#include <string>
#include <cstdio>

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

extern int mode, _width, _height, total_num_cameras, total_num_frames, temp_num_cameras, color_bits, depth_bits;
extern double MinZ, MaxZ;
extern vector<Vector2d> tech_minmaxZ;
extern vector<double> geo_min, geo_max;
extern string path;
extern vector<CalibStruct> m_CalibParams;
extern ofstream writeFile;
extern vector<float> PSNRs;



typedef struct {
	float geometry[3];
	uchar color[16][3];//[num_of_cam][color_index]
	bool occlusion_pattern[16];
} PPC;

typedef struct {
	unsigned short geometry[3];
	uchar color[16][3];//[num_of_cam][color_index]
	bool occlusion_pattern[16];
} PPC_S;

typedef struct {
	float geometry[3];
	short norm_color[3];
	short res[16][3];//[num_of_cam][color_index]
} PPC_N;




