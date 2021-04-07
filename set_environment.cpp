#include "set_environment.h"


using namespace std;
using namespace cv;

#define PI 3.14159265

void set_parameters(int mode);
void get_num_camera_N_frame(int &total_num_cameras, int &total_num_frames);
void load_matrix_data();
void compute_projection_matrices();
void load_file_name(vector<vector<string>> &color_names, vector<vector<string>> &depth_names);
void load_file_name(vector<string> &color_names_, vector<string> &depth_names_);
Vector3d rad2deg(Vector3d radian);
Vector3d deg2rad(Vector3d degree);

void set_parameters(int mode)
{
	switch (mode)
	{
	case MSR3DVideo_Ballet:
		total_num_cameras = 8;
		total_num_frames = 100;

		_width = 1024;
		_height = 768;

		MinZ = 42.0;
		MaxZ = 130.0;

		color_bits = 8;
		depth_bits = 8;

		path = "C:\\Users\\한현민\\Desktop\\pcl_build\\MSR3DVideo-Ballet";

		break;

	case Poznan_Fencing:
		total_num_cameras = 10;
		total_num_frames = 250;

		_width = 1920;
		_height = 1080;

		MinZ = 3.5;
		MaxZ = 7.0;

		color_bits = 8;
		depth_bits = 16;

		path = "C:\\Users\\한현민\\Desktop\\pcl_build\\Poznan_Fencing";

		break;

	case Intel_Kermit:
		//total_num_cameras = 13;
		total_num_cameras = 10;
		total_num_frames = 300;

		MinZ = 0.3;
		MaxZ = 1.62;


		_width = 1920;
		_height = 1080;

		color_bits = 10;
		depth_bits = 16;

		path = "C:\\Users\\한현민\\Desktop\\pcl_build\\Intel_Kermit";

		break;

	case Technicolor_Painter:
		total_num_cameras = 16;
		total_num_frames = 372;

		_width = 2048;
		_height = 1088;

		MinZ = 1.773514;
		MaxZ = 5.300389;

		color_bits = 10;
		depth_bits = 16;

		path = "C:\\Users\\한현민\\Desktop\\pcl_build\\Technicolor_Painter";
		break;

	default:
		cerr << "Wrong mode!!!" << endl;
		exit(0);
	}
}

void get_num_camera_N_frame(
	int &total_num_cameras,
	int &total_num_frames)
{
	//   get cameras
	string camera_path = path + "\\*";

	struct _finddata_t camera_fd;

	intptr_t camera_handle = _findfirst(camera_path.c_str(), &camera_fd);

	int cameras = 0;

	while (_findnext(camera_handle, &camera_fd) == 0) cameras++;

	_findclose(camera_handle);

	if (mode == MSR3DVideo_Ballet) total_num_cameras = cameras - 2;
	else total_num_cameras = cameras / 3;

	//   get frames
	if (mode) total_num_cameras = 300;
	else {
		string frame_path = path + "\\cam0\\*.jpg";

		intptr_t frame_handle;

		struct _finddata_t frame_fd;

		frame_handle = _findfirst(frame_path.c_str(), &frame_fd);

		int frames = 0;

		do frames++;
		while (_findnext(frame_handle, &frame_fd) == 0);

		_findclose(frame_handle);

		total_num_frames = frames;
	}
}

void load_matrix_data()
{
	if (mode == MSR3DVideo_Ballet || mode == Poznan_Fencing) {
		string matrix_path = path + "\\*.txt";

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

		while (!openFile.eof() && total_num_cameras != camera_idx)
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

			// skip distortion coefficient
			getline(openFile, buffer);
			getline(openFile, buffer);
			if (mode == Poznan_Fencing) getline(openFile, buffer);

			// get extrinsics
			while (openFile >> col0 >> col1 >> col2 >> col3)
			{
				temp_CalibParams[camera_idx].m_RotMatrix(row_count, 0) = col0;
				temp_CalibParams[camera_idx].m_RotMatrix(row_count, 1) = col1;
				temp_CalibParams[camera_idx].m_RotMatrix(row_count, 2) = col2;
				temp_CalibParams[camera_idx].m_Trans(row_count, 0) = col3;

				if (row_count > 1) {
					row_count = 0;
					break;
				}
				row_count++;
			}

			if (mode) {
				temp_CalibParams[camera_idx].m_Trans = -1 * temp_CalibParams[camera_idx].m_RotMatrix * temp_CalibParams[camera_idx].m_Trans;
			}

			getline(openFile, buffer);
			getline(openFile, buffer);

			camera_idx++;
		}

		m_CalibParams = temp_CalibParams;

		if (mode) {
			int ref = total_num_cameras / 2;
			Matrix3d refR = m_CalibParams[ref].m_RotMatrix;
			Matrix3Xd refT(3, 1);
			refT = m_CalibParams[ref].m_Trans;

			Matrix3Xd refRT(3, 4);
			refRT.col(0) = m_CalibParams[ref].m_RotMatrix.col(0);
			refRT.col(1) = m_CalibParams[ref].m_RotMatrix.col(1);
			refRT.col(2) = m_CalibParams[ref].m_RotMatrix.col(2);
			refRT.col(3) = m_CalibParams[ref].m_Trans.col(0);

			Matrix4d refRT4x4;
			refRT4x4.row(0) = refRT.row(0);
			refRT4x4.row(1) = refRT.row(1);
			refRT4x4.row(2) = refRT.row(2);
			refRT4x4.row(3) << 0, 0, 0, 1;

			for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
			{
				m_CalibParams[cam_num].m_Trans = m_CalibParams[cam_num].m_RotMatrix * (-refR.inverse() * refT) + m_CalibParams[cam_num].m_Trans;
				m_CalibParams[cam_num].m_RotMatrix *= refR.inverse();

				m_CalibParams[cam_num].m_ProjMatrix = compute_projection_matrices(cam_num);
			}
		}

		openFile.close();
		_findclose(matrix_handle);
	}

	else if (mode == Intel_Kermit || mode == Technicolor_Painter) {
		string matrix_path;
		vector<CalibStruct> temp_CalibParams(total_num_cameras);
		if (mode == Intel_Kermit)
			matrix_path = path + "\\IntelKermit.json";

		else if (mode == Technicolor_Painter)
			matrix_path = path + "\\TechnicolorPainter.json";

		char fileName[100];
		strcpy(fileName, matrix_path.c_str());

		vector<Vector3d> R_vec;
		vector<Vector3d> P_vec;
		vector<Vector2d> DR_vec;
		vector<Vector3d> KF_vec;
		vector<Vector3d> KP_vec;
		if (mode == Intel_Kermit)
			get_RT_data_json(fileName, R_vec, P_vec, KF_vec, KP_vec, total_num_cameras);

		else if (mode == Technicolor_Painter)
			get_RT_data_json(fileName, R_vec, P_vec, DR_vec, KF_vec, KP_vec, total_num_cameras);


		for (int camera_idx = 0; camera_idx < total_num_cameras; camera_idx++) {
			temp_CalibParams[camera_idx].m_K(0, 0) = KF_vec[camera_idx][0];
			temp_CalibParams[camera_idx].m_K(1, 1) = KF_vec[camera_idx][1];
			temp_CalibParams[camera_idx].m_K(0, 2) = KP_vec[camera_idx][0];
			temp_CalibParams[camera_idx].m_K(1, 2) = KP_vec[camera_idx][1];
			temp_CalibParams[camera_idx].m_K(2, 2) = 1.0;

			Euler2RotationMat(R_vec[camera_idx], temp_CalibParams[camera_idx].m_RotMatrix);

			Vector3d temp;
			temp << P_vec[camera_idx][1], P_vec[camera_idx][2], P_vec[camera_idx][0];
			P_vec[camera_idx] = temp;

			temp_CalibParams[camera_idx].m_Trans = P_vec[camera_idx];
		}

		m_CalibParams = temp_CalibParams;

		int ref = total_num_cameras / 2;
		Matrix3d refR = m_CalibParams[ref].m_RotMatrix;
		Matrix3Xd refT(3, 1);
		refT = m_CalibParams[ref].m_Trans;

		Matrix3Xd refRT(3, 4);
		refRT.col(0) = m_CalibParams[ref].m_RotMatrix.col(0);
		refRT.col(1) = m_CalibParams[ref].m_RotMatrix.col(1);
		refRT.col(2) = m_CalibParams[ref].m_RotMatrix.col(2);
		refRT.col(3) = m_CalibParams[ref].m_Trans.col(0);

		Matrix4d refRT4x4;
		refRT4x4.row(0) = refRT.row(0);
		refRT4x4.row(1) = refRT.row(1);
		refRT4x4.row(2) = refRT.row(2);
		refRT4x4.row(3) << 0, 0, 0, 1;

		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
		{
			m_CalibParams[cam_num].m_Trans = m_CalibParams[cam_num].m_RotMatrix * (-refR.inverse() * refT) + m_CalibParams[cam_num].m_Trans;
			m_CalibParams[cam_num].m_RotMatrix *= refR.inverse();
			m_CalibParams[cam_num].m_ProjMatrix = compute_projection_matrices(cam_num);
		}
		if (mode == Technicolor_Painter) {
			tech_minmaxZ = DR_vec;
		}
	}

}

void Quaternion2RotationMat(Vector4d& quaternion, Matrix3d& rotationMat)
{
	rotationMat(0, 0) = 1.0 - 2.0 * quaternion(2) * quaternion(2) - 2.0 * quaternion(3) * quaternion(3);
	rotationMat(0, 1) = 2.0 * quaternion(1) * quaternion(2) - 2.0 * quaternion(3) * quaternion(0);
	rotationMat(0, 2) = 2.0 * quaternion(1) * quaternion(3) + 2.0 * quaternion(2) * quaternion(0);
	rotationMat(1, 0) = 2.0 * quaternion(1) * quaternion(2) + 2.0 * quaternion(3) * quaternion(0);

	rotationMat(1, 1) = 1.0 - 2.0 * quaternion(1) * quaternion(1) - 2.0 * quaternion(3) * quaternion(3);
	rotationMat(1, 2) = 2.0 * quaternion(2) * quaternion(3) - 2.0 * quaternion(1) * quaternion(0);
	rotationMat(2, 0) = 2.0 * quaternion(1) * quaternion(3) - 2.0 * quaternion(2) * quaternion(0);
	rotationMat(2, 1) = 2.0 * quaternion(2) * quaternion(3) + 2.0 * quaternion(1) * quaternion(0);
	rotationMat(2, 2) = 1.0 - 2.0 * quaternion(1) * quaternion(1) - 2.0 * quaternion(2) * quaternion(2);

	rotationMat = rotationMat.transpose();
}

void GetRotationMat(Vector3d& euler, Matrix3d& rotationMat)
{
	double sx = sin(euler(0));
	double cx = cos(euler(0));
	double sy = sin(euler(1));
	double cy = cos(euler(1));
	double sz = sin(euler(2));
	double cz = cos(euler(2));

	Matrix3d Rx, Ry, Rz;
	Rx <<
		1, 0, 0,
		0, cx, -1.0*sx,
		0, sx, cx;
	Ry <<
		cy, 0, sy,
		0, 1, 0,
		-1.0*sy, 0, cy;
	Rz <<
		cz, -1.0*sz, 0,
		sz, cz, 0,
		0, 0, 1;
	rotationMat = Rx * Ry * Rz;
}

void Euler2RotationMat(Vector3d& euler, Matrix3d& rotationMat)
{
	euler = deg2rad(euler);

	double sinc = sin(euler(2)); //sinc
	double cosc = cos(euler(2)); //cosc
	double sinb = sin(euler(1)); //
	double cosb = cos(euler(1)); //cosb
	double sina = sin(euler(0)); //
	double cosa = cos(euler(0)); //

	//Euler ZYX
	rotationMat(0, 0) = cosb * cosa;
	rotationMat(0, 1) = sinc * sinb * cosa - cosc * sina;
	rotationMat(0, 2) = cosc * sinb * cosa + sinc * sina;

	rotationMat(1, 0) = cosb * sina;
	rotationMat(1, 1) = sinc * sinb * sina + cosc * cosa;
	rotationMat(1, 2) = cosc * sinb * sina - sinc * cosa;

	rotationMat(2, 0) = -1.0 * sinb;
	rotationMat(2, 1) = sinc * cosb;
	rotationMat(2, 2) = cosc * cosb;
}

// 카메라 RT 저장
void compute_projection_matrices()
{
	cout << "compute projection matrix START:: " << endl;
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
			for (int j = 0; j < 4; j++) {
				m_CalibParams[cam_idx].m_ProjMatrix(i, j) = 0.0;

				for (int k = 0; k < 3; k++)
					m_CalibParams[cam_idx].m_ProjMatrix(i, j) += inMat(i, k) * exMat(k, j);
			}
		m_CalibParams[cam_idx].m_ProjMatrix(3, 0) = 0.0;
		m_CalibParams[cam_idx].m_ProjMatrix(3, 1) = 0.0;
		m_CalibParams[cam_idx].m_ProjMatrix(3, 2) = 0.0;
		m_CalibParams[cam_idx].m_ProjMatrix(3, 3) = 1.0;
	}
	cout << "compute projection matrix ENDDDDD:: " << endl;
}

void load_file_name(
	vector<vector<string>> &color_names,
	vector<vector<string>> &depth_names)
{
	string cam_path = path + "\\cam";

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

void load_file_name(
	vector<string> &color_names_,
	vector<string> &depth_names_)
{
	intptr_t color_handle, depth_handle;

	struct _finddata_t color_fd, depth_fd;

	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
	{
		string color_path, depth_path;
		switch (mode) {
		case Poznan_Fencing:
			color_path = path + "\\*cam" + to_string(cam_num) + "_tex*.yuv";
			if (depth_bits == 8) depth_path = path + "\\*cam" + to_string(cam_num) + "_depth*.yuvcut.yuv";
			else depth_path = path + "\\*cam" + to_string(cam_num) + "_depth*.yuv";
			break;

		case Intel_Kermit:
			color_path = path + "\\v" + to_string(cam_num + 1) + "_tex*.yuv";
			depth_path = path + "\\v" + to_string(cam_num + 1) + "_dep*.yuv";
			break;

		case Technicolor_Painter:
			color_path = path + "\\v" + to_string(cam_num) + "_tex*.yuv";
			depth_path = path + "\\v" + to_string(cam_num) + "_dep*.yuv";
			break;
		}

		color_handle = _findfirst(color_path.c_str(), &color_fd);
		depth_handle = _findfirst(depth_path.c_str(), &depth_fd);

		color_names_[cam_num] = color_fd.name;
		depth_names_[cam_num] = depth_fd.name;

		_findnext(color_handle, &color_fd);
		_findnext(depth_handle, &depth_fd);
	}

	_findclose(color_handle);
	_findclose(depth_handle);
}



