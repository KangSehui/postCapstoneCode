#pragma once

#include "global.h"
#include "common_dulppo.h"

extern int _width, _height, total_num_cameras, total_num_frames;
extern double MinZ, MaxZ;
extern vector<Vector2d> tech_minmaxZ;
extern string path;
extern vector<CalibStruct> m_CalibParams;

void set_parameters(int mode);
void get_num_camera_N_frame(int& total_num_cameras, int& total_num_frames);
void load_matrix_data();
void compute_projection_matrices();
void load_file_name(vector<vector<string>>& color_names, vector<vector<string>>& depth_names);
void load_file_name(vector<string>& color_names_, vector<string>& depth_names_);
