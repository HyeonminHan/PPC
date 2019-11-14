#pragma once

#include "global.h"
#include "common.h"

extern int _width, _height, total_num_cameras, total_num_frames;
extern double MinZ, MaxZ;
extern string path;
extern vector<CalibStruct> m_CalibParams;

void set_parameters(int mode);
void get_num_camera_N_frame(int &total_num_cameras, int &total_num_frames);
void load_matrix_data();
void compute_projection_matrices();
void load_file_name(vector<vector<string>> &color_names, vector<vector<string>> &depth_names);
void load_file_name(vector<string> &color_names_, vector<string> &depth_names_);
void GetRotationMat(Vector3d& euler, Matrix3d& rotationMat);
void Quaternion2RotationMat(Vector4d& quaternion, Matrix3d& rotationMat);
void Euler2RotationMat(Vector3d& euler, Matrix3d& rotationMat);
void get_RT_data_json(const char* file, vector<Vector3d>& Rotation_vec, vector<Vector3d>& Position_vec, vector<Vector3d>& KFocal_vec, vector<Vector3d>& KPrinciple_vec, int total_num_cameras);
