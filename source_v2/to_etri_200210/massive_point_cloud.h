#pragma once

#include "global.h"
#include "common.h"

extern int _width, _height, total_num_cameras, total_num_frames;
extern double MinZ, MaxZ;
extern string path;
extern vector<CalibStruct> m_CalibParams;

PointCloud<PointXYZRGBL>::Ptr make_PC_MPC(int camera, vector<Mat> color_imgs, vector<Mat> depth_imgs);
vector<PointCloud<PointXYZRGBL>::Ptr> get_PC_of_every_camera_MPC(int frame, vector<vector<string>> color_names, vector<vector<string>> depth_names, vector<Mat> &color_imgs);
PointCloud<PointXYZRGBL>::Ptr make_registered_PC_MPC(vector<PointCloud<PointXYZRGBL>::Ptr> pointclouds);
void view_PC_MPC(PointCloud<PointXYZRGBL>::Ptr pointcloud);
void projection_MPC(PointCloud<PointXYZRGBL>::Ptr pointcloud, int camera, Mat &img, Mat &depthimg);
Mat make_filled_image(Mat depthimg, Mat colorimg, Mat hole_image, int window_size);