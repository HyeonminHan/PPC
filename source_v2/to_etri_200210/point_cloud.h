#pragma once
#include "global.h"
#include "common.h"

void make_proj_img_vec(PointCloud<PointXYZRGB>::Ptr pointcloud, vector<Mat> &proj_img_vec, vector<Mat> &depth_value_img_vec);
map<int, list<PointXYZRGB>> make_3dGrid_map(PointCloud<PointXYZRGB>::Ptr registered_PC, vector<PointCloud<PointXYZRGB>::Ptr> pointclouds, vector<char> camera_order, int voxel_div_num);
Mat find_hole(Mat depthimg, int window_size);
void hole_filling(vector<Mat> depthimgs, vector<Mat> colorimgs, vector<Mat> &filled_imgs);
Mat make_filled_image(Mat depthimg, Mat colorimg, Mat hole_image, int window_size);