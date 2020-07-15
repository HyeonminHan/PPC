#pragma once

#include "global.h"

vector<int> make_camOrder(int refView);
double depth_level_2_Z(unsigned char d);
double depth_level_2_Z_s(unsigned short d);
double depth_level_2_Z_s(unsigned short d, int camera);
double depth_level_2_Z_s_direct(unsigned short d);
double dequantization(unsigned short norm, double Min, double Max);
unsigned short quantization(double denorm, double Min, double Max);
void projection_UVZ_2_XY_PC(Matrix4d projMatrix, double u, double v, double z, double *x, double *y);
double MVG(Matrix3d K, Matrix3d R, Matrix3Xd t, int x, int y, double Z, double *X, double *Y);
bool confirm_point(int camera, PointXYZRGB p, vector<Mat> color_imgs);
//Mat cvt_yuv2bgr(string name, int frame, int type);
Mat cvt_yuv2bgr(
	string name,
	int frame,
	int type,
	bool is_yuv = true);
PointCloud<PointXYZRGB>::Ptr make_PC(int camera, Mat color_img, Mat depth_img);
vector<PointCloud<PointXYZRGB>::Ptr> get_PC_of_every_camera(
	int frame, 
	vector<vector<string>> color_names, 
	vector<vector<string>> depth_names, 
	vector<Mat> &color_imgs, 
	vector<Mat> &depth_imgs);
vector<PointCloud<PointXYZRGB>::Ptr> get_PC_of_every_camera(
	int frame,
	vector<string> color_names_,
	vector<string> depth_names_,
	vector<Mat> &color_imgs,
	vector<Mat> &depth_imgs);	
void get_color_and_depth_imgs(
	int frame,
	vector<int> camera_order,
	vector<vector<string>> color_names,
	vector<vector<string>> depth_names,
	vector<Mat> &color_imgs,
	vector<Mat> &depth_imgs);
void get_color_and_depth_imgs(
	int frame,
	vector<string> color_names_,
	vector<string> depth_names_,
	vector<Mat> &color_imgs,
	vector<Mat> &depth_imgs);
PointCloud<PointXYZRGB>::Ptr make_registered_PC(vector<PointCloud<PointXYZRGB>::Ptr> pointclouds);
void find_min_max(PointCloud<PointXYZRGB>::Ptr source_PC, vector<float> &min, vector<float> &max);
void find_min_max(
	vector<PointCloud<PointXYZRGB>::Ptr> vec_PC,
	vector<float>& min,
	vector<float>& max);
void find_min_max(
	vector<PPC*> source_PC,
	vector<float> &min,
	vector<float> &max);
void view_PC(PointCloud<PointXYZRGB>::Ptr pointcloud);
void view_PC_yuvTorgb(PointCloud<PointXYZRGB>::Ptr pointcloud);
void view_PC(PointCloud<PointXYZRGB>::Ptr pointcloud, int cam_idx);
void view_PC(PointCloud<PointXYZRGB>::Ptr pointcloud1, PointCloud<PointXYZRGB>::Ptr pointcloud2);
void projection(PointCloud<PointXYZRGB>::Ptr pointcloud, int camera, Mat &img, Mat &depthimg, Mat& is_hole_img);
void projection_bypoint(PointXYZRGB p, int camera, Mat& img, Mat& dist_img, Mat& is_hole_img);
double det(double mat[3][3]);
void printPSNRWithBlackPixel_RGB(Mat orig_img, Mat proj_img);
void printPSNRWithBlackPixel_RGB(vector<Mat> orig_imgs, vector<Mat> proj_imgs, vector<Mat>is_hole_filled_imgs, vector<float>& psnrs_b,vector<float>& psnrs_g,vector<float>& psnrs_r, vector<int>& num_holes);
void printPSNRWithBlackPixel(vector<Mat> orig_imgs, vector<Mat> proj_imgs, vector<float> &psnrs_y, vector<float>& psnrs_u, vector<float>& psnrs_v);
void printPSNRWithoutBlackPixel_RGB(Mat orig_img, Mat proj_img);
void printPSNRWithoutBlackPixel_RGB(vector<Mat> orig_imgs, vector<Mat> proj_imgs, vector<Mat>is_hole_proj_imgs, vector<float>& psnrs_b,vector<float>& psnrs_g,vector<float>& psnrs_r, vector<int>& num_holes);
void printPSNRWithoutBlackPixel(vector<Mat> orig_imgs, vector<Mat> proj_imgs, vector<float>& psnrs_y, vector<float>& psnrs_u, vector<float>& psnrs_v, vector<int> &num_holes);
void printPSNRWithBlackPixel(
	vector<Mat> orig_imgs,
	vector<Mat> proj_imgs,
	vector<float> &psnrs_y,
	vector<float>& psnrs_u,
	vector<float>& psnrs_v);
void printPSNRWithoutBlackPixel(
	vector<Mat> orig_imgs,
	vector<Mat> proj_imgs,
	vector<float>& psnrs,
	vector<int>& num_holes);
void printPSNRWithBlackPixel_2(
	vector<Mat> orig_imgs,
	vector<Mat> proj_imgs,
	vector<float>& psnrs);
void printPSNRWithoutBlackPixel_2(
	vector<Mat> orig_imgs,
	vector<Mat> proj_imgs,
	vector<float>& psnrs,
	vector<int> &num_holes);
void back_projection(PointCloud<PointXYZRGB>::Ptr pointcloud, int camera, Mat &img, Mat& is_hole_img, int nNeighbor);
double projection_XYZ_2_UV(Matrix4d projMatrix, double x, double y, double z, int& u, int& v);
double find_point_dist(double w, int camera);


PointCloud<PointXYZRGB>::Ptr filtering(PointCloud<PointXYZRGB>::Ptr original_PC);
void RGB_dev(vector<PPC> PPC, vector<vector<float>> &dev_pointnum_percent, vector<float> &point_num_per_color);
void HSV_dev(vector<PPC*> PPC, vector<vector<float>>& dev_pointnum_percent, vector<float>& point_num_per_color);
void YUV_dev(vector<PPC*> PPC, vector<vector<float>>& dev_pointnum_percent, vector<float>& point_num_per_color);
void YUV_dev2(vector<PPC*> PPC, vector<vector<float>>& dev_pointnum, vector<int>& point_num_per_color, vector<int>& full_color_dev);
void YUV_dev3_about_MaxValue(vector<PPC*> PPC, vector<float>& point_num_per_color);
void printPSNRWithoutBlackPixel(vector<Mat> orig_imgs, vector<Mat> proj_imgs, vector<float>& psnrs, vector<int> &num_holes);
void printPSNRWithBlackPixel(vector<Mat> orig_imgs, vector<Mat> proj_imgs, vector<float> &psnrs);
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