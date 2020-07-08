#pragma once 

#include "global.h"
#include "common.h"

extern int _width, _height, total_num_cameras, total_num_frames;
extern double MinZ, MaxZk, scaleZ;
extern string path;
extern vector<CalibStruct> m_CalibParams;
extern double version;
extern vector<int> camera_order;

vector<PPC*> make_incremental_Plen_PC(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	int colorspace,
	vector<int> camera_order);

vector<PPC*> make_incremental_Plen_PC(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	int colorspace,
	vector<int> camera_order,
	int voxel_div_num, 
	float& depth_threshold);

vector<PPC*> make_voxelized_Plen_PC(
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	int voxel_div_num);

vector<PPC*> make_voxelized_Plen_PC(
	vector<PPC*> PPC_vec,
	int voxel_div_num,
	vector<float>& Cube_size,
	vector<float>& cube_size);

vector<PPC*> make_formulaic_voxelized_Plen_PC(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	int voxel_div_num);

vector<PPC*> make_formulaic_voxelized_Plen_PC2(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	int voxel_div_num,
	vector<float>& Cube_size,
	vector<float>& cube_size);

vector<PointCloud<PointXYZRGB>::Ptr> make_all_PC(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs);

void make_proj_img_vec_ppc(
	vector<PPC*> PPC,
	vector<Mat>& proj_img_vec,
	vector<Mat>& is_hole_proj_imgs,
	vector<PointCloud<PointXYZRGB>::Ptr>& pointclouds,
	int nNeighbor);

void projection_PPC_with_hole_filling(vector<PPC*> Plen_PC, vector<Mat> &projection_imgs, vector<Mat> &filled_imgs, vector<Mat>& is_hole_proj_imgs,
	vector<Mat>& is_hole_filled_imgs, vector<PointCloud<PointXYZRGB>::Ptr> &pointclouds_,int nNeighbor, int window_size);

void hole_filling_PPC(vector<Mat> proj_imgs, vector<Mat> &filled_imgs, vector<Mat>& is_hole_filled_imgs, int window_size);
Mat find_hole_PPC(Mat projection_img);
Mat make_filled_image_PPC(Mat colorimg, Mat &hole_image, int window_size);
void save_ppc(vector<PPC*> ppc, string filename);
vector<PPC*> load_ppc(string filename);