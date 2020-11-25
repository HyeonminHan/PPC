#pragma once 

#include "global.h"
#include "common.h"

extern int _width, _height, total_num_cameras, total_num_frames;
extern double MinZ, MaxZ, scaleZ;
extern string path;
extern vector<CalibStruct> m_CalibParams;
extern double version;
extern vector<int> camera_order;
extern vector<PPC_v1> ppc_vec;

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

void extract_largeNunit_CubeSize(
	vector<float>& min,
	vector<float>& max,
	int voxel_div_num,
	vector<float>& Cube_size,
	vector<float>& cube_size);

set<unsigned long long> find_valid_cube_indices(
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds, 
	int voxel_div_num, 
	vector<float> min,
	vector<float> Cube_size, 
	vector<float> cube_size);

vector<PPC*> make_modified_Batch_Plen_PC2(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	int voxel_div_num,
	vector<float>& Cube_size,
	vector<float>& cube_size);

void find_min_max_3D_space(
	vector<PointCloud<PointXYZRGB>::Ptr>& pointclouds,
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	vector<float>& min,
	vector<float>& max);

void find_valid_voxels(
	vector<PointCloud<PointXYZRGB>::Ptr>& pointclouds,
	vector<float> min,
	vector<float> max,
	int voxel_div_num,
	vector<float>& space_size,
	vector<float>& voxel_size,
	set<unsigned long long>& valid_cube_indices);

void make_PPC_modified_batch(
	int iteration,
	int max_ppc_size,
	vector<float> min,
	int voxel_div_num,
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	vector<float>& space_size,
	vector<float>& voxel_size,
	set<unsigned long long>& valid_cube_indices,
	bool& end_ppc_generation,
	int& cur_ppc_size);

vector<PointCloud<PointXYZRGB>::Ptr> make_all_PC(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs);

void make_proj_img_vec_ppc(
	vector<PPC*> PPC,
	vector<Mat>& proj_img_vec,
	vector<Mat>& is_hole_proj_imgs,
	vector<PointCloud<PointXYZRGB>::Ptr>& pointclouds,
	int nNeighbor);

void make_proj_img_vec_ppc2(
	vector<PPC*> PPC,
	vector<Mat>& proj_img_vec,
	vector<Mat>& is_hole_proj_imgs,
	int nNeighbor);

void make_proj_img_vec_ppc2_per_viewpoint(
	vector<PPC*> PPC,
	int cam_num,
	Mat& proj_img,
	Mat& is_hole_proj_img,
	int nNeighbor);

void projection_PPC_with_hole_filling(vector<PPC*> Plen_PC, 
	vector<Mat> &projection_imgs, 
	vector<Mat> &filled_imgs, 
	vector<Mat>& is_hole_proj_imgs,
	vector<Mat>& is_hole_filled_imgs, 
	vector<PointCloud<PointXYZRGB>::Ptr> &pointclouds_,
	int nNeighbor, 
	int window_size);

void projection_PPC_with_hole_filling_per_viewpoint(
	vector<PPC*> Plen_PC,
	int cam_num,
	Mat& projection_img,
	Mat& filled_img,
	Mat& is_hole_proj_img,
	Mat& is_hole_filled_img,
	int nNeighbor,
	int window_size);

void perform_projection(
	int cam_num,
	int cur_ppc_size,
	Mat& proj_img,
	Mat& is_hole_proj_img,
	Mat& depth_value_img);

void hole_filling_PPC(vector<Mat> proj_imgs, 
	vector<Mat> &filled_imgs, 
	vector<Mat>& is_hole_filled_imgs, 
	int window_size);

Mat find_hole_PPC(Mat projection_img);

void holefilling_per_viewpoint(Mat colorimg, 
	Mat& filled_image, 
	Mat& hole_image,
	int window_size);

void save_ppc(vector<PPC*> ppc, 
	string filename);

void save_ppc_v1(int total_ppc_size, string filename);

vector<PPC*> load_ppc(string filename);

void load_ppc_v1(string filename, int& total_ppc_size);

void calc_YUV_stddev_global(int cur_ppc_size, 
	vector<vector<float>>& dev_pointnum, 
	vector<int>& point_num_per_color, 
	vector<int>& full_color_dev);

void color_imaging(int query_x, int query_y, vector<Mat> color_imgs, vector<Mat> depth_imgs, vector<float> min, vector<float> Cube_size, vector<float> voxel_size, int voxel_div_num, int tile_size, Mat& ref_img);
void color_imaging2(int query_x, int query_y, vector<Mat> color_imgs, vector<Mat> depth_imgs, vector<float> min, vector<float> Cube_size, vector<float> voxel_size, int voxel_div_num, int tile_size,
	Mat& ref_img_spe, Mat& ref_img_lam, vector<float>& psnr_spe, vector<float>& psnr_lam, int& cnt_spe, int& cnt_lam, string data_name);

void write_yuv(int x, int y, Mat colors, Mat occlusion, Mat& ref_img);
void write_yuv2(int x, int y, Mat colors, Mat occlusion, Mat colors_zoom, Mat& ref_img, string name, string data_name);

void make_PPC_modified_batch_DCT(
	int iteration,
	int max_ppc_size,
	vector<float> min,
	int voxel_div_num,
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	vector<float>& space_size,
	vector<float>& voxel_size,
	set<unsigned long long>& valid_cube_indices,
	bool& end_ppc_generation,
	int& cur_ppc_size,
	int dct_y_valid_pixnum=8,
	int dct_uv_valid_pixnum=4,
	int backgroundfilling_mode=0);

void regionFill(
	Mat* io_image,
	Mat* in_mask,
	Mat* in_downsized_image);

void CreateCoarseLayer(
	Mat* in_image,
	Mat* out_mip,
	Mat* in_occupancyMap,
	Mat* out_mipOccupancyMap);

void interpolate_background_w_laplacian(
	Mat* in_image,
	Mat* in_mask,
	Mat* out_image);

void imshow_zoomin(Mat result, int tile_size, string img_name);