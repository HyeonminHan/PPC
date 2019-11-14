#pragma once 

#include "global.h"
#include "common.h"

extern int _width, _height, total_num_cameras, total_num_frames;
extern double MinZ, MaxZ;
extern string path;
extern vector<CalibStruct> m_CalibParams;
extern vector<float> geo_min, geo_max;


map<unsigned long long, PPC> make_voxelized_Plen_PC2(vector<PPC> PPC_vec, vector<PointCloud<PointXYZRGB>::Ptr> pointclouds, int voxel_div_num);
vector<PPC> make_sequenced_Plen_PC(vector<PointCloud<PointXYZRGB>::Ptr> pointclouds, vector<Mat> color_imgs, vector<Mat> depth_imgs);
vector<PPC> make_voxelized_Plen_PC(PointCloud<PointXYZRGB>::Ptr registered_PC, vector<PointCloud<PointXYZRGB>::Ptr> pointclouds, int voxel_div_num);
vector<PPC> make_voxelized_Plen_PC(vector<PPC> PPC_vec, vector<PointCloud<PointXYZRGB>::Ptr> pointclouds, int voxel_div_num);
void make_proj_img_vec_ppc(vector<PPC> PPC, vector<Mat> &proj_img_vec, vector<PointCloud<PointXYZRGB>::Ptr> &pointclouds, int nNeighbors);
void projection_PPC_with_hole_filling(vector<PPC> Plen_PC, vector<Mat> &projection_imgs, vector<Mat> &filled_imgs, vector<PointCloud<PointXYZRGB>::Ptr> &pointclouds_, int nNeighbor, int window_size);
void hole_filling_PPC(vector<Mat> proj_imgs, vector<Mat> &filled_imgs, int window_size);
Mat find_hole_PPC(Mat projection_img);
Mat make_filled_image_PPC(Mat colorimg, Mat hole_image, int window_size);
void save_ppc(vector<PPC> ppc);
vector<PPC> load_ppc(void);
void make_proj_img_vec_by_seq(vector<PPC> Plen_PC, vector<Mat> &proj_img_vec, vector<Mat> &depth_value_img_vec);
vector<PPC> make_frustum_Plen_PC(vector<PPC> PPC_vec, vector<PointCloud<PointXYZRGB>::Ptr> pointclouds, int voxel_div_num);
vector<PPC> make_frustum_Plen_PC(PointCloud<PointXYZRGB>::Ptr registered_PC, vector<PointCloud<PointXYZRGB>::Ptr> pointclouds, int voxel_div_num);
void make_frustum(PointCloud<PointXYZRGB>::Ptr source_PC, Point3f& min, Point3f& max, Point3f& frustum_maxXY_maxZ, Point3f& frustum_minxy_maxZ);
vector<PPC_S> make_normalized_Plen_PC(vector<PPC> Plen_PC);
void projection_PPC_with_hole_filling(vector<PPC_S> Plen_PC_S, vector<Mat>& projection_imgs, vector<Mat>& filled_imgs, vector<PointCloud<PointXYZRGB>::Ptr>& pointclouds_, int nNeighbor, int window_size);
void make_proj_img_vec_ppc(vector<PPC_S> PPC_S, vector<Mat>& proj_img_vec, vector<PointCloud<PointXYZRGB>::Ptr>& pointclouds, int nNeighbor);