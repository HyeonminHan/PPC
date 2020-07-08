#pragma once 

#include "global.h"
#include "common.h"

extern int _width, _height, total_num_cameras, total_num_frames;
extern double MinZ, MaxZ;
extern string path;
extern vector<CalibStruct> m_CalibParams;
extern double version;
extern int degree;

vector<PPC> make_sequenced_Plen_PC(vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,vector<Mat> color_imgs,vector<Mat> depth_imgs,int d_threshold,int c_threshold);
//vector<PPC> make_sequenced_Plen_PC(PointCloud<PointXYZRGB>::Ptr firstPC, vector<Mat> color_imgs, vector<Mat> depth_imgs, int d_threshold, int c_threshold);
vector<PPC*> make_sequenced_Plen_PC(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs);

void projection_PPC_with_hole_filling(vector<PPC*> Plen_PC, vector<Mat> &projection_imgs, vector<Mat> &filled_imgs, vector<PointCloud<PointXYZRGB>::Ptr> &pointclouds_, int nNeighbor, int window_size);

void hole_filling_PPC(vector<Mat> proj_imgs, vector<Mat> &filled_imgs, int window_size);
Mat find_hole_PPC(Mat projection_img);
Mat make_filled_image_PPC(Mat colorimg, Mat hole_image, int window_size);
void save_ppc(vector<PPC*> ppc, string filename);
vector<PPC*> load_ppc(string filename);
