#pragma once

#include "global.h"
#include "common.h"

void print_class_PPC_N2PPC(vector<PPC_N> input_PPC_N);
void print_class_PPC2PPC(vector<PPC> input_PPC_A, vector<PPC> input_PPC_B);
vector<PPC_N> Encode(vector<PPC> input_PPC, PointCloud<pcl::Normal>::Ptr cloud_normal, vector<vector<double>>& cos_T_vector);
PPC calc_plen_color(PPC input_PP, pcl::Normal Pt_norm, vector<double>& cos_T);
PPC_N make_err_vector(PPC input_norm_PP, PPC input_PP, vector<double> cos_theta);
double model_inv_function(double input_I, double cos_theta);
double model_function(double input_I, double cos_theta);
vector<PPC> Decode(vector<PPC_N> input_PPC_N, vector<vector<double>> cos_theta);
pcl::PointCloud<pcl::Normal>::Ptr make_Normal(vector<PPC> input_PPC);// input:: PPC  -> ouput:: Normal