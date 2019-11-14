#pragma once

#include "global.h"
#include "common.h"


Vec3f get_mean(PPC input_PP, int& num);
vector<uint> get_cnt_vec(vector<PPC> input_PPC);
vector<vector< VectorXd>> KLT_trans(vector<PPC> input_PPC, int cnt, vector<MatrixXd>& cov_mat, int principal_axis);
MatrixXd get_KLT(MatrixXd KLT, int cnt);
vector<vector< VectorXd>> decode_KLT(vector<vector< VectorXd>> Sn, vector<MatrixXd> cov_mat);
vector<vector< VectorXd>> decode_PPC_prime(vector<vector< VectorXd>> PPC_KLT, vector<vector<bool>> PPC_flag);
vector<PPC> PPC_double2uchar(vector<vector<vector< VectorXd>>> ppc_double, vector<vector<PPC>> class_PPC, int start_idx);
vector<PPC> PPC_double2uchar(vector<vector<vector< VectorXd>>> ppc_double, vector<vector<PPC>> class_PPC, vector<int>& En_idx_vec);
vector<double> get_err_norm1(vector<PPC> ppc_, vector<vector<VectorXd>> ppc_prime);
vector<double> get_err_norm2(vector<PPC> ppc_, vector<vector<VectorXd>> ppc_prime);
vector<vector<PPC>> classify_PPC(vector<PPC> input_PPC, vector<uint> cnt_vec);
vector<PPC> get_merged(vector<PPC> input_PPC, vector<vector<bool>>& PPC_flag);

///Encode
vector<vector<vector< VectorXd>>> Encoder_(vector<PPC> input_PPC, vector<vector<MatrixXd>>& cov_mat, vector<vector<vector<bool>>>& ppc_flag_vec, vector<vector<PPC>>& class_PPC, int start_idx, int end_idx);
vector<vector<vector< VectorXd>>> Encoder_v2(vector<PPC> input_PPC, vector<vector<MatrixXd>>& cov_mat, vector<vector<vector<bool>>>& ppc_flag_vec, vector<vector<PPC>>& class_PPC, vector<int>& En_idx_vec, int reductoin_dim);
vector<vector<VectorXd>> Encode(vector<PPC> class_PPC_merged, int class_idx, int klt_axis, vector<MatrixXd>& cov_mat_temp);
///Decoder
vector<PPC> Decoder_(vector<vector<vector<VectorXd>>> ppc_prime_vec, vector<vector<MatrixXd>> cov_mat_vec, vector<vector<vector<bool>>> ppc_flag_vec, vector<vector<PPC>> class_ppc, vector<vector<vector< VectorXd>>>& _PPC_double, int start_idx);
vector<PPC> Decoder_v2(vector<vector<vector<VectorXd>>> ppc_prime_vec, vector<vector<MatrixXd>> cov_mat_vec, vector<vector<vector<bool>>> ppc_flag_vec, vector<vector<PPC>> class_ppc, vector<vector<vector< VectorXd>>>& _PPC_double, vector<int>& En_idx_vec);
vector < vector<VectorXd>> Decode(vector < vector<VectorXd>> ppc_prime, vector<MatrixXd> cov_mat, vector<vector<bool>> ppc_flag);


void print_class(vector<Vec3f> vec_, uint num_cnt);