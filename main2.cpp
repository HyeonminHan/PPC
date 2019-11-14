#if 1
#include "global.h"
#include "set_environment.h"
#include "common.h"
#include "massive_point_cloud.h"
#include "plenoptic_point_cloud.h"
#include "point_cloud.h"
#include "ppc_prime.h"
#include "Codec_KLT.h"
#include "color_modeling.h"
#define camera 10

int mode, _width, _height, total_num_cameras, total_num_frames, color_bits, depth_bits; // color, depth 몇 bit인지
vector<float> geo_min, geo_max;
double MinZ, MaxZ;
string path;
vector<CalibStruct> m_CalibParams;



int main()
{
	mode = 0;

	int sub_block_mode = 0;
	int sub_block_res = 0;
	int ppc_mode = 3;
	int shape_of_voxel = 1;
	int voxel_mode = 2;
	int voxel_div_num = 0;
	int color_model_mode = 0;
	//cout << "mode: ";
	//cin >> mode;

	/*
		MSR3DVideo_Ballet 0
		Poznan_Fencing 1
		Intel_Kermit 2
		Technicolor_Painter 3
	*/

	set_parameters(mode);
	load_matrix_data();
	compute_projection_matrices();

	// 파일 이름 다 저장해놓고 원하는 거 불러오기
	vector<vector<string>> color_names(total_num_cameras, vector<string>(total_num_frames));
	vector<vector<string>> depth_names(total_num_cameras, vector<string>(total_num_frames));
	vector<string> color_names_(total_num_cameras);
	vector<string> depth_names_(total_num_cameras);
	if (!mode) load_file_name(color_names, depth_names);
	else load_file_name(color_names_, depth_names_);

	Mat blank_c, blank_d;
	Mat temp_8(_height, _width, CV_8UC3, Scalar::all(0));
	Mat temp_16(_height, _width, CV_16UC3, Scalar::all(0));

	switch (mode) {
	case 0:
		blank_c = temp_8;
		blank_d = temp_8;
		break;

	case 1:
	case 2:
	case 3:
		blank_c = temp_8;
		blank_d = temp_16;
		break;
	}

	vector<Mat> color_imgs(total_num_cameras, blank_c);
	vector<Mat> depth_imgs(total_num_cameras, blank_d);
	Mat depth_value_img(_height, _width, CV_64F, -1);
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds(total_num_cameras);
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds_(total_num_cameras);
	vector<PointCloud<PointXYZRGB>::Ptr> filtered_pointclouds(total_num_cameras);
	PointCloud<PointXYZRGB>::Ptr registered_PC(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr merged_PC(new PointCloud<PointXYZRGB>);

	int how_many_similar_points[27] = { 0, };

	for (sub_block_mode = 1; sub_block_mode <= 1; sub_block_mode++) {
		for (ppc_mode = 2; ppc_mode <= 2; ppc_mode++) {
			for (shape_of_voxel = 1; shape_of_voxel <= 1; shape_of_voxel++) {
				for (voxel_mode = 2; voxel_mode <= 2; voxel_mode++) {

					//ofstream fout_cs;
					//ofstream fout_os;
					//ofstream fout_data;
					//ofstream fout_dev;

					// 전체 frame의 10%만 돌린다.
					int frame_num = 1; //total_num_frames / 10;
					int cnt = 0;

					if (sub_block_mode == 1) sub_block_res = 8;
					else if (sub_block_mode == 2) sub_block_res = 16;
					else if (sub_block_mode == 3) sub_block_res = 32;

					if (voxel_mode == 1) voxel_div_num = 1024;
					else if (voxel_mode == 2) voxel_div_num = 2048;
					else if (voxel_mode == 3) voxel_div_num = 4096;

					string name_mode;
					if (mode == 0) name_mode = "ballet";
					else if (mode == 1) name_mode = "fencing";
					else if (mode == 2) name_mode = "intel";
					else if (mode == 3) name_mode = "tech";

					string shape, name_ppc;
					if (shape_of_voxel == 1) shape = "frustum";
					else shape = "cube";

					if (ppc_mode == 1) name_ppc = "incre_d";
					else if (ppc_mode == 2) name_ppc = "incre_c";
					else name_ppc = "batch";

					string name_cs = "output\\" + name_mode + "_" + to_string(voxel_div_num) + "_" + shape + "_" + name_ppc + "_sub_" + to_string(sub_block_res) + "_cs.csv";
					string name_os = "output\\" + name_mode + "_" + to_string(voxel_div_num) + "_" + shape + "_" + name_ppc + "_sub_" + to_string(sub_block_res) + "_os.csv";
					string name_data = "output\\" + name_mode + "_" + to_string(voxel_div_num) + "_" + shape + "_" + name_ppc + "_sub_" + to_string(sub_block_res) + "_data.csv";
					string name_dev = "output\\" + name_mode + "_" + to_string(voxel_div_num) + "_" + shape + "_" + name_ppc + "_sub_" + to_string(sub_block_res) + "_dev.csv";

					//fout_cs.open(name_cs);
					//fout_os.open(name_os);
					//fout_data.open(name_data);
					//fout_dev.open(name_dev);
					//
					//fout_data << "frame,PC,PPC,hole num";
					//for (int i = 0; i < total_num_cameras; i++)
					//   fout_data << ",cam" << i;
					//
					//fout_data << ",PSNR p";
					//for (int i = 0; i < total_num_cameras; i++)
					//   fout_data << ",cam" << i;
					//
					//fout_data << ",PSNR h";
					//for (int i = 0; i < total_num_cameras; i++)
					//   fout_data << ",cam" << i;
					//
					//fout_data << "\n";
					//
					//
					//fout_dev << "frame,";
					//for (int i = 0; i < total_num_cameras; i++)
					//   fout_dev << i + 1 << "개,0~5,5~10,10~15,15~20,20~,";
					//
					//fout_dev << "\n";


					// 100 프레임짜리 영상
					for (int frame = 0; frame < 3; frame++)
					{
						if (frame == frame_num) break;
						//fout_data << frame << ",";
						//fout_dev << frame << ",";

						clock_t start = clock();
						cout << "\n----------------------------<frame " << frame << ">---------------------------\n" << endl;

						if (!mode)
							pointclouds = get_PC_of_every_camera(frame, color_names, depth_names, color_imgs, depth_imgs);
						else
							pointclouds = get_PC_of_every_camera(frame, color_names_, depth_names_, color_imgs, depth_imgs);

						cout << "Registeration ENDDDD : :" << endl;
						registered_PC = make_registered_PC(pointclouds);

						//view_PC(registered_PC);

						//fout_data << registered_PC->points.size() << ",";

						cout << "Before refinement, Plenoptic Point Cloud size:\t" << registered_PC->points.size() << endl;

						vector<PPC> Plen_PC;
						vector<PPC_S> Plen_PC_S;
						map<unsigned long long, PPC> Plen_PC_map;
						int ppc_number = 0;


						// ppc mode 1 : HHHP(only depth) , mode 2 : HHHP(depth+color) , mode3 : voxelized
						// shape of voxel mode 1 : frustum , mode 2 : cube
						// voxel_mode 1 : 1024 , 2 : 2048 , 3 : 4096
						// color_model mode 0 : nothing, 1 : linear, 2 : 2 degree , 3 : hybrid , 4 : KLT
						// excel의 2열 : PPC mode / 4열 : shape of voxel / 5열 : 해상도 / 6열 : 총 점 개수 / 7,8열 각각 점수, 분포, / 9,10,11 PSNR ::::: color model mode 0
						// for문 돌아가야 하는 경우의 수 : color mode 0 따로 한번(전체 뽑을 수 있는거 다 뽑기), 123 따로 한번(PSNR with hole)

						if (voxel_mode == 1) voxel_div_num = 1024;
						else if (voxel_mode == 2) voxel_div_num = 2048;
						else if (voxel_mode == 3) voxel_div_num = 4096;

						//if (ppc_mode == 2 && shape_of_voxel == 2 && voxel_mode == 3) continue;

						for (color_model_mode = 0; color_model_mode <= 0; color_model_mode++)
						{
							cout << "ppcmode : " << ppc_mode << " shape of voxel : " << shape_of_voxel << " voxel_mode : " << voxel_mode << " color model mode :" << color_model_mode << endl;
							if (ppc_mode == 1)
							{
								Plen_PC = make_sequenced_Plen_PC(pointclouds, color_imgs, depth_imgs);

								if (shape_of_voxel == 1)
								{
									Plen_PC = make_frustum_Plen_PC(Plen_PC, pointclouds, voxel_div_num);

									if (voxel_mode == 1) ppc_number = 3;
									else if (voxel_mode == 3) ppc_number = 4;
								}
								else if (shape_of_voxel == 2)
								{
									Plen_PC = make_voxelized_Plen_PC(Plen_PC, pointclouds, voxel_div_num);

									if (voxel_mode == 1) ppc_number = 0;
									else if (voxel_mode == 2) ppc_number = 1;
									else if (voxel_mode == 3) ppc_number = 2;
								}
							}

							else if (ppc_mode == 2)
							{
								cout << "color threshold start" << endl;
								Plen_PC = make_sequenced_Plen_PC(pointclouds, color_imgs, depth_imgs);
								cout << "after incremental ppc size : " << Plen_PC.size() << endl;
								cout << "color threshold end" << endl;
								cout << "sequence plenpc size :: " << Plen_PC.size() << endl;

								Plen_PC_S = make_normalized_Plen_PC(Plen_PC);

								//cout << "ffffffffffffffffff\t" << Plen_PC[12345].geometry[0] << "\t" << Plen_PC[12345].geometry[1] << "\t" << Plen_PC[12345].geometry[2] << endl;

								// voxelization
								//if (shape_of_voxel == 1)
								//{
								//   //Plen_PC = make_frustum_Plen_PC(Plen_PC, pointclouds, voxel_div_num);
								//   if (voxel_mode == 1) ppc_number = 8;
								//   else if (voxel_mode == 3) ppc_number = 9;
								//}
								//else if (shape_of_voxel == 2)
								//{
								//   //Plen_PC = make_voxelized_Plen_PC(Plen_PC, pointclouds, voxel_div_num);
								//   if (voxel_mode == 1) ppc_number = 5;
								//   else if (voxel_mode == 2) ppc_number = 6;
								//   else if (voxel_mode == 3) ppc_number = 7;
								//}


							}

							else if (ppc_mode == 3)
							{
								if (shape_of_voxel == 1)
								{
									Plen_PC = make_frustum_Plen_PC(registered_PC, pointclouds, voxel_div_num);
									if (voxel_mode == 1) ppc_number = 13;
									else if (voxel_mode == 3) ppc_number = 14;
								}
								else if (shape_of_voxel == 2)
								{
									Plen_PC = make_voxelized_Plen_PC(registered_PC, pointclouds, voxel_div_num);
									//Plen_PC_map = make_voxelized_Plen_PC2(Plen_PC, pointclouds, voxel_div_num);
									if (voxel_mode == 1) ppc_number = 10;
									else if (voxel_mode == 2) ppc_number = 11;
									else if (voxel_mode == 3) ppc_number = 12;
								}
							}
							vector<PPC> _PPC;

							vector<PPC_S> _PPC_S;

							//if (color_model_mode == 0) _PPC = Plen_PC;
							if (color_model_mode == 0) _PPC_S = Plen_PC_S;
							else
							{
								if (color_model_mode == 1) // linear
								{
									vector<PPC3> Plen_PC_color_modeling = encodeColorModelingPPC2(Plen_PC);
									_PPC = decodeColorModelingPPC2(Plen_PC_color_modeling);
								}
								else if (color_model_mode == 2) // 2nd order
								{
									vector<PPC2> Plen_PC_color_modeling = encodeColorModelingPPC(Plen_PC);
									_PPC = decodeColorModelingPPC(Plen_PC_color_modeling);
								}
								else if (color_model_mode == 3) // hybrid
								{
									vector<PPC4> Plen_PC_color_modeling = encodeColorModelingPPC3(Plen_PC);
									_PPC = decodeColorModelingPPC3(Plen_PC_color_modeling);
								}
								else if (color_model_mode == 4)
								{
									vector<vector<MatrixXd>> cov_mat;
									vector<vector<vector< VectorXd>>> PPC_prime;
									vector<vector<vector<bool>>> class_PPC_merged_flag;
									vector<vector<PPC>> class_PPC;

									vector<int> En_idx_vec{ 3,4,5,6,7,8,9,10 };
									cout << "Before En_idx_vec(" << En_idx_vec.size() << ") -> ";
									for (int i = 0; i < En_idx_vec.size(); i++)
									{
										cout << En_idx_vec[i] << " ";
									}cout << endl;

									/*Encoder*/
									PPC_prime = Encoder_v2(Plen_PC, cov_mat, class_PPC_merged_flag, class_PPC, En_idx_vec, 2);
									cout << "After En_idx_vec(" << En_idx_vec.size() << ") -> ";
									for (int i = 0; i < En_idx_vec.size(); i++)
									{
										cout << En_idx_vec[i] << " ";
									}
									cout << endl;

									vector<vector<vector< VectorXd>>> _PPC_double;
									/*Decoder*/
									_PPC = Decoder_v2(PPC_prime, cov_mat, class_PPC_merged_flag, class_PPC, _PPC_double, En_idx_vec);
								}
								else if (color_model_mode == 5)
								{
									vector<PPC5> Plen_PC_color_modeling = encodeColorModelingPPC4(Plen_PC);
									_PPC = decodeColorModelingPPC4(Plen_PC_color_modeling);
								}
							}

							clock_t end1 = clock();
							cout << "====================================================================ppc time : "
								<< float(end1 - start) / CLOCKS_PER_SEC << "================================================" << endl;

							//fout_data << Plen_PC.size() << ",";
							cout << "After refinement, Plenoptic Point Cloud size:\t" << Plen_PC.size() << endl;
							/*

							// similarity 계산하는 방법
							vector<float> min(3), max(3);
							double p_r = 0;
							double p_g = 0;
							double p_b = 0;
							find_min_max(registered_PC, min, max);

							float x_size = (max[0] - min[0]);
							float y_size = (max[1] - min[1]);
							float z_size = (max[2] - min[2]);

							size_t x_voxel_num = voxel_div_num;
							size_t y_voxel_num = voxel_div_num;
							size_t z_voxel_num = voxel_div_num;

							size_t x_sub_voxel_num = voxel_div_num / sub_block_res; // 4*4*4짜리 subblock 만든것
							size_t y_sub_voxel_num = voxel_div_num / sub_block_res;
							size_t z_sub_voxel_num = voxel_div_num / sub_block_res;

							int x_sub_block_index = 0, y_sub_block_index = 0, z_sub_block_index = 0;

							map<size_t, vector<PPC> > sub_blocked_cube;

							for (size_t i = 0; i < Plen_PC.size(); i++) {
							   size_t  x_voxel_index = (int)floor((Plen_PC[i].geometry[0] - min[0]) / x_size * ((float)x_voxel_num - 1));
							   size_t  y_voxel_index = (int)floor((Plen_PC[i].geometry[1] - min[1]) / y_size * ((float)y_voxel_num - 1));
							   size_t  z_voxel_index = (int)floor((Plen_PC[i].geometry[2] - min[2]) / z_size * ((float)z_voxel_num - 1));

							   x_sub_block_index = x_voxel_index / sub_block_res;
							   y_sub_block_index = y_voxel_index / sub_block_res;
							   z_sub_block_index = z_voxel_index / sub_block_res;

							   size_t sub_block_index = x_sub_block_index * (y_sub_voxel_num * z_sub_voxel_num) + y_sub_block_index * z_sub_voxel_num + z_voxel_index;

							   sub_blocked_cube[sub_block_index].push_back(Plen_PC[i]);
							}

							for (map<size_t, vector<PPC>>::iterator it = sub_blocked_cube.begin(); it != sub_blocked_cube.end(); it++) {
							   int x_idx = it->first / (y_sub_voxel_num * z_sub_voxel_num);
							   int y_idx = (it->first % (y_sub_voxel_num * z_sub_voxel_num)) / z_sub_voxel_num;
							   int z_idx = (it->first % (y_sub_voxel_num * z_sub_voxel_num)) % z_sub_voxel_num;
							}

							for (map<size_t, vector<PPC>>::iterator it = sub_blocked_cube.begin(); it != sub_blocked_cube.end(); it++) {
							   //cout << "sub block 내의 점 개수 :: " << it->second.size() << endl;
							   if (it->second.size() < 3) continue;

							   // Hamming Distance 구하기
							   int hamming_distance[100] = { 0, };
							   int ham_dist = 0;
							   double occlustion_pattern_value = 0;

							   for (int i = 0; i < it->second.size(); i++) { // 같은 sub block 내에 있는 point의 개수만큼 돌아야함
								  for (int j = 0; j < it->second.size(); j++) {
									 for (int k = 0; k < 8; k++) {
										if (i < j) {
										   if ((it->second[i].occlusion_pattern[k] != it->second[j].occlusion_pattern[k]) == 1) { // XOR 연산
											  ham_dist++;
										   }
										}
									 }
								  }
								  hamming_distance[i] = ham_dist;
							   }

							   // Occlusion Pattern Similarity 구하기
							   int subblock_point_num = it->second.size();
							   int combination_value = 0;

							   vector<double> occlusion_pattern_vec;

							   combination_value = combination(subblock_point_num, 2);
							   occlustion_pattern_value = 1 - double(ham_dist) / double(combination_value * total_num_cameras);
							   fout_os << occlustion_pattern_value << ",";
							   if (cnt % 1000 == 999)
								  fout_os << endl;
							   // Color Similarity 구하기
							   // Sub Block 내의 색상값들의 평균 내기
							   vector<int> color_mean = { 0, };

							   // Covariance 계산
							   // Mean 구하기
							   vector<vector<double>> mean_(3, vector<double>(total_num_cameras));

							   for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
							   {
								  for (int idx = 0; idx < subblock_point_num; idx++)
									 for (int chan_idx = 0; chan_idx < 3; chan_idx++)
										mean_[chan_idx][cam_idx] += it->second[idx].color[cam_idx][chan_idx];

								  for (int chan_idx = 0; chan_idx < 3; chan_idx++)
									 mean_[chan_idx][cam_idx] /= subblock_point_num;

							   }

							   // Covariance 계산
							   const int cnt_size = total_num_cameras;
							   MatrixXd cov_1(cnt_size, cnt_size);
							   MatrixXd cov_2(cnt_size, cnt_size);
							   MatrixXd cov_3(cnt_size, cnt_size);

							   // Variance 계산
							   MatrixXd var_1(cnt_size, cnt_size);
							   MatrixXd var_2(cnt_size, cnt_size);
							   MatrixXd var_3(cnt_size, cnt_size);

							   Vec3f color_;
							   Vec3f var_color_i;
							   Vec3f var_color_j;

							   // i,j는 sub block 내의 점의 개수
							   for (int i = 0; i < cnt_size; i++)
								  for (int j = 0; j < cnt_size; j++)
								  {
									 color_(0) = 0;
									 color_(1) = 0;
									 color_(2) = 0;

									 var_color_i(0) = 0;
									 var_color_i(1) = 0;
									 var_color_i(2) = 0;

									 var_color_j(0) = 0;
									 var_color_j(1) = 0;
									 var_color_j(2) = 0;

									 for (int idx1 = 0; idx1 < subblock_point_num; idx1++)
										for (int idx2 = 0; idx2 < subblock_point_num; idx2++)
										   if (idx1 < idx2)
											  for (int chan_idx = 0; chan_idx < 3; chan_idx++)
												 if ((it->second[idx1].occlusion_pattern[i] == true) && (it->second[idx2].occlusion_pattern[j] == true)) {
													color_(chan_idx) += ((it->second[idx1].color[i][chan_idx] - mean_[chan_idx][i]) * (it->second[idx2].color[j][chan_idx] - mean_[chan_idx][j]));

													var_color_i(chan_idx) += ((it->second[idx1].color[i][chan_idx] - mean_[chan_idx][i]) * (it->second[idx1].color[i][chan_idx] - mean_[chan_idx][i]));
													var_color_j(chan_idx) += ((it->second[idx2].color[j][chan_idx] - mean_[chan_idx][j]) * (it->second[idx2].color[j][chan_idx] - mean_[chan_idx][j]));
												 }


									 if (color_(0) > DBL_MAX || color_(1) > DBL_MAX || color_(2) > DBL_MAX) {
										cout << "Max value err!" << endl;
										break;
									 }

									 cov_1(i, j) = color_(0) / (double)(subblock_point_num);  // covariance of R color similarity
									 cov_2(i, j) = color_(1) / (double)(subblock_point_num);  // covariance of G color similarity
									 cov_3(i, j) = color_(2) / (double)(subblock_point_num);  // covariance of B color similarity

									 // 한 Sub Block 내 점들의 R , G , B Color에 대해서 분산 계산
									 var_color_i(0) /= (double)(subblock_point_num);
									 var_color_i(1) /= (double)(subblock_point_num);
									 var_color_i(2) /= (double)(subblock_point_num);

									 var_color_j(0) /= (double)(subblock_point_num);
									 var_color_j(1) /= (double)(subblock_point_num);
									 var_color_j(2) /= (double)(subblock_point_num);

									 // i , j에 대해 분산을 다 구하고 난 후 곱한것을 sqrt 처리함
									 if (var_color_i(0) == 0 || var_color_j(0) == 0)
										p_r = 1;

									 else if (var_color_i(1) == 0 || var_color_j(1) == 0)
										p_g = 1;

									 else if (var_color_i(2) == 0 || var_color_j(2) == 0)
										p_b = 1;

									 else if (var_color_i(0) > 0 && var_color_i(1) > 0 && var_color_i(2) > 0 && var_color_j(0) > 0 && var_color_j(1) > 0 && var_color_j(2) > 0) {
										p_r = cov_1(i, j) / sqrt(var_color_i(0) * var_color_j(0));
										p_g = cov_2(i, j) / sqrt(var_color_i(1) * var_color_j(1));
										p_b = cov_3(i, j) / sqrt(var_color_i(2) * var_color_j(2));
									 }
								  }


							   double cor_coeff = (p_r + p_g + p_b) / 3.0;
							   fout_cs << cor_coeff << ",";
							   if (cnt % 1000 == 999)
								  fout_cs << endl;

							   // sqrt 처리 한 이후 그 값으로 채널별 covariance를 나눔
							   // -1 ~ 1 사이의 값이 나오면 그것을 평균냄
							   cnt++;
							}


							// 색상 분포도 확인
							vector<vector<float>> dev_pointnum(total_num_cameras, vector<float>(5));
							vector<float> point_num_per_color(total_num_cameras);
							HSV_dev(_PPC, dev_pointnum, point_num_per_color);

							for (int i = 0; i < total_num_cameras; i++)
							   fout_dev << "," << dev_pointnum[i][0]
							   << "," << dev_pointnum[i][1]
							   << "," << dev_pointnum[i][2]
							   << "," << dev_pointnum[i][3]
							   << "," << dev_pointnum[i][4] << ",";

							fout_dev << "\n";

							*/


						//	cout << "denormalized value\n"
						//		<< (float)denormalization_s(0, geo_min[0], geo_max[0]) << "\t"
						//		<< (float)denormalization_s(0, geo_min[1], geo_max[1]) << "\t"
						//		<< (float)denormalization_s(0, geo_min[2], geo_max[2]) << endl
						//
						//		<< (float)denormalization_s(1, geo_min[0], geo_max[0]) << "\t"
						//		<< (float)denormalization_s(1, geo_min[1], geo_max[1]) << "\t"
						//		<< (float)denormalization_s(1, geo_min[2], geo_max[2]) << endl;


							cout << "projection !!!" << endl;
							vector<Mat> projection_imgs(total_num_cameras, temp_8);
							vector<Mat> filled_imgs(total_num_cameras, temp_8);
							vector<Mat> depth_value_imgs(total_num_cameras, depth_value_img);

							int nNeighbor = 4;
							int window_size = 2;
							//projection_PPC_with_hole_filling(_PPC, projection_imgs, filled_imgs, pointclouds_, nNeighbor, window_size);
							projection_PPC_with_hole_filling(_PPC_S, projection_imgs, filled_imgs, pointclouds_, nNeighbor, window_size);

							view_PC(pointclouds_[5]);


							vector<float> psnrs_p, psnrs_h;
							vector<int> num_holes;

							printPSNRWithoutBlackPixel(color_imgs, projection_imgs, psnrs_p, num_holes);
							printPSNRWithBlackPixel(color_imgs, filled_imgs, psnrs_h);

							for (int i = 0; i < total_num_cameras; i++)
							{
								imshow("projection", projection_imgs[i]);
								imshow("filled_imgs", filled_imgs[i]);
								waitKey(0);
							}


							//for (int i = 0; i < total_num_cameras; i++)
							//   fout_data << "," << num_holes[i];
							//
							//fout_data << ",";
							//
							//for (int i = 0; i < total_num_cameras; i++)
							//   fout_data << "," << psnrs_p[i];
							//
							//fout_data << ",";
							//
							//for (int i = 0; i < total_num_cameras; i++)
							//   fout_data << "," << psnrs_h[i];
							//
							//fout_data << "\n";

							Plen_PC.clear();
							_PPC.clear();

						}
						clock_t end = clock();

						cout << "====================================================================time : "
							<< float(end - start) / CLOCKS_PER_SEC << "================================================" << endl;

						color_imgs.clear();
						depth_imgs.clear();
						pointclouds.clear();
						registered_PC.reset(new PointCloud<PointXYZRGB>);
					}


					//fout_cs.close();
					//fout_os.close();
					//fout_data.close();
					//fout_dev.close();
				}
			}
		}
	}

	return 0;
}
#endif

#if 0
#include "global.h"
#include "set_environment.h"
#include "common.h"
#include "massive_point_cloud.h"
#include "plenoptic_point_cloud.h"
#include "point_cloud.h"
#include "ppc_prime.h"
#include "Codec_KLT.h"
#include "color_modeling.h"

int mode, _width, _height, total_num_cameras, total_num_frames;
double MinZ, MaxZ;
string path;
vector<CalibStruct> m_CalibParams;

int main()
{
	int voxel_div_mode = 0;
	int ppc_mode = 0;
	int shape_of_voxel = 0;
	int color_model_mode = 0;

	// 이미지나 매트릭스 데이터 입력
	cout << "Generating Plenoptic Point Cloud " << endl;
	cout << "0 : MSR3DVideo_Ballet" << endl;
	cout << "1 : Poznan_Fencing" << endl;
	cout << "2 : Intel_Kermit" << endl;
	cout << "3 : Technicolor_Painter" << endl;
	cout << "input : ";
	cin >> mode;
	cout << endl;

	cout << "1 : 1024 or 2 : 1536 or 3 : 2048 " << endl;
	cout << "input : ";
	cin >> voxel_div_mode;
	cout << endl;

	cout << "1 : PPC1(only depth_thres) or 2 : PPC1_prime(depth_thres + color_thres) or 3 : PPC2(voxelized PPC) " << endl;
	cout << "input : ";
	cin >> ppc_mode;
	cout << endl;

	cout << "1 : frustum or 2 : cube" << endl;
	cout << "input : ";
	cin >> shape_of_voxel;
	cout << endl;

	cout << "0 : no color modeling" << endl;
	cout << "1 : linear regression" << endl;
	cout << "2 : 2nd degree polynomial regression" << endl;
	cout << "3 : diffuse(linear) + specular(2nd degree polynomial) method" << endl;
	cout << "4 : KLT method" << endl;
	cout << "input : ";
	cin >> color_model_mode;

	set_parameters(mode);
	//get_num_camera_N_frame(total_num_cameras, total_num_frames);
	load_matrix_data();
	compute_projection_matrices();
	//for(int i = 0; i < total_num_cameras; i++) cout << m_CalibParams[i].m_ProjMatrix << endl << endl;
	vector<vector<string>> color_names(total_num_cameras, vector<string>(total_num_frames));
	vector<vector<string>> depth_names(total_num_cameras, vector<string>(total_num_frames));
	vector<string> color_names_(total_num_cameras);
	vector<string> depth_names_(total_num_cameras);

	// 파일 이름 다 저장해놓고 원하는 거 불러오기
	if (!mode) load_file_name(color_names, depth_names);
	else load_file_name(color_names_, depth_names_);

	Mat blank;
	if (mode == 1) {
		Mat temp(_height, _width, CV_8UC3, Scalar::all(0));
		blank = temp;
	}
	else {
		Mat temp(_height, _width, CV_16UC3, Scalar::all(0));
		blank = temp;
	}
	vector<Mat> color_imgs(total_num_cameras, blank);
	vector<Mat> depth_imgs(total_num_cameras, blank);
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds(total_num_cameras);
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds_(total_num_cameras);
	vector<PointCloud<PointXYZRGB>::Ptr> filtered_pointclouds(total_num_cameras);
	PointCloud<PointXYZRGB>::Ptr registered_PC(new PointCloud<PointXYZRGB>);

	// voxel 해상도 조절
	int voxel_div_num = 0;
	if (voxel_div_mode == 1) {
		voxel_div_num = 1024;
	}
	else if (voxel_div_mode == 2) {
		voxel_div_num = 1536;
	}
	else if (voxel_div_mode == 3) {
		voxel_div_num = 2048;
	}

	PointCloud<PointXYZRGB>::Ptr merged_PC(new PointCloud<PointXYZRGB>);

	// 100 프레임짜리 영상
	for (int frame = 0; frame < total_num_frames; frame++)
	{
		cout << "\n----------------------------<frame " << frame << ">---------------------------\n" << endl;
		//모든 카메라로부터 PC가져와서 PC배열에 넣기
		if (!mode)
			pointclouds = get_PC_of_every_camera(frame, color_names, depth_names, color_imgs, depth_imgs);
		else
			pointclouds = get_PC_of_every_camera(frame, color_names_, depth_names_, color_imgs, depth_imgs);

		//view_PC(pointclouds[0], pointclouds[2]);
		//위의 배열을 하나로 다합치기
		registered_PC = make_registered_PC(pointclouds);

		//view_PC(registered_PC);

		cout << "Before refinement, Plenoptic Point Cloud size:\t" << registered_PC->points.size() << endl;

		//view_PC(registered_PC);
	   //
		vector<PPC> Plen_PC;

		// PPC1
		if (ppc_mode == 1) {
			int depth_thres = 10;

			Plen_PC = make_sequenced_Plen_PC(pointclouds, color_imgs, depth_imgs, depth_thres); //기하1개 컬러8개 <= 기하8개 컬러8개

			if (shape_of_voxel == 1) {
				Plen_PC = make_frustum_Plen_PC(Plen_PC, pointclouds, voxel_div_num);
			}

			else if (shape_of_voxel == 2) {
				Plen_PC = make_voxelized_Plen_PC(Plen_PC, pointclouds, voxel_div_num);
			}
		}

		// PPC1 prime
		else if (ppc_mode == 2) {
			int depth_thres = 10;
			int color_thres = 50;
			Plen_PC = make_sequenced_Plen_PC(pointclouds, color_imgs, depth_imgs, depth_thres, color_thres); //기하1개 컬러8개 <= 기하8개 컬러8개

			if (shape_of_voxel == 1) {
				Plen_PC = make_frustum_Plen_PC(Plen_PC, pointclouds, voxel_div_num);
			}
			else if (shape_of_voxel == 2) {
				Plen_PC = make_voxelized_Plen_PC(Plen_PC, pointclouds, voxel_div_num);
			}
		}
		else if (ppc_mode == 3) {
			if (shape_of_voxel == 1) {
				Plen_PC = make_frustum_Plen_PC(registered_PC, pointclouds, voxel_div_num);
			}
			else if (shape_of_voxel == 2) {
				Plen_PC = make_voxelized_Plen_PC(registered_PC, pointclouds, voxel_div_num);
			}
		}

		vector<PPC> _PPC;
		if (color_model_mode == 4) {
			cout << "!!! ============= Hi PPC Prime ============= !!!" << endl;

			/* Encoder : use klt
			input: PPC_
			output: PPC_prime, cov_mat, index_flag
			*/
			vector<vector<MatrixXd>> cov_mat;//output
			vector<vector<vector< VectorXd>>> PPC_prime;//output
			vector<vector<vector<bool>>> class_PPC_merged_flag;//output
			vector<vector<PPC>> class_PPC;///output

			int start_idx = 4;
			int med_idx = 6;
			int end_idx = 8;

			PPC_prime = Encoder_(Plen_PC, cov_mat, class_PPC_merged_flag, class_PPC, start_idx, med_idx, end_idx);///Encoder

			/*
			Decoder
			intput: PPC_prime, cov_mat, idex_flag
			output: _PPC (+ _PPC_double: recovered double type PPC)
			*/
			vector<vector<vector< VectorXd>>> _PPC_double;///output

			_PPC = Decoder_(PPC_prime, cov_mat, class_PPC_merged_flag, class_PPC, _PPC_double, start_idx);///Decoder

			cout << "!!! ============= Bye PPC Prime ============= !!!" << endl;
		}

		if (color_model_mode > 0 && color_model_mode < 4) {
			if (color_model_mode == 1) {
				// Color modeling method : linear regression
				vector<PPC3> Plen_PC_color_modeling = encodeColorModelingPPC2(Plen_PC);
				_PPC = decodeColorModelingPPC2(Plen_PC_color_modeling);
			}

			else if (color_model_mode == 2) {
				//  Color modeling method : 2nd degree polynomial regression
				vector<PPC2> Plen_PC_color_modeling = encodeColorModelingPPC(Plen_PC);
				_PPC = decodeColorModelingPPC(Plen_PC_color_modeling);
			}

			else if (color_model_mode == 3) {
				// Color modeling method : diffuse (linear) + specular (2nd degree polynomial)
				vector<PPC4> Plen_PC_color_modeling = encodeColorModelingPPC3(Plen_PC);
				_PPC = decodeColorModelingPPC3(Plen_PC_color_modeling);
			}

			cout << "color modeling done!" << endl;
		}
		else if (color_model_mode == 0) {
			_PPC = Plen_PC;
		}

		cout << "After refinement, Plenoptic Point Cloud size : \t" << _PPC.size() << endl;
		vector<Mat> projection_imgs(total_num_cameras, blank);
		vector<Mat> filled_imgs(total_num_cameras, blank);

		// nNeighnor : 4, 8, 12, 20, 24
		int nNeighbor = 4;
		int window_size = 2;
		projection_PPC_with_hole_filling(_PPC, projection_imgs, filled_imgs, pointclouds_, nNeighbor, window_size);

		//view_PC(pointclouds_[0]);

		cout << endl << "<PSNR Without counting empty pixels>" << endl;
		printPSNRWithoutBlackPixel(color_imgs, projection_imgs);

		cout << endl << "<PSNR With counting empty pixels>" << endl;
		printPSNRWithBlackPixel(color_imgs, projection_imgs);

		cout << endl << "<Hole filled PSNR Without counting empty pixels>" << endl;
		printPSNRWithBlackPixel(color_imgs, filled_imgs);

		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
		{
			imshow("original", color_imgs[cam_num]);
			imshow("projection", projection_imgs[cam_num]);
			imshow("hole filling", filled_imgs[cam_num]);
			waitKey(0);
		}

		color_imgs.clear();
		pointclouds.clear();
		pointclouds_.clear();
		registered_PC.reset(new PointCloud<PointXYZRGB>);
	}
	return 0;
}
#endif
