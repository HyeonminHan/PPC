#if 1
//v2
#include "global.h"
#include "set_environment.h"
#include "common.h"
//#include "massive_point_cloud.h"
#include "plenoptic_point_cloud.h"
//#include "point_cloud.h"
//#include "ppc_prime.h"
//#include "Codec_KLT.h"
//#include "color_modeling.h"

//#define TEST
//#define SIMILARITY
//#define COLOR_MODELING

int mode, _width, _height, total_num_cameras, total_num_frames, color_bits, depth_bits; // color, depth �� bit����
double MinZ, MaxZ;
vector<Vector2d> tech_minmaxZ;
string path;
vector<CalibStruct> m_CalibParams;
vector<float> geo_min, geo_max;
double version;
int degree;

int main()
{
	//#define MSR3DVideo_Ballet 0
	//#define Poznan_Fencing 1
	//#define Intel_Kermit 2
	//#define Technicolor_Painter 3
	// ppc mode 1 : HHHP(only depth) , mode 2 : HHHP(depth+color) , mode3 : voxelized
	// shape of voxel mode 1 : frustum , mode 2 : cube
	// voxel_mode 1 : 1024 , 2 : 2048 , 3 : 4096
	// color_model mode 0 : nothing, 1 : linear, 2 : 2 degree , 3 : hybrid , 4 : KLT
	// excel의 2열 : PPC mode / 4열 : shape of voxel / 5열 : 해상도 / 6열 : 총 점 개수 / 7,8열 각각 점수, 분포, / 9,10,11 PSNR ::::: color model mode 0
	// for문 돌아가야 하는 경우의 수 : color mode 0 따로 한번(전체 뽑을 수 있는거 다 뽑기), 123 따로 한번(PSNR with hole)

	version = 3.1;
	mode = 0;
	degree = 2;

	cout << " ============================= " << endl;
	cout << "          version " << version << endl;
	cout << " ============================= " << endl;
	cout << "          mode  " << mode << endl;
	cout << " ============================= " << endl;
	cout << "          degree  " << degree << endl;
	cout << " ============================= " << endl;

#ifdef SIMILARITY
	int sub_block_mode = 0;
	int sub_block_res = 0;
#endif
	int ppc_mode = 2;
	int shape_of_voxel = 2;
	int voxel_mode = 3;
	int color_model_mode = 0;

	int voxel_div_num = 0;
	//cout << "mode: ";
	//cin >> mode;

	set_parameters(mode);
	load_matrix_data();
	compute_projection_matrices();
	// ���� �̸� �� �����س��� ���ϴ� �� �ҷ�����
	vector<vector<string>> color_names(total_num_cameras, vector<string>(total_num_frames));
	vector<vector<string>> depth_names(total_num_cameras, vector<string>(total_num_frames));
	vector<string> color_names_(total_num_cameras);
	vector<string> depth_names_(total_num_cameras);
	if (!mode) load_file_name(color_names, depth_names);
	else if(mode == hotelroom_r2_front_sample) load_file_name_mode4(color_names, depth_names);
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
	//vector<PointCloud<PointXYZRGB>::Ptr> pointclouds(total_num_cameras);
	//vector<PointCloud<PointXYZRGB>::Ptr> filtered_pointclouds(total_num_cameras);
	//PointCloud<PointXYZRGB>::Ptr registered_PC(new PointCloud<PointXYZRGB>);
	//PointCloud<PointXYZRGB>::Ptr merged_PC(new PointCloud<PointXYZRGB>);

	//int how_many_similar_points[27] = { 0, };

	int frame_num = 25;

#ifdef TEST
	for (sub_block_mode = 1; sub_block_mode <= 3; sub_block_mode++) {
		for (ppc_mode = 3; ppc_mode <= 3; ppc_mode++) {
			for (shape_of_voxel = 2; shape_of_voxel <= 2; shape_of_voxel++) {
				for (voxel_mode = 3; voxel_mode <= 3; voxel_mode++) {
					//if (ppc_mode == 3 && shape_of_voxel == 1 && voxel_mode == 2) continue; // ballet frustum batch ����
					//if (ppc_mode == 3 && shape_of_voxel == 1 && voxel_mode == 3)continue; // fencing frustum batch ����

					ofstream fout_cs;
					ofstream fout_os;
					ofstream fout_data;
					ofstream fout_dev;

					// ��ü frame�� 10%�� ������.
					frame_num = total_num_frames / 10;
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

					fout_cs.open(name_cs);
					fout_os.open(name_os);
					fout_data.open(name_data);
					fout_dev.open(name_dev);

					fout_data << "frame,PC,PPC,hole num";
					for (int i = 0; i < total_num_cameras; i++)
						fout_data << ",cam" << i;

					fout_data << ",PSNR p";
					for (int i = 0; i < total_num_cameras; i++)
						fout_data << ",cam" << i;

					fout_data << ",PSNR h";
					for (int i = 0; i < total_num_cameras; i++)
						fout_data << ",cam" << i;

					fout_data << "\n";

					fout_dev << "frame,";
					for (int i = 0; i < total_num_cameras; i++)
						fout_dev << i + 1 << "��,0~5,5~10,10~15,15~20,20~,";

					fout_dev << "\n";
#endif
					// 100 ������¥�� ����
					for (int frame = 0; frame < frame_num; frame++)
					{
						if (frame == frame_num) break;
#ifdef TEST
						fout_data << frame << ",";
						fout_dev << frame << ",";
#endif
						

						cout << "get_color_and_depth_imgs : ";
						cout << " camera number : " << total_num_cameras << endl;
						clock_t t1 = clock();
						if (!mode || mode == hotelroom_r2_front_sample)
							get_color_and_depth_imgs(frame, color_names, depth_names, color_imgs, depth_imgs);
						else
							get_color_and_depth_imgs(frame, color_names_, depth_names_, color_imgs, depth_imgs);
						clock_t t2 = clock();
						cout << float(t2 - t1) / CLOCKS_PER_SEC << endl;

						
#ifdef TEST
						fout_data << registered_PC->points.size() << ",";
#endif

						vector<PPC*> Plen_PC;
						int ppc_number = 0;

						if (voxel_mode == 1) voxel_div_num = 1024;
						else if (voxel_mode == 2) voxel_div_num = 2048;
						else if (voxel_mode == 3) voxel_div_num = 4096;

#ifdef TEST
						for (color_model_mode = 0; color_model_mode <= 0; color_model_mode++)
						{
							cout << "ppcmode : " << ppc_mode << " shape of voxel : " << shape_of_voxel << " voxel_mode : " << voxel_mode << " color model mode :" << color_model_mode << endl;
#endif
							cout << "make_PPC : ";
							clock_t t5 = clock();

							Plen_PC = make_sequenced_Plen_PC(color_imgs, depth_imgs);

							cout << "Size of PPC : " << Plen_PC.size() << endl;
							clock_t t6 = clock();

							cout <<"make_PPC : " << float(t6 - t5) / CLOCKS_PER_SEC << endl;

							//vector<PPC_S> Plen_PC_S = make_quantized_Plen_PC(Plen_PC);

							vector<PPC*> _PPC;

							if (color_model_mode == 0) _PPC = Plen_PC;

#ifdef COLOR_MODELING
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
#endif
							//clock_t end = clock();
							//cout << "time : " << float(end - start) / CLOCKS_PER_SEC << endl;
#ifdef TEST
							fout_data << Plen_PC.size() << ",";
#endif
							//cout << "After refinement, Plenoptic Point Cloud size:\t" << Plen_PC.size() << endl;
#ifdef SIMILARITY							
							// similarity ����ϴ� ���

							if (sub_block_mode == 1) sub_block_res = 8;
							else if (sub_block_mode == 2) sub_block_res = 16;
							else if (sub_block_mode == 3) sub_block_res = 32;

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

							size_t x_sub_voxel_num = voxel_div_num / sub_block_res; // 4*4*4¥�� subblock ����� 
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
								//cout << "sub block ���� �� ���� :: " << it->second.size() << endl;
								if (it->second.size() < 3) continue;

								// Hamming Distance ���ϱ�
								int hamming_distance[100] = { 0, };
								int ham_dist = 0;
								double occlustion_pattern_value = 0;

								for (int i = 0; i < it->second.size(); i++) { // ���� sub block ���� �ִ� point�� ������ŭ ���ƾ���
									for (int j = 0; j < it->second.size(); j++) {
										for (int k = 0; k < 8; k++) {
											if (i < j) {
												if ((it->second[i].occlusion_pattern[k] != it->second[j].occlusion_pattern[k]) == 1) { // XOR ����
													ham_dist++;
												}
											}
										}
									}
									hamming_distance[i] = ham_dist;
								}

								// Occlusion Pattern Similarity ���ϱ�
								int subblock_point_num = it->second.size();
								int combination_value = 0;

								vector<double> occlusion_pattern_vec;

								combination_value = combination(subblock_point_num, 2);
								occlustion_pattern_value = 1 - double(ham_dist) / double(combination_value * total_num_cameras);
#ifdef TEST
								fout_os << occlustion_pattern_value << ",";
#endif
								if (cnt % 1000 == 999)
									fout_os << endl;
								// Color Similarity ���ϱ�
								// Sub Block ���� ���󰪵��� ��� ����
								vector<int> color_mean = { 0, };

								// Covariance ���
								// Mean ���ϱ�
								vector<vector<double>> mean_(3, vector<double>(total_num_cameras));

								for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
								{
									for (int idx = 0; idx < subblock_point_num; idx++)
										for (int chan_idx = 0; chan_idx < 3; chan_idx++)
											mean_[chan_idx][cam_idx] += it->second[idx].color[cam_idx][chan_idx];

									for (int chan_idx = 0; chan_idx < 3; chan_idx++)
										mean_[chan_idx][cam_idx] /= subblock_point_num;

								}

								// Covariance ���
								const int cnt_size = total_num_cameras;
								MatrixXd cov_1(cnt_size, cnt_size);
								MatrixXd cov_2(cnt_size, cnt_size);
								MatrixXd cov_3(cnt_size, cnt_size);

								// Variance ���
								MatrixXd var_1(cnt_size, cnt_size);
								MatrixXd var_2(cnt_size, cnt_size);
								MatrixXd var_3(cnt_size, cnt_size);

								Vec3f color_;
								Vec3f var_color_i;
								Vec3f var_color_j;

								// i,j�� sub block ���� ���� ����
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

										// �� Sub Block �� ������ R , G , B Color�� ���ؼ� �л� ���
										var_color_i(0) /= (double)(subblock_point_num);
										var_color_i(1) /= (double)(subblock_point_num);
										var_color_i(2) /= (double)(subblock_point_num);

										var_color_j(0) /= (double)(subblock_point_num);
										var_color_j(1) /= (double)(subblock_point_num);
										var_color_j(2) /= (double)(subblock_point_num);

										// i , j�� ���� �л��� �� ���ϰ� �� �� ���Ѱ��� sqrt ó����
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
#ifdef TEST
								fout_cs << cor_coeff << ",";
#endif
								if (cnt % 1000 == 999)
									fout_cs << endl;

								// sqrt ó�� �� ���� �� ������ ä�κ� covariance�� ����
								// -1 ~ 1 ������ ���� ������ �װ��� ��ճ�
								cnt++;
							}
#endif
#ifdef TEST
							// ���� ������ Ȯ��
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
#endif

							vector<PPC*> vec_ppc_temp;
							string filename;
							string version_ = to_string(version).substr(0, 3); 
							if (mode == 0) filename = "binaryFiles_V" + version_ + "/ballet/ballet_v" + version_ + "_deg" + to_string(degree)+ "_f" + to_string(frame) + ".bin";
							else if (mode == 1) filename = "binaryFiles_V" + version_ + "/fencing/fencing_v" + version_ + "_deg" + to_string(degree) + "_f" + to_string(frame) + ".bin";
							else if (mode == 2) filename = "binaryFiles_V" + version_ + "/intel/intel_v" + version_ + "_deg" + to_string(degree) + "_f" + to_string(frame) + ".bin";
							else if (mode == 3) filename = "binaryFiles_V" + version_ + "/tech/tech_v" + version_ + "_deg" + to_string(degree) + "_f" + to_string(frame) + ".bin";
							else if (mode == 4) filename = "binaryFiles_V" + version_ + "/hotel/hotel_v" + version_ + "_deg" + to_string(degree) + "_f" + to_string(frame) + ".bin";
							cout << " Plen_PC.size()" << Plen_PC.size() << endl;
							//save_ppc(Plen_PC, filename);
							
							//vec_ppc_temp = load_ppc(filename);
							//cout << " vec_ppc_temp.size()" << vec_ppc_temp.size() << endl;

							//exit(1);

							vector<Mat> projection_imgs(total_num_cameras, temp_8);
							vector<Mat> filled_imgs(total_num_cameras, temp_8);

							int nNeighbor = 4;
							int window_size = 2;

							cout << "projection_PPC_with_hole_filling : ";

							clock_t t7 = clock();
							vector<PointCloud<PointXYZRGB>::Ptr> pointclouds_(total_num_cameras);
							projection_PPC_with_hole_filling(Plen_PC, projection_imgs, filled_imgs, pointclouds_, nNeighbor, window_size);
							//projection_PPC_with_hole_filling(vec_ppc_temp, projection_imgs, filled_imgs, pointclouds_, nNeighbor, window_size);
							clock_t t8 = clock();
							cout << float(t8 - t7) / CLOCKS_PER_SEC << endl;

							view_PC_yuvTorgb(pointclouds_[7]);

							vector<float> psnrs_p, psnrs_h;
							vector<int> num_holes;

							printPSNRWithoutBlackPixel_2(color_imgs, projection_imgs, psnrs_p, num_holes);
							printPSNRWithBlackPixel_2(color_imgs, filled_imgs, psnrs_h);
							cout << (int)filled_imgs[0].at<Vec3b>(1256, 609)[0]<< " " << (int)filled_imgs[0].at<Vec3b>(1256, 609)[1] << " " << (int)filled_imgs[0].at<Vec3b>(1256, 609)[2] << " " << endl;
							cout << (int)filled_imgs[0].at<Vec3b>(1257, 609)[0]<< " " << (int)filled_imgs[0].at<Vec3b>(1257, 609)[1] << " " << (int)filled_imgs[0].at<Vec3b>(1257, 609)[2] << " " << endl;
							cout << (int)filled_imgs[0].at<Vec3b>(1258, 609)[0]<< " " << (int)filled_imgs[0].at<Vec3b>(1258, 609)[1] << " " << (int)filled_imgs[0].at<Vec3b>(1258, 609)[2] << " " << endl;

							for (int i = 0; i < filled_imgs.size(); i++) {
								imshow("filled_img", filled_imgs[i]);
								//imshow("depth", depth_imgs[i]);
								//imshow("projection_img", projection_imgs[i]);
								Mat viewImg;
								cvtColor(filled_imgs[i], viewImg, CV_YUV2BGR);
								if (i == 0) imwrite("hotel.jpg", viewImg);
								imshow("viewImg", viewImg);
								waitKey(0);
							}

							exit(1);
#ifdef TEST
							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << num_holes[i];

							fout_data << ",";

							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << psnrs_p[i];

							fout_data << ",";

							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << psnrs_h[i];

							fout_data << "\n";
#endif
							Plen_PC.clear();
							_PPC.clear();
#ifdef TEST
						}

						//clock_t end = clock();
						//
						//cout << "====================================================================time : "
						//	<< float(end - start) / CLOCKS_PER_SEC << "================================================" << endl;
#endif
						color_imgs.clear();
						depth_imgs.clear();
						//pointclouds.clear();
						//registered_PC.reset(new PointCloud<PointXYZRGB>);
					}

#ifdef TEST
					fout_cs.close();
					fout_os.close();
					fout_data.close();
					fout_dev.close();
				}
			}
		}
	}
#endif

	return 0;
}
#endif

