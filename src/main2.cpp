#if 1
#include "global.h"
#include "set_environment.h"
#include "common.h"
#include "plenoptic_point_cloud.h"

int data_mode, _width, _height, total_num_cameras, total_num_frames, color_bits, depth_bits, mask_size, furthest_index;
double MinZ, MaxZ, scaleZ;
vector<Vector2d> tech_minmaxZ;
string path;
vector<CalibStruct> m_CalibParams;
double version;
vector<int> camera_order;
int proj_mode = 0; //0-projection , 1-backprojection

int main()
{

	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
	////		version															////
	////			2.1 - Lossless version 										////
	////			2.2 - Loss version											////
	////		data_mode 														////
	////			0 - ballet , 1 - fencing, 2 - intel, 3 - tech, 				////
	////			4, 5 - hotel(front) , 6, 7 - hotel(back)					////
	////			8, 9 - restaurant(left), 10, 11 - restaurant(back)			////
	////			12, 13 - Apartment(left)									////
	////		ppc_mode 														////
	////			0 - only incremental										////
	////			1 - incremental + voxelized									////
	////			2 - batch + voxelized	
	////			3 - formulaic
	////		voxel_mode														////
	////			0 - 10bit(1024)												////
	////			1 - 11bit(2048)												////
	////			2 - 12bit(4096)												////
	////			3 - 16bit(65536)											////
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	///////input///////
	version = 1.0;
	data_mode = 5;
	int ppc_mode = 3;
	mask_size = 11; // 3 -> 3x3 / 5 -> 5x5 / ... / 11 -> 11x11
	int voxel_div_num = 8192;
	///////////////////

	int colorspace = 0; // 0: YUV, 1: HSV, 2: BGR
	int referenceView = 220;
	furthest_index = (mask_size + 1) / 2 * 22;

#ifndef TEST
	cout << " ============================= " << endl;
	cout << "          data_mode  " << data_mode << endl;
	cout << " ============================= " << endl;
	cout << "          version  " << version << endl;
	cout << " ============================= " << endl;
	cout << "          ppc_mode  " << ppc_mode << endl;
	cout << " ============================= " << endl;
	cout << "          mask size  " << mask_size << endl;
	cout << " ============================= " << endl;
	cout << "      voxel_div_num  " << voxel_div_num << endl;
	cout << " ============================= " << endl;
#endif

#ifdef TEST
	vector<int> datas = { 5 };
	for (int data_i = 0; data_i < datas.size(); data_i++) {
		data_mode = datas[data_i];
#endif
		//set information of the data
		set_parameters(data_mode);

		//set view order
		camera_order = make_camOrder(referenceView);
		/*camera_order = { 221,
			220,
			222,
			219,
			218,
			201,
			239,
			200,
			240,
			199,
			241,
			198,
			242,
			197,
			243,
			180,
			260,
			179,
			261,
			178,
			262,
			177,
			263,
			176,
			264
		};*/
		//for (int i = 0; i < total_num_cameras; i++) cout << camera_order[i] << endl;

		//load camera parameters of each view
		load_matrix_data();

		//compute projection matrices by camera parameters
		compute_projection_matrices();

		vector<string> color_names_(total_num_cameras);
		vector<string> depth_names_(total_num_cameras);

		vector<vector<string>> color_names;
		vector<vector<string>> depth_names;

		if (referenceView == 220 && data_mode >= S01_H1) {
			for (int i = 0; i < referenceView + furthest_index; i++) {
				vector<string> temp_vec;
				temp_vec.resize(total_num_frames);
				color_names.push_back(temp_vec);
				depth_names.push_back(temp_vec);
			}
		}
		else {
			for (int i = 0; i < total_num_cameras; i++) {
				vector<string> temp_vec;
				temp_vec.resize(total_num_frames);
				color_names.push_back(temp_vec);
				depth_names.push_back(temp_vec);
			}
		}
		cout << "color name resize done .. " << endl << endl;

		//load image names
		if (!data_mode) load_file_name(color_names, depth_names);
		else if (data_mode >= S01_H1) load_file_name_mode4(color_names, depth_names, referenceView);
		else load_file_name(color_names_, depth_names_);

		cout << "load_file_name done .. " << endl << endl;

		Mat blank_c, blank_d;
		Mat temp_8(_height, _width, CV_8UC3, Scalar::all(0));
		Mat temp_16(_height, _width, CV_16UC3, Scalar::all(0));

		switch (data_mode) {
		case 0:
			blank_c = temp_8;
			blank_d = temp_8;
			break;

		case 1:
		case 2:
		case 3: 
		//case 4: case 5: case 6: case 7: case 8:
		//case 9: case 10: case 11: case 12 :
			blank_c = temp_8;
			blank_d = temp_16;
			break;
		}

		vector<Mat> color_imgs(total_num_cameras, blank_c);
		vector<Mat> depth_imgs(total_num_cameras, blank_d);
		Mat depth_value_img(_height, _width, CV_64F, -1);

		int frame_num = 1;

#ifdef TEST
		vector<int> voxel_div_nums = { 8192 };
		for (int voxel_i = 0; voxel_i < voxel_div_nums.size(); voxel_i++) {
			voxel_div_num = voxel_div_nums[voxel_i];
			for (ppc_mode = 3; ppc_mode <= 3; ppc_mode++) {
				if (ppc_mode == 2) continue;

				cout << " ============================= " << endl;
				cout << "          version  " << version << endl;
				cout << " ============================= " << endl;
				cout << "          data_mode  " << data_mode << endl;
				cout << " ============================= " << endl;
				cout << "          ppc_mode  " << ppc_mode << endl;
				cout << " ============================= " << endl;
				cout << "         color_space  " << colorspace << endl;
				cout << " ============================= " << endl;
				cout << "      voxel_div_num  " << voxel_div_num << endl;
				cout << " ============================= " << endl;

				ofstream fout_data;
				ofstream fout_dev;

				frame_num = 1; //total_num_frames / 10;
				int cnt = 0;

				string name_mode;
				if (data_mode == 0) name_mode = "ballet";
				else if (data_mode == 1) name_mode = "fencing";
				else if (data_mode == 2) name_mode = "intel";
				else if (data_mode == 3) name_mode = "tech";
				else if (data_mode == 4) name_mode = "hotel1";
				else if (data_mode == 5) name_mode = "hotel2";
				else if (data_mode == 6) name_mode = "hotel3";
				else if (data_mode == 7) name_mode = "hotel4";
				else if (data_mode == 8) name_mode = "rest1";
				else if (data_mode == 9) name_mode = "rest2";
				else if (data_mode == 10) name_mode = "rest3";
				else if (data_mode == 11) name_mode = "rest4";
				else if (data_mode == 12) name_mode = "apart1";
				else if (data_mode == 13) name_mode = "apart2";

				string name_ppc;
				if (ppc_mode == 0) name_ppc = "incre";
				else if (ppc_mode == 1) name_ppc = "increNvoxel";
				else if (ppc_mode == 2) name_ppc = "batch";
				else if (ppc_mode == 3) name_ppc = "formulaic";

				string name_colorspace;
				if (colorspace == 0) name_colorspace = "YUV";
				else if (colorspace == 1) name_colorspace = "HSV";
				else if (colorspace == 2) name_colorspace = "BGR";

				string version_ = to_string(version).substr(0, 3);
				string name_data = "output\\" + version_ + "_" + name_mode + "_" + name_ppc + "_" + name_colorspace + "_" + to_string(voxel_div_num) + "_data.csv";
				string name_dev = "output\\" + version_ + "_" + name_mode + "_" + name_ppc + "_" + name_colorspace + "_" + to_string(voxel_div_num) + "_dev.csv";

				fout_data.open(name_data);
				fout_dev.open(name_dev);

				if (ppc_mode == 1) fout_data << "frame,#PC,depth_threhsold,increPPC,increNvoxPPC,degOfDecreasedPoint,Cube_x_size,Cube_y_size,Cube_z_size,cube_x_size,cube_y_size,cube_z_size,";
				else if (ppc_mode == 3) fout_data << "frame,#PC,formulaicPPC,degOfDecreasedPoint,Cube_x_size,Cube_y_size,Cube_z_size,cube_x_size,cube_y_size,cube_z_size,";
				fout_data << "\n";

				fout_dev << "frame,";
				fout_dev << "\n";

#endif
				for (int frame = 0; frame < frame_num; frame++)
				{

#ifdef TEST
					fout_data << frame << ",";
					fout_dev << frame << "\n";
#endif
					//get color images and depth images to make ppc
					if (!data_mode || data_mode >= S01_H1) get_color_and_depth_imgs(frame, camera_order, color_names, depth_names, color_imgs, depth_imgs);
					else get_color_and_depth_imgs(frame, color_names_, depth_names_, color_imgs, depth_imgs);

					cout << "get_color_and_depth_imgs done... " << endl << endl;

#ifdef TEST
					fout_data << _width * _height * total_num_cameras << ",";
					int increPPC_size = 0;
#endif
					vector<PPC*> Plen_PC;
					int ppc_number = 0;

					clock_t t5 = clock();

					cout << "voxel_div_num : " << voxel_div_num << endl;
					if (ppc_mode == 1) {
						float depth_threshold;
						Plen_PC = make_incremental_Plen_PC(color_imgs, depth_imgs, colorspace, camera_order, voxel_div_num, depth_threshold);
						cout << "Size of incremental PPC : " << Plen_PC.size() << endl << endl;
#ifdef TEST
						increPPC_size = Plen_PC.size();
						fout_data << depth_threshold << "," << Plen_PC.size() << ",";
#endif
						vector<float> Cube_size, cube_size;

						Plen_PC = make_voxelized_Plen_PC(Plen_PC, voxel_div_num, Cube_size, cube_size);
						cout << "Size of incremental + voxelized PPC : " << Plen_PC.size() << endl << endl;

#ifdef TEST
						fout_data << Plen_PC.size() << "," << 100 - ((float)Plen_PC.size() / (_width * _height * total_num_cameras) * 100) << "%," <<
							Cube_size[0] << "," << Cube_size[1] << "," << Cube_size[2] << "," <<
							cube_size[0] << "," << cube_size[1] << "," << cube_size[2] << ",";
#endif
					}
					else if (ppc_mode == 3) {
						vector<float> Cube_size, cube_size;
						Plen_PC = make_formulaic_voxelized_Plen_PC2(color_imgs, depth_imgs, voxel_div_num, Cube_size, cube_size);

#ifdef TEST			
						fout_data << Plen_PC.size() << "," << 100 - ((float)Plen_PC.size() / (_width * _height * total_num_cameras) * 100) << "%," <<
							Cube_size[0] << "," << Cube_size[1] << "," << Cube_size[2] << "," <<
							cube_size[0] << "," << cube_size[1] << "," << cube_size[2] << ",";
#endif

						cout << "ppc size : " << Plen_PC.size() << endl;
					}
					clock_t t6 = clock();
					cout << "make_PPC time : ";
					cout << float(t6 - t5) / CLOCKS_PER_SEC << endl << endl;
					cout << "make ppc done..." << endl << endl;

#ifdef TEST
					vector<vector<float>> dev_pointnum(total_num_cameras, vector<float>(4, 0));
					vector<int> point_num_per_color(total_num_cameras, 0);
					vector<int> full_color_dev(20, 0);

					//YUV_dev(Plen_PC, dev_pointnum, point_num_per_color);
					YUV_dev2(Plen_PC, dev_pointnum, point_num_per_color, full_color_dev);

					fout_dev << "pointNum" << "\n";
					for (int i = 0; i < total_num_cameras; i++)
						fout_dev << "color" << i + 1 << "," << point_num_per_color[i] << "\n";
					fout_dev << "\n";
					
					fout_dev << "Y dev" << "\n";
					for (int i = 0; i < total_num_cameras; i++)
						fout_dev << "color" << i + 1 << "," << dev_pointnum[i][1] << "\n";
					fout_dev << "\n";

					fout_dev << "U dev" << "\n";
					for (int i = 0; i < total_num_cameras; i++)
						fout_dev << "color" << i + 1 << "," << dev_pointnum[i][2] << "\n";
					fout_dev << "\n";

					fout_dev << "V dev" << "\n";
					for (int i = 0; i < total_num_cameras; i++)
						fout_dev << "color" << i + 1 << "," << dev_pointnum[i][3] << "\n";
					fout_dev << "\n";

					fout_dev << "# point per color dev of full color " << "\n";
					for (int i = 0; i < full_color_dev.size(); i++)
						fout_dev << i*5 <<"-"<<(i+1)*5 << "," << full_color_dev[i]<< "\n";
					fout_dev << "\n";

#endif
					{
						//저장 및 로드 
						//vector<PPC*> vec_ppc_temp;
						//string filename;
						//string version_ = to_string(version).substr(0, 3);
						//if (data_mode == 0) filename = "binaryFiles_V" + version_ + "/ballet/ballet_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						//else if (data_mode == 1) filename = "binaryFiles_V" + version_ + "/fencing/fencing_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						//else if (data_mode == 2) filename = "binaryFiles_V" + version_ + "/intel/intel_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						//else if (data_mode == 3) filename = "binaryFiles_V" + version_ + "/tech/tech_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						//else if (data_mode == 4) filename = "binaryFiles_V" + version_ + "/hotel/hotel_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						//else if (data_mode == 7) filename = "binaryFiles_V" + version_ + "/hotel4/hotel4_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						//cout << " Plen_PC.size()" << Plen_PC.size() << endl;

						/*cout << "save_ppc time: ";
						clock_t t9 = clock();
						save_ppc(Plen_PC, filename);
						clock_t t10 = clock();
						cout << float(t10 - t9) / CLOCKS_PER_SEC << endl;*/

						//vec_ppc_temp = load_ppc(filename);
						//cout << " vec_ppc_temp.size()" << vec_ppc_temp.size() << endl;
					}

					for (proj_mode = 0; proj_mode <= 1; proj_mode++) {
						if (proj_mode == 1) continue;

						cout << "===============================" << endl;
						cout << "          proj_mode : " << proj_mode << endl;
						cout << "===============================" << endl;
						Mat is_hole_temp(_height, _width, CV_8U, Scalar::all(1));
						vector<Mat> projection_imgs(total_num_cameras, temp_8);
						vector<Mat> filled_imgs(total_num_cameras, temp_8);
						vector<Mat> is_hole_proj_imgs(total_num_cameras, is_hole_temp);
						vector<Mat> is_hole_filled_imgs(total_num_cameras, is_hole_temp);

						int nNeighbor = 4;
						int window_size = 2;

						//execute projection of ppc to each view
						clock_t t7 = clock();
						vector<PointCloud<PointXYZRGB>::Ptr> pointclouds_(total_num_cameras);
						vector<int> pointNum_Of_colorN(total_num_cameras, 0);
						projection_PPC_with_hole_filling(Plen_PC, projection_imgs, filled_imgs, is_hole_proj_imgs, is_hole_filled_imgs, pointclouds_, nNeighbor, window_size);
						
						//cout << "point size : " << pointclouds_[0]->points.size() << endl;
						clock_t t8 = clock();
						cout << "projection and hole filling time: " << float(t8 - t7) / CLOCKS_PER_SEC << endl << endl;

						//calculate and print PSNR
						vector<float> psnrs_p, psnrs_h;
						vector<float> psnrs_p_1, psnrs_p_2, psnrs_p_3;
						vector<float> psnrs_h_1, psnrs_h_2, psnrs_h_3;
						vector<int> num_holes_p, num_holes_h;

						printPSNRWithoutBlackPixel_RGB(color_imgs, projection_imgs, is_hole_proj_imgs, psnrs_p_1, psnrs_p_2, psnrs_p_3, num_holes_p);
						printPSNRWithBlackPixel_RGB(color_imgs, filled_imgs, is_hole_filled_imgs, psnrs_h_1, psnrs_h_2, psnrs_h_3, num_holes_h);

						/*for (int i = 0; i < filled_imgs.size(); i++) {
							imshow("filled_img", filled_imgs[i]);

							Mat view_projImg, view_filledImg;
							cvtColor(projection_imgs[i], view_projImg, CV_YUV2BGR);
							cvtColor(filled_imgs[i], view_filledImg, CV_YUV2BGR);

							imshow("view_projImg", view_projImg);
							imshow("view_filledImg", view_filledImg);
							waitKey(0);
						}*/
#ifdef TEST
						
						fout_data << "\n\n";
						if (proj_mode == 0) fout_data << "projection" << "\n";
						else if (proj_mode == 1) fout_data << "back_projection" << "\n";
						fout_data << "time" << ",";
						fout_data << float(t6 - t5) / CLOCKS_PER_SEC + float(t8 - t7) / CLOCKS_PER_SEC << ",\n\n";

						map<int, int> num_holes_p_map, num_holes_h_map;
						map<int, float> psnrs_p_1_map, psnrs_p_2_map, psnrs_p_3_map, psnrs_h_1_map, psnrs_h_2_map, psnrs_h_3_map;
						map<int, Mat> proj_imgs_map, filled_imgs_map;
						for (int i = 0; i < total_num_cameras; i++) {
							num_holes_p_map.insert(make_pair(camera_order[i], num_holes_p[i]));
							num_holes_h_map.insert(make_pair(camera_order[i], num_holes_h[i]));
							psnrs_p_1_map.insert(make_pair(camera_order[i], psnrs_p_1[i]));
							psnrs_p_2_map.insert(make_pair(camera_order[i], psnrs_p_2[i]));
							psnrs_p_3_map.insert(make_pair(camera_order[i], psnrs_p_3[i]));
							psnrs_h_1_map.insert(make_pair(camera_order[i], psnrs_h_1[i]));
							psnrs_h_2_map.insert(make_pair(camera_order[i], psnrs_h_2[i]));
							psnrs_h_3_map.insert(make_pair(camera_order[i], psnrs_h_3[i]));
							proj_imgs_map.insert(make_pair(camera_order[i], projection_imgs[i]));
							filled_imgs_map.insert(make_pair(camera_order[i], filled_imgs[i]));
						}

						Mat proj_viewImg, filled_viewImg;
						int count_it = 0;

						string folder_name_string = "output\\image\\" + name_mode;
						const char* foler_name = folder_name_string.c_str();// +name_mode;

						CreateDirectory(foler_name, NULL);

						for (map<int, Mat>::iterator it = proj_imgs_map.begin(); it != proj_imgs_map.end(); it++) {
							cvtColor(it->second, proj_viewImg, CV_YUV2BGR);
							imwrite("output\\image\\" + name_mode + "\\" + version_ + "_" + name_mode + "_" + name_ppc + "_" + to_string(voxel_div_num) + "_projmode" + to_string(proj_mode) + "_view" + to_string(count_it++) + "_proj.png", proj_viewImg);
						}
						count_it = 0;
						for (map<int, Mat>::iterator it = filled_imgs_map.begin(); it != filled_imgs_map.end(); it++) {
							cvtColor(it->second, filled_viewImg, CV_YUV2BGR);
							imwrite("output\\image\\" + name_mode + "\\" + version_ + "_" + name_mode + "_" + name_ppc + "_" + to_string(voxel_div_num) + "_projmode" + to_string(proj_mode) + "_view" + to_string(count_it++) + "_filled.png", filled_viewImg);
						}

						/*for (map<int, Mat>::iterator it = filled_imgs_map.begin(); it != filled_imgs_map.end(); it++) {
							imshow("filled_img", it->second);

							Mat viewImg;
							cvtColor(it->second, viewImg, CV_YUV2BGR);

							imshow("viewImg", viewImg);
							waitKey(0);
						}*/

						fout_data << "hole_num\n";
						count_it = 0;
						for (map<int, int>::iterator it = num_holes_p_map.begin(); it != num_holes_p_map.end(); it++) {
							fout_data << "cam" << count_it++ << "," << it->second << "\n";
						}
						count_it = 0;
						fout_data << "\nPSNR_without_hole_R\n";
						for (map<int, float>::iterator it = psnrs_p_1_map.begin(); it != psnrs_p_1_map.end(); it++) {
							fout_data << "cam" << count_it++ << "," << it->second << "\n";
						}
						count_it = 0;
						fout_data << "\nPSNR_without_hole_G\n";
						for (map<int, float>::iterator it = psnrs_p_2_map.begin(); it != psnrs_p_2_map.end(); it++) {
							fout_data << "cam" << count_it++ << "," << it->second << "\n";
						}
						count_it = 0;
						fout_data << "\nPSNR_without_hole_B\n";
						for (map<int, float>::iterator it = psnrs_p_3_map.begin(); it != psnrs_p_3_map.end(); it++) {
							fout_data << "cam" << count_it++ << "," << it->second << "\n";
						}
						count_it = 0;
						fout_data << "\n";

						fout_data << "\nhole_num_after_holefilling\n";
						for (map<int, int>::iterator it = num_holes_h_map.begin(); it != num_holes_h_map.end(); it++) {
							fout_data << "cam" << count_it++ << "," << it->second << "\n";
						}
						count_it = 0;
						fout_data << "\nPSNR_with_hole_R\n";
						for (map<int, float>::iterator it = psnrs_h_1_map.begin(); it != psnrs_h_1_map.end(); it++) {
							fout_data << "cam" << count_it++ << "," << it->second << "\n";
						}
						count_it = 0;
						fout_data << "\nPSNR_with_hole_G\n";
						for (map<int, float>::iterator it = psnrs_h_2_map.begin(); it != psnrs_h_2_map.end(); it++) {
							fout_data << "cam" << count_it++ << "," << it->second << "\n";
						}
						count_it = 0;
						fout_data << "\nPSNR_with_hole_B\n";
						for (map<int, float>::iterator it = psnrs_h_3_map.begin(); it != psnrs_h_3_map.end(); it++) {
							fout_data << "cam" << count_it++ << "," << it->second << "\n";
						}
						count_it = 0;
						fout_data << "\n\n";

						////////////////////////////////////////////////////////////////////

						/*for (int i = 0; i < total_num_cameras; i++)
							fout_data << "cam" << i << "," << num_holes_p[i] << "\n";

						fout_data << "\nPSNR_without_hole_R\n";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << "cam" << i << "," << psnrs_p_1[i] << "\n";
						fout_data << "\nPSNR_without_hole_G\n";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << "cam" << i << "," << psnrs_p_2[i] << "\n";
						fout_data << "\nPSNR_without_hole_B\n";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << "cam" << i << "," << psnrs_p_3[i] << "\n";
						fout_data << "\n";

						fout_data << "\nPSNR_with_hole_R\n";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << "cam" << i << "," << psnrs_h_1[i] << "\n";
						fout_data << "\nPSNR_with_hole_G\n";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << "cam" << i << "," << psnrs_h_2[i] << "\n";
						fout_data << "\nPSNR_with_hole_B\n";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << "cam" << i << "," << psnrs_h_3[i] << "\n";
						fout_data << "\n\n";*/

#endif
					}

					Plen_PC.clear();
					color_imgs.clear();
					depth_imgs.clear();
				}
#ifdef TEST
				if (ppc_mode == 0) break;

				fout_data.close();
				fout_dev.close();
			}
		}
	}
#endif



	return 0;
}
#endif

