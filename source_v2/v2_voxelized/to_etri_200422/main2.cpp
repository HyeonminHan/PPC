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
	////			2 - batch + voxelized										////
	////		voxel_mode														////
	////			0 - 10bit(1024)												////
	////			1 - 11bit(2048)												////
	////			2 - 12bit(4096)												////
	////			3 - 16bit(65536)											////
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
				

	///////input///////
	version = 2.1;
	data_mode = 12; 
	int ppc_mode = 1; 
	int voxel_mode = 3; 
	mask_size = 3; // 3 -> 3x3 / 5 -> 5x5 / ... / 11 -> 11x11
	///////////////////



	int colorspace = 0; // 0: YUV, 1: HSV, 2: BGR
	int referenceView = 220;
	furthest_index = (mask_size + 1) / 2 * 22;
	int voxel_div_num = 0;

	cout << " ============================= " << endl;
	cout << "          version  " << version << endl;
	cout << " ============================= " << endl;
	cout << "          data_mode  " << data_mode << endl;
	cout << " ============================= " << endl;
	cout << "          ppc_mode  " << ppc_mode << endl;
	cout << " ============================= " << endl;
	cout << "          voxel_mode  " << voxel_mode << endl;
	cout << " ============================= " << endl;
	cout << "          mask size  " << mask_size << endl;
	cout << " ============================= " << endl;

#ifdef TEST
	for (data_mode = 4; data_mode <= 4; data_mode++) {
#endif
		//set information of the data
		set_parameters(data_mode);

		//set view order
		camera_order = make_camOrder(referenceView);

		//load camera parameters of each view
		load_matrix_data();

		//compute projection matrices by camera parameters
		compute_projection_matrices();

		vector<string> color_names_(total_num_cameras);
		vector<string> depth_names_(total_num_cameras);

		vector<vector<string>> color_names;
		vector<vector<string>> depth_names;


		if (data_mode >= S01_H1) {

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
			blank_c = temp_8;
			blank_d = temp_16;
			break;
		}

		vector<Mat> color_imgs(total_num_cameras, blank_c);
		vector<Mat> depth_imgs(total_num_cameras, blank_d);
		Mat depth_value_img(_height, _width, CV_64F, -1);

		if (voxel_mode == 0) voxel_div_num = 1024;
		else if (voxel_mode == 1) voxel_div_num = 2048;
		else if (voxel_mode == 2) voxel_div_num = 4096;
		else if (voxel_mode == 3) voxel_div_num = 65536;

		int frame_num = 1;

#ifdef TEST
		for (version = 2.1; version <= 2.1; version += 0.1) {
			for (ppc_mode = 1; ppc_mode <= 1; ppc_mode++) {
				for (colorspace = 0; colorspace <= 0; colorspace++) {
					for (voxel_mode = 2; voxel_mode <= 2; voxel_mode++) {
						//if (voxel_mode == 2) continue;

						cout << " ============================= " << endl;
						cout << "          version  " << version << endl;
						cout << " ============================= " << endl;
						cout << "          data_mode  " << data_mode << endl;
						cout << " ============================= " << endl;
						cout << "          ppc_mode  " << ppc_mode << endl;
						cout << " ============================= " << endl;
						cout << "          color_space  " << colorspace << endl;
						cout << " ============================= " << endl;
						cout << "          voxel_mode  " << voxel_mode << endl;
						cout << " ============================= " << endl;

						if (voxel_mode == 0) voxel_div_num = 1024;
						else if (voxel_mode == 1) voxel_div_num = 2048;
						else if (voxel_mode == 2) voxel_div_num = 4096;
						else if (voxel_mode == 3) voxel_div_num = 65536;

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
						if (ppc_mode == 0) name_ppc = "incre_";
						else if (ppc_mode == 1) name_ppc = "increNvoxel_";
						else name_ppc = "batch";

						string name_colorspace;
						if (colorspace == 0) name_colorspace = "YUV";
						else if (colorspace == 1) name_colorspace = "HSV";
						else if (colorspace == 2) name_colorspace = "BGR";

						string version_ = to_string(version).substr(0, 3);
						string name_data = "output\\" + version_ + "_" + name_mode + "_" + name_ppc + "_" + name_colorspace + "_" + to_string(voxel_div_num) + "_data.csv";
						string name_dev = "output\\" + version_ + "_" + name_mode + "_" + name_ppc + "_" + name_colorspace + "_" + to_string(voxel_div_num) + "_dev.csv";

						fout_data.open(name_data);
						fout_dev.open(name_dev);

						fout_data << "frame,PC,increPPC,increNvoxPPC,degOfDecreasedPoint,hole num";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << ",cam" << i;

						fout_data << ",PSNR p";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << ",cam" << i << "_y";
						fout_data << ",";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << ",cam" << i << "_u";
						fout_data << ",";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << ",cam" << i << "_v";
						fout_data << ",";

						fout_data << ",PSNR h";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << ",cam" << i << "_y";
						fout_data << ",";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << ",cam" << i << "_u";
						fout_data << ",";
						for (int i = 0; i < total_num_cameras; i++)
							fout_data << ",cam" << i << "_v";
						fout_data << ",";
						fout_data << "\n";

						fout_dev << "frame,";
						
						fout_dev << "pointnum";
						for (int i = 1; i < total_num_cameras+1; i++)
							fout_dev << ",color" << i;
						fout_dev << ",avrdev";
						for (int i = 1; i < total_num_cameras + 1; i++)
							fout_dev << ",color" << i;
						fout_dev << ",Ydev";
						for (int i = 1; i < total_num_cameras + 1; i++)
							fout_dev << ",color" << i;
						fout_dev << ",Udev";
						for (int i = 1; i < total_num_cameras + 1; i++)
							fout_dev << ",color" << i;
						fout_dev << ",Vdev";
						for (int i = 1; i < total_num_cameras + 1; i++)
							fout_dev << ",color" << i;
							

						fout_dev << "\n";
#endif

						for (int frame = 0; frame < frame_num; frame++)
						{
#ifdef TEST
							fout_data << frame << ",";
							fout_dev << frame << ",";
#endif
							//get color images and depth images to make ppc
							if (!data_mode || data_mode >= S01_H1)
								get_color_and_depth_imgs(frame, camera_order, color_names, depth_names, color_imgs, depth_imgs);
							else
								get_color_and_depth_imgs(frame, color_names_, depth_names_, color_imgs, depth_imgs);

							cout << "get_color_and_depth_imgs done... " << endl << endl;

#ifdef TEST
							fout_data << _width * _height * total_num_cameras << ",";
#endif
							vector<PPC*> Plen_PC;
							int ppc_number = 0;
							
							clock_t t5 = clock();

							int increPPC_size = 0;

							//make ppc
							cout << "make ppc start..." << endl << endl;
							if (ppc_mode == 0) { 
								Plen_PC = make_sequenced_Plen_PC(color_imgs, depth_imgs, colorspace, camera_order);
								cout << "Size of incremental PPC : " << Plen_PC.size() << endl << endl;
#ifdef TEST			
								fout_data << "," << Plen_PC.size() << ",,";
#endif
							}
							else if (ppc_mode == 1) { 
								Plen_PC = make_sequenced_Plen_PC(color_imgs, depth_imgs, colorspace, camera_order);
								cout << "Size of incremental PPC : " << Plen_PC.size() << endl << endl;
								increPPC_size = Plen_PC.size();
#ifdef TEST
								fout_data << Plen_PC.size() << ",";
#endif
								
								Plen_PC = make_voxelized_Plen_PC(Plen_PC, voxel_div_num);
								cout << "Size of incremental + voxelized PPC : " << Plen_PC.size() << endl << endl;
#ifdef TEST
								fout_data << Plen_PC.size() << "," << 100 - ((float)Plen_PC.size() / increPPC_size * 100) << "%,";
#endif
							}
							else if (ppc_mode == 2) { 
								vector<PointCloud<PointXYZRGB>::Ptr> pointclouds;
								pointclouds = make_all_PC(color_imgs, depth_imgs);
								Plen_PC = make_voxelized_Plen_PC(pointclouds, voxel_div_num);
								cout << "Size of batch + voxelized PPC : " << Plen_PC.size() << endl << endl;
								pointclouds.clear();
							}

							clock_t t6 = clock();
							cout << "make_PPC time : ";
							cout << float(t6 - t5) / CLOCKS_PER_SEC << endl << endl;  
							cout << "make ppc done..." << endl << endl;

#ifdef TEST

							//vector<vector<float>> dev_pointnum(total_num_cameras, vector<float>(5));
							vector<vector<float>> dev_pointnum2(total_num_cameras, vector<float>(4));
							vector<float> point_num_per_color(total_num_cameras);
							
							//YUV_dev(Plen_PC, dev_pointnum, point_num_per_color);
							YUV_dev2(Plen_PC, dev_pointnum2, point_num_per_color);
							cout << dev_pointnum2.size() << endl;
							cout << point_num_per_color.size() << endl;

							/*for (int i = 0; i < total_num_cameras; i++)
								
								_dev << "," << point_num_per_color[i]
								<< "," << dev_pointnum2[i][0]
								<< "," << dev_pointnum2[i][1]
								<< "," << dev_pointnum2[i][2]
								<< "," << dev_pointnum2[i][3] << ",";

							fout_dev << "\n";*/

							for (int i = 0; i < total_num_cameras; i++)
								fout_dev << "," << point_num_per_color[i];
							fout_dev << ",";
							for (int i = 0; i < total_num_cameras; i++)
								fout_dev << "," << dev_pointnum2[i][0];
							fout_dev << ",";

							for (int i = 0; i < total_num_cameras; i++)
								fout_dev << "," << dev_pointnum2[i][1];
							fout_dev << ",";

							for (int i = 0; i < total_num_cameras; i++)
								fout_dev << "," << dev_pointnum2[i][2];
							fout_dev << ",";

							for (int i = 0; i < total_num_cameras; i++)
								fout_dev << "," << dev_pointnum2[i][3];
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

							Mat is_hole_temp(_height, _width, CV_8U, Scalar::all(1));
							vector<Mat> projection_imgs(total_num_cameras, temp_8);
							vector<Mat> filled_imgs(total_num_cameras, temp_8);
							vector<Mat> is_hole_proj_imgs(total_num_cameras, is_hole_temp);
							vector<Mat> is_hole_filled_imgs(total_num_cameras, is_hole_temp);

							int nNeighbor = 4;
							int window_size = 2;

							//execute projection of ppc to each view
							cout << "projection and hole filling time: ";
							clock_t t7 = clock();
							vector<PointCloud<PointXYZRGB>::Ptr> pointclouds_(total_num_cameras);
							vector<int> pointNum_Of_colorN(total_num_cameras, 0);
							projection_PPC_with_hole_filling(Plen_PC, projection_imgs, filled_imgs, is_hole_proj_imgs, is_hole_filled_imgs, pointclouds_, nNeighbor, window_size);
							clock_t t8 = clock();
							cout << float(t8 - t7) / CLOCKS_PER_SEC << endl << endl;
							cout << "projection and hole filling done... " << endl;

							//calculate and print PSNR
							vector<float> psnrs_p, psnrs_h;
							vector<float> psnrs_p_1, psnrs_p_2, psnrs_p_3;
							vector<float> psnrs_h_1, psnrs_h_2, psnrs_h_3;
							vector<int> num_holes_p, num_holes_h;

							printPSNRWithoutBlackPixel(color_imgs, projection_imgs, psnrs_p_1, psnrs_p_2, psnrs_p_3, num_holes_p);
							printPSNRWithBlackPixel(color_imgs, filled_imgs, psnrs_h_1, psnrs_h_2, psnrs_h_3);
							
							/*printPSNRWithoutBlackPixel_RGB(color_imgs, projection_imgs, is_hole_proj_imgs, psnrs_p_1, psnrs_p_2, psnrs_p_3, num_holes_p);
							printPSNRWithBlackPixel_RGB(color_imgs, filled_imgs, is_hole_filled_imgs, psnrs_h_1, psnrs_h_2, psnrs_h_3, num_holes_h);*/
							
							/*for (int i = 0; i < filled_imgs.size(); i++) {
								imshow("filled_img", filled_imgs[i]);

								Mat viewImg;
								cvtColor(filled_imgs[i], viewImg, CV_YUV2BGR);

								imshow("viewImg", viewImg);
								waitKey(0);
							}*/

#ifdef TEST
							Mat viewImg;
							cvtColor(filled_imgs[1], viewImg, CV_YUV2BGR);
							imwrite("output\\" + name_mode + "_v" + version_ + "_bit" + to_string(voxel_div_num) + ".png", viewImg);

							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << num_holes_p[i];

							fout_data << ",";

							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << psnrs_p_1[i];
							fout_data << ",";
							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << psnrs_p_2[i];
							fout_data << ",";
							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << psnrs_p_3[i];
							fout_data << ",";

							fout_data << ",";

							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << psnrs_h_1[i];
							fout_data << ",";
							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << psnrs_h_2[i];
							fout_data << ",";
							for (int i = 0; i < total_num_cameras; i++)
								fout_data << "," << psnrs_h_3[i];
							fout_data << ",";

							fout_data << "\n";

#endif
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
		}
	}
#endif

	return 0;
}
#endif

