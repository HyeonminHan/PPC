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

int data_mode, _width, _height, total_num_cameras, total_num_frames, color_bits, depth_bits; // color, depth �� bit����
double MinZ, MaxZ;
vector<Vector2d> tech_minmaxZ;
string path;
vector<CalibStruct> m_CalibParams;
double version;
vector<int> camera_order;

int main()
{
	//#define MSR3DVideo_Ballet 0
	//#define Poznan_Fencing 1
	//#define Intel_Kermit 2
	//#define Technicolor_Painter 3

	// voxel_mode 1 : 1024 , 2 : 2048 , 3 : 4096
	// color_model mode 0 : nothing, 1 : linear, 2 : 2 degree , 3 : hybrid , 4 : KLT
	// excel의 2열 : PPC mode / 4열 : shape of voxel / 5열 : 해상도 / 6열 : 총 점 개수 / 7,8열 각각 점수, 분포, / 9,10,11 PSNR ::::: color model mode 0
	// for문 돌아가야 하는 경우의 수 : color mode 0 따로 한번(전체 뽑을 수 있는거 다 뽑기), 123 따로 한번(PSNR with hole)



#ifdef SIMILARITY
	int sub_block_mode = 0;
	int sub_block_res = 0;
#endif

	version = 2.1;
	data_mode = 4; //0 : ballet , 1 : fencing, 2 : intel, 3 : tech, 4 : hotel
	int ppc_mode = 0; //0 : only incremental, 1 : incremental + voxelized, 2 : batch + voxelized
	int voxel_mode = 3; // 0 : 1024, 1 : 2048, 2: 4096, 3 : 65536
	int voxel_div_num = 0;
	int colorspace = 0; // 0: YUV, 1: HSV, 2: BGR

	cout << " ============================= " << endl;
	cout << "          version  " << version << endl;
	cout << " ============================= " << endl;
	cout << "          data_mode  " << data_mode << endl;
	cout << " ============================= " << endl;
	cout << "          ppc_mode  " << ppc_mode << endl;
	cout << " ============================= " << endl;
	cout << "          voxel_mode  " << voxel_mode << endl;
	cout << " ============================= " << endl;


	set_parameters(data_mode);

	int referenceView = 220;
	camera_order = make_camOrder(referenceView);

	/*for (int i = 0; i < camera_order.size(); i++)
		cout << camera_order[i] << endl;*/

	load_matrix_data();

	compute_projection_matrices();
	
	vector<string> color_names_(total_num_cameras);
	vector<string> depth_names_(total_num_cameras);

	//vector<vector<string>> color_names(total_num_cameras, vector<string>(total_num_frames));
	//vector<vector<string>> depth_names(total_num_cameras, vector<string>(total_num_frames));

	vector<vector<string>> color_names;
	vector<vector<string>> depth_names;

	if (referenceView == 220) {
		//color_names.resize(MAXNUM_11X11);
		//depth_names.resize(MAXNUM_11X11);

		for (int i = 0; i < MAXNUM_5X5; i++) { //MAXNUM_11X11
			vector<string> temp_vec1, temp_vec2;
			temp_vec1.resize(total_num_frames);
			temp_vec2.resize(total_num_frames);
			color_names.push_back(temp_vec1);
			depth_names.push_back(temp_vec2);
		}
	}
	else {
		//color_names.resize(total_num_cameras);
		//depth_names.resize(total_num_cameras);

		for (int i = 0; i < total_num_cameras; i++) {
			vector<string> temp_vec;
			temp_vec.resize(total_num_frames);
			color_names.push_back(temp_vec);
			depth_names.push_back(temp_vec);
		}
	}

	cout << "color name resize done .. " << endl;

	if (!data_mode) load_file_name(color_names, depth_names);
	else if (data_mode == hotelroom_r2_front_sample) load_file_name_mode4(color_names, depth_names, referenceView);
	else load_file_name(color_names_, depth_names_);

	cout << "load_file_name done .. " << endl;
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


	int frame_num =1 ;

#ifdef TEST
	for(version = 2.1 ; version <= 2.2 ; version += 0.1){
		for (ppc_mode = 1; ppc_mode <= 1; ppc_mode++) {
			for (colorspace = 0; colorspace <= 2; colorspace++) {
				for (voxel_mode = 3; voxel_mode <= 3; voxel_mode++) {
					//ppc mode --- 0 : only incremental, 1 : incremental + voxelized, 2 : batch + voxelized
					//voxel mode --- 0 : 1024, 1 : 2048, 2: 4096, 3 : 65536

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


					ofstream fout_data;
					ofstream fout_dev;

					// ��ü frame�� 10%�� ������.
					frame_num = 3;//total_num_frames / 10;
					int cnt = 0;


					if (voxel_mode == 0) voxel_div_num = 1024;
					else if (voxel_mode == 1) voxel_div_num = 2048;
					else if (voxel_mode == 2) voxel_div_num = 4096;
					else if (voxel_mode == 3) voxel_div_num = 65536;

					string name_mode;
					if (data_mode == 0) name_mode = "ballet";
					else if (data_mode == 1) name_mode = "fencing";
					else if (data_mode == 2) name_mode = "intel";
					else if (data_mode == 3) name_mode = "tech";
					else if (data_mode == 4) name_mode = "hotel";

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
						fout_data << ",cam" << i;

					fout_data << ",PSNR h";
					for (int i = 0; i < total_num_cameras; i++)
						fout_data << ",cam" << i;

					fout_data << "\n";

					fout_dev << "frame,";
					for (int i = 0; i < total_num_cameras; i++)
						fout_dev << i + 1 << ",pointNum,0~5,5~10,10~15,15~20,20~,";

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
						//cout << "get_color_and_depth_imgs : ";
						cout << "before get color depth imgs " << endl;
						clock_t t1 = clock();
						if (!data_mode || data_mode == hotelroom_r2_front_sample)
							get_color_and_depth_imgs(frame, camera_order, color_names, depth_names, color_imgs, depth_imgs);
						else
							get_color_and_depth_imgs(frame, color_names_, depth_names_, color_imgs, depth_imgs);

						cout << "get_color_and_depth_imgs done... " << endl;
						clock_t t2 = clock();
						//cout << float(t2 - t1) / CLOCKS_PER_SEC << endl;

#ifdef TEST
				//전체 점의 개수 
						fout_data << _width * _height * total_num_cameras << ",";
#endif

						vector<PPC*> Plen_PC;

						int ppc_number = 0;

						cout << "make_PPC time : ";
						clock_t t5 = clock();

						int increPPC_size = 0;

						if (ppc_mode == 0) { //only incremental
							Plen_PC = make_sequenced_Plen_PC(color_imgs, depth_imgs, colorspace, camera_order);
							cout << "Size of incremental PPC : " << Plen_PC.size() << endl;
#ifdef TEST			
							fout_data << "," << Plen_PC.size() << ",,";
#endif
						}
						else if (ppc_mode == 1) { //incremental + voxelized
							Plen_PC = make_sequenced_Plen_PC(color_imgs, depth_imgs, colorspace, camera_order);


							/*cout << "		loac PPC time : ";
							clock_t t5 = clock();
							string filename = "hotel_v2.1_m0_c0_vd2048_f0.bin";
							Plen_PC = load_ppc(filename);
							clock_t t6 = clock();
							cout << float(t6 - t5) / CLOCKS_PER_SEC << endl;*/

							cout << "Size of incremental PPC : " << Plen_PC.size() << endl;
							increPPC_size = Plen_PC.size();
#ifdef TEST
							fout_data << Plen_PC.size() << ",";
#endif
							Plen_PC = make_voxelized_Plen_PC2(Plen_PC, voxel_div_num);
							cout << "Size of incremental + voxelized PPC : " << Plen_PC.size() << endl;

#ifdef TEST
							fout_data << Plen_PC.size() << "," << 100 - ((float)Plen_PC.size() / increPPC_size * 100) << "%,";
#endif
						}
						else if (ppc_mode == 2) { //batch + voxelized
							vector<PointCloud<PointXYZRGB>::Ptr> pointclouds;
							pointclouds = make_all_PC(color_imgs, depth_imgs);
							Plen_PC = make_voxelized_Plen_PC(pointclouds, voxel_div_num);
							pointclouds.clear();
						}

						clock_t t6 = clock();

						cout << float(t6 - t5) / CLOCKS_PER_SEC << endl;


#ifdef TEST

						vector<vector<float>> dev_pointnum(total_num_cameras, vector<float>(5));
						vector<float> point_num_per_color(total_num_cameras);
						//YUV_dev(Plen_PC, dev_pointnum, point_num_per_color);

						/*for (int i = 0; i < total_num_cameras; i++)
							fout_dev << "," << point_num_per_color[i]
							<< "," << dev_pointnum[i][0]
							<< "," << dev_pointnum[i][1]
							<< "," << dev_pointnum[i][2]
							<< "," << dev_pointnum[i][3]
							<< "," << dev_pointnum[i][4] << ",";

						fout_dev << "\n";*/
#endif
						//저장 및 로드 
						vector<PPC*> vec_ppc_temp;
						string filename;
						string version_ = to_string(version).substr(0, 3);
						if (data_mode == 0) filename = "binaryFiles_V" + version_ + "/ballet/ballet_v" + version_ + "_m" + to_string(ppc_mode) +"_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						else if (data_mode == 1) filename = "binaryFiles_V" + version_ + "/fencing/fencing_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						else if (data_mode == 2) filename = "binaryFiles_V" + version_ + "/intel/intel_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						else if (data_mode == 3) filename = "binaryFiles_V" + version_ + "/tech/tech_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						else if (data_mode == 4) filename = "binaryFiles_V" + version_ + "/hotel/hotel_v" + version_ + "_m" + to_string(ppc_mode) + "_c" + to_string(colorspace) + "_vd" + to_string(voxel_div_num) + "_f" + to_string(frame) + ".bin";
						//cout << " Plen_PC.size()" << Plen_PC.size() << endl;
						
						/*cout << "save_ppc time: ";
						clock_t t9 = clock();
						save_ppc(Plen_PC, filename);
						clock_t t10 = clock();
						cout << float(t10 - t9) / CLOCKS_PER_SEC << endl;*/

						//vec_ppc_temp = load_ppc(filename);
						//cout << " vec_ppc_temp.size()" << vec_ppc_temp.size() << endl;


						vector<Mat> projection_imgs(total_num_cameras, temp_8);
						vector<Mat> filled_imgs(total_num_cameras, temp_8);

						int nNeighbor = 4;
						int window_size = 2;

						cout << "projection_PPC_with_hole_filling time: ";

						clock_t t7 = clock();
						vector<PointCloud<PointXYZRGB>::Ptr> pointclouds_(total_num_cameras);
						vector<int> pointNum_Of_colorN(total_num_cameras, 0);
						projection_PPC_with_hole_filling(Plen_PC, projection_imgs, filled_imgs, pointclouds_, nNeighbor, window_size);
						//projection_PPC_with_hole_filling(vec_ppc_temp, projection_imgs, filled_imgs, pointclouds_, nNeighbor, window_size);
						clock_t t8 = clock();
						cout << float(t8 - t7) / CLOCKS_PER_SEC << endl;

						//view_PC_yuvTorgb(pointclouds_[1]);

						vector<float> psnrs_p, psnrs_h;
						vector<int> num_holes;

						printPSNRWithoutBlackPixel_2(color_imgs, projection_imgs, psnrs_p, num_holes);
						printPSNRWithBlackPixel_2(color_imgs, filled_imgs, psnrs_h);

						for (int i = 0; i < filled_imgs.size(); i++) {
							imshow("filled_img", filled_imgs[i]);

							Mat viewImg;
							cvtColor(filled_imgs[i], viewImg, CV_YUV2BGR);

							imshow("viewImg", viewImg);
							waitKey(0);
						}

						//exit(1);
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
						color_imgs.clear();
						depth_imgs.clear();
						//pointclouds.clear();
					}

#ifdef TEST
					if (ppc_mode == 0) break;

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

