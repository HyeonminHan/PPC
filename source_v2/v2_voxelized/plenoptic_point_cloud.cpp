#include "plenoptic_point_cloud.h"

//vector<PPC> make_sequenced_Plen_PC(
//	vector<Mat> color_imgs,
//	vector<Mat> depth_imgs,
//	int d_threshold,
//	int c_threshold) {
//
//	vector<PPC*> Plen_PC;
//	clock_t start, end;
//
//	start = clock();
//
//	PointCloud<PointXYZRGB>::Ptr firstPC;
//	if (version == 2.1) {
//		firstPC = make_PC(0, color_imgs[0], depth_imgs[0]);
//	}
//	else if (version == 2.2) {
//		firstPC = make_PC(0, color_imgs[0], depth_imgs[0]);
//	}
//
//	for (int point_idx = 0; point_idx < firstPC->points.size(); point_idx++) {
//		PPC* point_ppc;
//
//		if (version == 2.1 ) point_ppc = new PPC_v2_1();
//		else if (version == 2.2 ) point_ppc = new PPC_v2_2();
//
//
//		point_ppc->SetGeometry(firstPC->points[point_idx]);
//
//		if (version == 2.1 || version == 3.1) point_ppc->SetRefColor(firstPC->points[point_idx], 0);
//		else if (version == 2.2 || version == 3.2) point_ppc->SetColor(firstPC->points[point_idx], 0);  //point_ppc->SetColor(firstPC->points[point_idx], refView); 
//
//		Plen_PC.push_back(point_ppc);
//	}
//
//	for (int i = 1; i < total_num_cameras; i++) {
//		int cam = i;
//
//		double X;
//		double Y;
//		double Z;
//
//		int u;
//		int v;
//
//		double dist;
//		double w;
//
//		Vec3b d;
//		Vec3s d_s;
//
//		Mat color_img(_height, _width, CV_8UC3, Scalar(0));
//		Mat depth_img(_height, _width, CV_64F, -1);
//		Mat confirm_img(_height, _width, CV_32S, -1);
//
//		for (int point_idx = 0; point_idx < Plen_PC.size(); point_idx++) {
//			
//			float* geometry = Plen_PC[point_idx]->GetGeometry();
//			X = geometry[0];
//			Y = geometry[1];
//			Z = geometry[2];
//
//			w = projection_XYZ_2_UV(
//				m_CalibParams[cam].m_ProjMatrix,
//				X,
//				Y,
//				Z,
//				u,
//				v); //change1 -> ballet success
//
//			dist = find_point_dist(w, cam);
//
//			if ((u < 0) || (v < 0) || (u > _width - 1) || (v > _height - 1)) continue;
//
//			if (depth_img.at<double>(v, u) == -1)
//				depth_img.at<double>(v, u) = dist;
//			else
//			{
//				if (dist < depth_img.at<double>(v, u))
//					depth_img.at<double>(v, u) = dist;
//				else continue;
//			}
//			confirm_img.at<int>(v, u) = point_idx;
//		}
//		//2, height-1
//		for (int v = 0; v < _height; v++) {
//			for (int u = 0; u < _width; u++) {
//				Vec3b color = color_imgs[cam].at<Vec3b>(v, u);
//				
//				switch (data_mode) {
//					case 0:
//						d = depth_imgs[cam].at<Vec3b>(v, u);
//						Z = depth_level_2_Z(d[0]);
//						break;
//					case 1:
//					case 2:
//						d_s = depth_imgs[cam].at<Vec3s>(v, u);
//						Z = depth_level_2_Z_s(d_s[0]);
//						break;
//					case 3:
//						d_s = depth_imgs[cam].at<Vec3s>(v, u);
//						Z = depth_level_2_Z_s(d_s[0], cam);
//						break;
//					case 4:
//						Z = depth_level_2_Z_s_direct(depth_imgs[cam].at<ushort>(v, u));
//						break;
//				}
//
//				if (confirm_img.at<int>(v, u) == -1) {
//					double Z;
//
//					
//					double X = 0.0;
//					double Y = 0.0;
//
//					if (!data_mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y);
//					else Z = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z, &X, &Y);
//
//					PPC* point_ppc;
//					if (version == 2.1) point_ppc = new PPC_v2_1();
//					else if (version == 2.2) point_ppc = new PPC_v2_2();
//
//					float geo[3] = { X, Y, Z };
//
//					point_ppc->SetGeometry(geo);
//					if (version == 2.1) point_ppc->SetRefColor(color, cam);
//					else if (version == 2.2) point_ppc->SetColor(color, cam);
//
//					Plen_PC.push_back(point_ppc);
//
//					//cout << point_ppc.color[cam][0] << " " << point_ppc.color[cam][1] << " " << point_ppc.color[cam][2] << " ";
//				}
//				// ������ �� ���
//				else
//				{
//					int point_idx = confirm_img.at<int>(v, u);
//
//					float* geometry = Plen_PC[point_idx]->GetGeometry();
//					X = geometry[0];
//					Y = geometry[1];
//					Z = geometry[2];
//
//					int u_, v_;
//
//					if (!data_mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y); // �� �ڵ��� �ǹ̴�?? �ϴ� ���� 
//					else Z = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z, &X, &Y);
//
//					w = m_CalibParams[cam].m_ProjMatrix(2, 0) * X + m_CalibParams[cam].m_ProjMatrix(2, 1) * Y +
//						m_CalibParams[cam].m_ProjMatrix(2, 2) * Z + m_CalibParams[cam].m_ProjMatrix(2, 3);
//
//					double w_origin;
//					Vec3b d = depth_imgs[cam].at<Vec3b>(v, u);
//					double X_ = 0.0;
//					double Y_ = 0.0;
//					double Z_ = depth_level_2_Z(d[0]);
//
//					w_origin = m_CalibParams[cam].m_ProjMatrix(2, 0) * X_ + m_CalibParams[cam].m_ProjMatrix(2, 1) * Y_ +
//						m_CalibParams[cam].m_ProjMatrix(2, 2) * Z_ + m_CalibParams[cam].m_ProjMatrix(2, 3);
//
//					//Mat hsv_new(1, 1, CV_8UC3);
//					//hsv_new.at<Vec3b>(0, 0) = color;
//					//cvtColor(hsv_new, hsv_new, CV_BGR2HSV);
//
//					// ���԰� ��ħ.
//					int nearCamNum = 0;
//					for (int c = cam-1; c >= 0; c--) {
//						if (!Plen_PC[point_idx].CheckOcclusion(c)) {
//							nearCamNum = c;
//							break;
//						}
//					}
//
//					//Mat hsv_preexisted(1, 1, CV_8UC3);
//					//hsv_preexisted.at<Vec3b>(0, 0)[0] = Plen_PC[point_idx].color[occ_true_camnum][2];
//					//hsv_preexisted.at<Vec3b>(0, 0)[1] = Plen_PC[point_idx].color[occ_true_camnum][1];
//					//hsv_preexisted.at<Vec3b>(0, 0)[2] = Plen_PC[point_idx].color[occ_true_camnum][0];
//					//cvtColor(hsv_preexisted, hsv_preexisted, CV_BGR2HSV);
//
//					//short sub_H = hsv_new.at<Vec3b>(0, 0)[0] - hsv_preexisted.at<Vec3b>(0, 0)[0];
//					int sub_V = color[2] - Plen_PC[point_idx].GetColor(nearCamNum)[0];
//					int sub_U = color[1] - Plen_PC[point_idx].GetColor(nearCamNum)[1];
//
//					// depth thres, color thres ok
//					if (abs(find_point_dist(w, cam) - find_point_dist(w_origin, cam)) < d_threshold && abs(sub_U) <= c_threshold && abs(sub_V) <= c_threshold) {
//						Plen_PC[point_idx].SetColor(color, cam);
//					}
//					else {
//						if (!data_mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z_, &X_, &Y_);
//						else Z_ = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z_, &X_, &Y_);
//
//						PPC* point_ppc = new PPC_v2_1();
//
//						point_ppc->SetGeometry(pointclouds[0]->points[point_idx]);
//						point_ppc->SetRefColor(pointclouds[0]->points[point_idx], 0);
//
//						Plen_PC.push_back(*point_ppc);
//					}
//				}
//			}
//		}
//	}
//	end = clock();
//	//cout << "Plen_pc size 2 : " << Plen_PC.size() << endl;
//
//	/*vector<int> sum_true(8), sum_false(8);
//	for (int i = 0; i < Plen_PC.size(); i++) {
//	   for (int cam = 0; cam < total_num_cameras; cam++) {
//
//		  if (Plen_PC[i].occlusion_pattern[cam] == true)
//			 sum_true[cam]++;
//		  else sum_false[cam]++;
//	   }
//	}
//	for (int i = 0; i < 8; i++) {
//
//	   cout << "sum_true : " << sum_true[i] << endl;
//	   cout << "sum_false : " << sum_false[i] << endl;
//	}*/
//
//	//cout << "�ҿ� �ð� : " << (double)(end - start) / CLOCKS_PER_SEC << endl;
//
//	return Plen_PC;
//}

/////////////////////here///////////////////////
vector<PPC*> make_sequenced_Plen_PC(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	int colorspace,
	vector<int> camera_order)
{
	vector<PPC*> Plen_PC;
	int c_threshold = 7;

	PointCloud<PointXYZRGB>::Ptr firstPC;
	if (version == 2.1) {
		firstPC = make_PC(0, color_imgs[0], depth_imgs[0]);
	}
	else if (version == 2.2) {
		firstPC = make_PC(0, color_imgs[0], depth_imgs[0]);
	}
	//cout << 1 << endl;
	// 첫번째 view 
	for (int point_idx = 0; point_idx < firstPC->points.size(); point_idx++) {
		PPC* point_ppc;

		if(version==2.1) point_ppc = new PPC_v2_1();
		else if(version == 2.2) point_ppc = new PPC_v2_2();
		
		point_ppc->SetGeometry(firstPC->points[point_idx]);

		if(version == 2.1 || version == 3.1) point_ppc->SetRefColor(firstPC->points[point_idx], 0);
		else if(version == 2.2 || version == 3.2) point_ppc->SetColor(firstPC->points[point_idx], 0);  //point_ppc->SetColor(firstPC->points[point_idx], refView); 

		Plen_PC.push_back(point_ppc);
	}
	//cout << 2 << endl;


	for (int i = 1; i < total_num_cameras; i++) {
		int cam = i;// camera_order[i];

		int u;
		int v;

		double w;

		Mat color_img(_height, _width, CV_8UC3, Scalar(0));
		Mat confirm_img(_height, _width, CV_32S, -1);

		Vec3b d;
		Vec3s d_s;

		for (int point_idx = 0; point_idx < Plen_PC.size(); point_idx++) {
			float* geometry = Plen_PC[point_idx]->GetGeometry();

			double X = geometry[0];
			double Y = geometry[1];
			double Z = geometry[2];

			w = projection_XYZ_2_UV(
				m_CalibParams[cam].m_ProjMatrix,
				X,
				Y,
				Z,
				u,
				v); //change1 -> ballet success

			if ((u < 0) || (v < 0) || (u > _width - 1) || (v > _height - 1)) continue;

			confirm_img.at<int>(v, u) = point_idx;
		}
		//2, height-1
		for (int v = 0; v < _height; v++) {
			for (int u = 0; u < _width; u++) {
				Vec3b color = color_imgs[cam].at<Vec3b>(v, u);

				double X;
				double Y;
				double Z;

				switch (data_mode) {
				case 0:
					d = depth_imgs[cam].at<Vec3b>(v, u);
					Z = depth_level_2_Z(d[0]);
					break;
				case 1:
				case 2:
					d_s = depth_imgs[cam].at<Vec3s>(v, u);
					Z = depth_level_2_Z_s(d_s[0]);
					break;
				case 3:
					d_s = depth_imgs[cam].at<Vec3s>(v, u);
					Z = depth_level_2_Z_s(d_s[0], cam);
					break;
				case 4:
					Z = depth_level_2_Z_s_direct(depth_imgs[cam].at<ushort>(v, u));
					break;
				}

				if (!data_mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y);
				else Z = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z, &X, &Y);

				// 투영이 안됬을 때
				if (confirm_img.at<int>(v, u) == -1) {
					
					PPC* point_ppc;
					if (version == 2.1 ) point_ppc = new PPC_v2_1();
					else if (version == 2.2 ) point_ppc = new PPC_v2_2();

					float geo[3] = { X,Y,Z };
					point_ppc->SetGeometry(geo);
					if (version == 2.1) point_ppc->SetRefColor(color, cam);
					else if (version == 2.2) point_ppc->SetColor(color, cam);

					Plen_PC.push_back(point_ppc);
				}
				// 투영이 됐을 때
				else
				{
					int point_idx = confirm_img.at<int>(v, u);

					int nearCamNum = 0;
					for (int c = cam - 1; c >= 0; c--) {
						if (!Plen_PC[point_idx]->CheckOcclusion(c)) {
							nearCamNum = c;
							break;
						}
					}

					if (colorspace == 0) { //YUV
						int sub_V = color[2] - Plen_PC[point_idx]->GetColor(nearCamNum)[0];
						int sub_U = color[1] - Plen_PC[point_idx]->GetColor(nearCamNum)[1];

						float* geo = Plen_PC[point_idx]->GetGeometry();
						float sub_Z = Z - geo[2];

						// color thres
						if (abs(sub_U) < c_threshold && abs(sub_V) < c_threshold) {
							//if (abs(sub_U) < c_threshold && abs(sub_V) < c_threshold && sub_Z < (MaxZ - MinZ)*0.0001) {
							Plen_PC[point_idx]->SetColor(color, cam);
						}
						else {
							PPC* point_ppc;
							if (version == 2.1) point_ppc = new PPC_v2_1();
							else if (version == 2.2) point_ppc = new PPC_v2_2();

							float geo[3] = { X,Y,Z };
							point_ppc->SetGeometry(geo);
							if (version == 2.1) point_ppc->SetRefColor(color, cam);
							else if (version == 2.2) point_ppc->SetColor(color, cam);

							Plen_PC.push_back(point_ppc);
						}
					}
					else if (colorspace == 1) { //HSV

						Mat hsv_org(1, 1, CV_8UC3);
						hsv_org.at<Vec3b>(0, 0) = color;
						cvtColor(hsv_org, hsv_org, CV_YUV2BGR);
						cvtColor(hsv_org, hsv_org, CV_BGR2HSV);

						Mat hsv_ppc(1, 1, CV_8UC3);
						hsv_ppc.at<Vec3b>(0, 0)[0] = Plen_PC[point_idx]->GetColor(nearCamNum)[2];
						hsv_ppc.at<Vec3b>(0, 0)[1] = Plen_PC[point_idx]->GetColor(nearCamNum)[1];
						hsv_ppc.at<Vec3b>(0, 0)[2] = Plen_PC[point_idx]->GetColor(nearCamNum)[0];
						cvtColor(hsv_ppc, hsv_ppc, CV_YUV2BGR);
						cvtColor(hsv_ppc, hsv_ppc, CV_BGR2HSV);

						int sub_H = hsv_org.at<Vec3b>(0, 0)[0] - hsv_ppc.at<Vec3b>(0, 0)[0];
						int sub_S = hsv_org.at<Vec3b>(0, 0)[1] - hsv_ppc.at<Vec3b>(0, 0)[1];

						float* geo = Plen_PC[point_idx]->GetGeometry();
						float sub_Z = Z - geo[2];

						//cout << sub_H << " " << sub_S << endl;

						// color thres
						if (abs(sub_H) < c_threshold && abs(sub_S) < c_threshold) {
							//if (abs(sub_U) < c_threshold && abs(sub_V) < c_threshold && sub_Z < (MaxZ - MinZ)*0.0001) {
							Plen_PC[point_idx]->SetColor(color, cam);
						}
						else {
							PPC* point_ppc;
							if (version == 2.1 ) point_ppc = new PPC_v2_1();
							else if (version == 2.2 ) point_ppc = new PPC_v2_2();

							float geo[3] = { X,Y,Z };
							point_ppc->SetGeometry(geo);
							if (version == 2.1) point_ppc->SetRefColor(color, cam);
							else if (version == 2.2) point_ppc->SetColor(color, cam);

							Plen_PC.push_back(point_ppc);
						}

					}
					else if (colorspace == 2) { //RGB

						Mat bgr_org(1, 1, CV_8UC3);
						bgr_org.at<Vec3b>(0, 0) = color;
						cvtColor(bgr_org, bgr_org, CV_YUV2BGR);

						Mat bgr_ppc(1, 1, CV_8UC3);
						bgr_ppc.at<Vec3b>(0, 0)[0] = Plen_PC[point_idx]->GetColor(nearCamNum)[2];
						bgr_ppc.at<Vec3b>(0, 0)[1] = Plen_PC[point_idx]->GetColor(nearCamNum)[1];
						bgr_ppc.at<Vec3b>(0, 0)[2] = Plen_PC[point_idx]->GetColor(nearCamNum)[0];
						cvtColor(bgr_ppc, bgr_ppc, CV_YUV2BGR);

						int sub_B = bgr_org.at<Vec3b>(0, 0)[0] - bgr_ppc.at<Vec3b>(0, 0)[0];
						int sub_G = bgr_org.at<Vec3b>(0, 0)[1] - bgr_ppc.at<Vec3b>(0, 0)[1];
						int sub_R = bgr_org.at<Vec3b>(0, 0)[2] - bgr_ppc.at<Vec3b>(0, 0)[2];

						float* geo = Plen_PC[point_idx]->GetGeometry();
						float sub_Z = Z - geo[2];

						// color thres
						if (abs(sub_B) < c_threshold && abs(sub_G) < c_threshold && abs(sub_R) < c_threshold) {
							//if (abs(sub_U) < c_threshold && abs(sub_V) < c_threshold && sub_Z < (MaxZ - MinZ)*0.0001) {
							Plen_PC[point_idx]->SetColor(color, cam);
						}
						else {
							PPC* point_ppc;
							if (version == 2.1 || version == 3.1) point_ppc = new PPC_v2_1();
							else if (version == 2.2 || version == 3.2) point_ppc = new PPC_v2_2();

							float geo[3] = { X,Y,Z };
							point_ppc->SetGeometry(geo);
							if (version == 2.1 || version == 3.1) point_ppc->SetRefColor(color, cam);
							else if (version == 2.2 || version == 3.2) point_ppc->SetColor(color, cam);

							Plen_PC.push_back(point_ppc);
						}
					}
				}
			}
		}
	}
	//cout << 3 << endl;

	
	Plen_PC.shrink_to_fit();

	return Plen_PC;
}

vector<PPC*> make_voxelized_Plen_PC(
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	int voxel_div_num) {

	vector<float> min(3), max(3);

	find_min_max(pointclouds, min, max);

	float x_size = (max[0] - min[0]);
	float y_size = (max[1] - min[1]);
	float z_size = (max[2] - min[2]);

	unsigned long long x_voxel_num = voxel_div_num;
	unsigned long long y_voxel_num = voxel_div_num;
	unsigned long long z_voxel_num = voxel_div_num;

	unsigned long long total_voxel_num = x_voxel_num * y_voxel_num * z_voxel_num;

	float x_stride = x_size / x_voxel_num;
	float y_stride = y_size / y_voxel_num;
	float z_stride = z_size / z_voxel_num;

	// color cube, 카메라 별로 색을 쌓아논거
	map< unsigned long long, vector<vector<Vec3b>> > color_map;
	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
		for (unsigned long long point_idx = 0; point_idx < pointclouds[cam_num]->points.size(); point_idx++) {
			unsigned long long  x_voxel_index = (int)floor((pointclouds[cam_num]->points[point_idx].x - min[0]) / x_size * ((float)x_voxel_num - 1));
			unsigned long long  y_voxel_index = (int)floor((pointclouds[cam_num]->points[point_idx].y - min[1]) / y_size * ((float)y_voxel_num - 1));
			unsigned long long  z_voxel_index = (int)floor((pointclouds[cam_num]->points[point_idx].z - min[2]) / z_size * ((float)z_voxel_num - 1));
			unsigned long long cube_index = x_voxel_index * (y_voxel_num * z_voxel_num) + y_voxel_index * z_voxel_num + z_voxel_index;

			// find(key를 찾는거)를 했을때 return이 end()이면 해당 key는 map에 없음
			// 없으면(기존에 할당되지 않은 큐브 인덱스) 새로 추가
			if (color_map.find(cube_index) == color_map.end()) {
				vector<vector<Vec3b>> temp_color(total_num_cameras);
				temp_color[cam_num].push_back(Vec3b(pointclouds[cam_num]->points[point_idx].r,
					pointclouds[cam_num]->points[point_idx].g,
					pointclouds[cam_num]->points[point_idx].b));
				color_map.insert(color_map.end(), pair< unsigned long long, vector<vector<Vec3b>> >(cube_index, temp_color));
			}
			// 기존 key 중에 존재하면, 해당 카메라에 색을 쌓음.
			else {
				color_map[cube_index][cam_num].push_back(Vec3b(pointclouds[cam_num]->points[point_idx].r,
					pointclouds[cam_num]->points[point_idx].g,
					pointclouds[cam_num]->points[point_idx].b));
			}
		}
	}

	vector<PPC*> Plen_PC;

	// map의 iterator 코드
	// 색의 평균을 구하는 부분
	// first = key, second = value

	for (map<unsigned long long, vector<vector<Vec3b>> >::iterator it = color_map.begin(); it != color_map.end(); ++it) {
		PPC* temp_PP;
		if (version == 2.1) temp_PP = new PPC_v2_1();
		else if (version == 2.2) temp_PP = new PPC_v2_2();

		int inter_r = 0, inter_g = 0, inter_b = 0;
		int inter_cnt = 0;

		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
			if (it->second[cam_num].size() > 0) {
				int avg_r = 0, avg_g = 0, avg_b = 0;
				for (unsigned long long i = 0; i < it->second[cam_num].size(); i++) {
					avg_r += int(it->second[cam_num][i][0]);
					avg_g += int(it->second[cam_num][i][1]);
					avg_b += int(it->second[cam_num][i][2]);
				}

				avg_r /= it->second[cam_num].size();
				avg_g /= it->second[cam_num].size();
				avg_b /= it->second[cam_num].size();

				if (version == 2.1) {
					if (inter_cnt == 0) {
						temp_PP->SetRefColor(Vec3b((uchar)avg_b, (uchar)avg_g, (uchar)avg_r), cam_num);
					}
					else {
						temp_PP->SetColor((uchar)avg_r, (uchar)avg_g, (uchar)avg_b, cam_num);
					}
				}

				//temp_PP.color[cam_num][0] = (short)avg_r;
				//temp_PP.color[cam_num][1] = (short)avg_g;
				//temp_PP.color[cam_num][2] = (short)avg_b;

				// 색 보간
				inter_r += avg_r;
				inter_g += avg_g;
				inter_b += avg_b;
				inter_cnt++;
			}
		}
		inter_r /= inter_cnt;
		inter_g /= inter_cnt;
		inter_b /= inter_cnt;

		// 보간 색 할당
		//for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
		//	if (it->second[cam_num].size() == 0) {
		//		//it->second[cam_num].push_back(Vec3b((uchar)inter_r, (uchar)inter_g, (uchar)inter_b));
		//		temp_PP.color[cam_num][0] = inter_r;
		//		temp_PP.color[cam_num][1] = inter_g;
		//		temp_PP.color[cam_num][2] = inter_b;
		//		temp_PP.occlusion_pattern[cam_num] = false;
		//	}

		//	else
		//		temp_PP.occlusion_pattern[cam_num] = true;
		//}

		int x_idx = it->first / (y_voxel_num * z_voxel_num);
		int y_idx = (it->first % (y_voxel_num * z_voxel_num)) / z_voxel_num;
		int z_idx = (it->first % (y_voxel_num * z_voxel_num)) % z_voxel_num;

		float geo[3] = { (float(x_idx) / (x_voxel_num - 1) * x_size) + min[0] + (x_stride / 2) ,
						(float(y_idx) / (y_voxel_num - 1) * y_size) + min[1] + (y_stride / 2) ,
						(float(z_idx) / (z_voxel_num - 1) * z_size) + min[2] + (z_stride / 2) };
		temp_PP->SetGeometry(geo);

		Plen_PC.push_back(temp_PP);
	}

	color_map.clear();

	return Plen_PC;
}

vector<PPC*> make_voxelized_Plen_PC(
	vector<PPC*> PPC_vec,
	int voxel_div_num) {

	vector<float> min(3), max(3);

	PointCloud<PointXYZRGB>::Ptr pointcloud_ppc(new PointCloud<PointXYZRGB>);

	for (int i = 0; i < PPC_vec.size(); i++) {
		PointXYZRGB p;
		float* geo = PPC_vec[i]->GetGeometry();
		p.x = geo[0];
		p.y = geo[1];
		p.z = geo[2];
		p.r = 0;
		p.g = 0;
		p.b = 0;

		pointcloud_ppc->points.push_back(p);
	}
	cout << "voxel 0" << endl;

	find_min_max(pointcloud_ppc, min, max);
	cout << "voxel 1" << endl;


	float x_size = (max[0] - min[0]);
	float y_size = (max[1] - min[1]);
	float z_size = (max[2] - min[2]);

	unsigned long long x_voxel_num = voxel_div_num;
	unsigned long long y_voxel_num = voxel_div_num;
	unsigned long long z_voxel_num = voxel_div_num;

	unsigned long long total_voxel_num = x_voxel_num * y_voxel_num * z_voxel_num;

	float x_stride = x_size / x_voxel_num;
	float y_stride = y_size / y_voxel_num;
	float z_stride = z_size / z_voxel_num;

	// color cube, 카메라 별로 색을 쌓아논거
	map<unsigned long long, vector<vector<Vec3b>>> color_map;
	cout << "voxel 2" << endl;

	int count1, count2 = 0;
	int hhcount = 0; 

	//for (unsigned long long point_idx = 0; point_idx < PPC_vec.size() ; point_idx++) {
	while(PPC_vec.begin() != PPC_vec.end()){
		if(hhcount % 1000 == 0) cout << "number :" << PPC_vec.size() << endl;
		unsigned long long point_idx = PPC_vec.size() - 1;
		float* geo = PPC_vec[point_idx]->GetGeometry();
		
		unsigned long long x_voxel_index = (int)floor((geo[0] - min[0]) / x_size * ((float)x_voxel_num - 1));
		unsigned long long y_voxel_index = (int)floor((geo[1] - min[1]) / y_size * ((float)y_voxel_num - 1));
		unsigned long long z_voxel_index = (int)floor((geo[2] - min[2]) / z_size * ((float)z_voxel_num - 1));
		unsigned long long cube_index = x_voxel_index * (y_voxel_num * z_voxel_num) + y_voxel_index * z_voxel_num + z_voxel_index;
		
		// find(key를 찾는거)를 했을때 return이 end()이면 해당 key는 map에 없음
		// 없으면(기존에 할당되지 않은 큐브 인덱스) 새로 추가
		if (color_map.find(cube_index) == color_map.end()) {
			vector<vector<Vec3b>> temp_color(total_num_cameras);
			for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
				if (!PPC_vec[point_idx]->CheckOcclusion(cam_num)) {
					Vec3b temp_col = PPC_vec[point_idx]->GetColor(cam_num);
					temp_color[cam_num].push_back(Vec3b(temp_col[0], temp_col[1], temp_col[2]));
				}
			}
			color_map.insert(color_map.end(), pair<unsigned long long, vector<vector<Vec3b>>>(cube_index, temp_color));
			count1++;
		}
		// 기존 key 중에 존재하면, 해당 카메라에 색을 쌓음.
		else {
			for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
				if (!PPC_vec[point_idx]->CheckOcclusion(cam_num)) {
					Vec3b temp_col = PPC_vec[point_idx]->GetColor(cam_num);
					color_map[cube_index][cam_num].push_back(Vec3b(temp_col[0], temp_col[1], temp_col[2]));
				}
			}
			count2++;
		}
		PPC_vec.pop_back();
		if(hhcount % 1000 ==0)
			PPC_vec.shrink_to_fit();
		hhcount++;
	}
	cout << "voxel 3" << endl;
	cout << "map size : " << color_map.size() << endl;

	vector<PPC*> Plen_PC;

	// map의 iterator 코드
	// 색의 평균을 구하는 부분
	// first = key, second = value
	for (map<unsigned long long, vector<vector<Vec3b>>>::iterator it = color_map.begin(); it != color_map.end(); ++it) {
		PPC* temp_PP;
		if (version == 2.1) temp_PP = new PPC_v2_1();
		else if (version == 2.2) temp_PP = new PPC_v2_2();

		int inter_r = 0, inter_g = 0, inter_b = 0;
		int inter_cnt = 0;

		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
			if (it->second[cam_num].size() > 0) {
				int avg_r = 0, avg_g = 0, avg_b = 0;

				for (unsigned long long i = 0; i < it->second[cam_num].size(); i++) {
					avg_r += int(it->second[cam_num][i][0]);
					avg_g += int(it->second[cam_num][i][1]);
					avg_b += int(it->second[cam_num][i][2]);
				}

				avg_r /= it->second[cam_num].size();
				avg_g /= it->second[cam_num].size();
				avg_b /= it->second[cam_num].size();

				if (version == 2.1) {
					if (inter_cnt == 0) {
						temp_PP->SetRefColor(Vec3b((uchar)avg_b, (uchar)avg_g, (uchar)avg_r), cam_num);
					}
					else {
						temp_PP->SetColor((uchar)avg_r, (uchar)avg_g, (uchar)avg_b, cam_num);
					}
				}
				else if (version == 2.2) {
					temp_PP->SetColor(Vec3b((uchar)avg_b, (uchar)avg_g, (uchar)avg_r), cam_num);
				}

				//temp_PP.color[cam_num][0] = (short)avg_r;
				//temp_PP.color[cam_num][1] = (short)avg_g;
				//temp_PP.color[cam_num][2] = (short)avg_b;

				// 색 보간
				/*inter_r += avg_r;
				inter_g += avg_g;
				inter_b += avg_b;
				inter_cnt++;*/

			}
		}
		cout << "voxel 4" << endl;

		/*inter_r /= inter_cnt;
		inter_g /= inter_cnt;
		inter_b /= inter_cnt;*/

		// 보간 색 할당
		//for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
		//	if (it->second[cam_num].size() == 0) {
		//		//it->second[cam_num].push_back(Vec3b((uchar)inter_r, (uchar)inter_g, (uchar)inter_b));

		//		temp_PP.color[cam_num][0] = inter_r;
		//		temp_PP.color[cam_num][1] = inter_g;
		//		temp_PP.color[cam_num][2] = inter_b;
		//		temp_PP.occlusion_pattern[cam_num] = false;
		//	}
		//	else
		//		temp_PP.occlusion_pattern[cam_num] = true;
		//}

		int x_idx = it->first / (y_voxel_num * z_voxel_num);
		int y_idx = (it->first % (y_voxel_num * z_voxel_num)) / z_voxel_num;
		int z_idx = (it->first % (y_voxel_num * z_voxel_num)) % z_voxel_num;

		float geo[3] = { (float(x_idx) / (x_voxel_num - 1) * x_size) + min[0] + (x_stride / 2) ,
						(float(y_idx) / (y_voxel_num - 1) * y_size) + min[1] + (y_stride / 2) ,
						(float(z_idx) / (z_voxel_num - 1) * z_size) + min[2] + (z_stride / 2) };

		temp_PP->SetGeometry(geo);
		//temp_PP.geometry[0] = (float(x_idx) / (x_voxel_num - 1) * x_size) + min[0] + (x_stride / 2);
		//temp_PP.geometry[1] = (float(y_idx) / (y_voxel_num - 1) * y_size) + min[1] + (y_stride / 2);
		//temp_PP.geometry[2] = (float(z_idx) / (z_voxel_num - 1) * z_size) + min[2] + (z_stride / 2);

		Plen_PC.push_back(temp_PP);
	}

	color_map.clear();

	cout << " Voxelized done ... " << endl;

	return Plen_PC;
}

vector<PPC*> make_voxelized_Plen_PC2(
	vector<PPC*> PPC_vec,
	int voxel_div_num) {

	vector<float> min(3), max(3);
	PointCloud<PointXYZRGB>::Ptr pointcloud_ppc(new PointCloud<PointXYZRGB>);

	for (int i = 0; i < PPC_vec.size(); i++) {
		PointXYZRGB p;
		float* geo = PPC_vec[i]->GetGeometry();
		p.x = geo[0];
		p.y = geo[1];
		p.z = geo[2];
		p.r = 0;
		p.g = 0;
		p.b = 0;

		pointcloud_ppc->points.push_back(p);
	}

	find_min_max(pointcloud_ppc, min, max);

	float x_size = (max[0] - min[0]);
	float y_size = (max[1] - min[1]);
	float z_size = (max[2] - min[2]);

	cout << "x_size : " << x_size << " y_size : " << y_size << " z_size : " << z_size << endl;
	cout << " 11 " << endl;
	unsigned long long x_voxel_num = voxel_div_num;
	unsigned long long y_voxel_num = voxel_div_num;
	unsigned long long z_voxel_num = voxel_div_num;
	cout << " 22 " << endl;

	unsigned long long total_voxel_num = x_voxel_num * y_voxel_num * z_voxel_num;
	cout << " 33 " << endl;

	float x_stride = x_size / x_voxel_num;
	float y_stride = y_size / y_voxel_num;
	float z_stride = z_size / z_voxel_num;
	cout << " 44 " << endl;

	//vector<int> numberOfcolor_inVoxels(total_voxel_num, 0);
	//map<unsigned long long, PPC*> ppc_in_voxels_map;

	for (int point_idx = 0; point_idx < PPC_vec.size(); point_idx++) {

		float* geo = PPC_vec[point_idx]->GetGeometry();
		unsigned long long x_voxel_index = (int)floor((geo[0] - min[0]) / x_size * ((float)x_voxel_num - 1));
		unsigned long long y_voxel_index = (int)floor((geo[1] - min[1]) / y_size * ((float)y_voxel_num - 1));
		unsigned long long z_voxel_index = (int)floor((geo[2] - min[2]) / z_size * ((float)z_voxel_num - 1));

		unsigned long long cube_index = x_voxel_index * (y_voxel_num * z_voxel_num) + y_voxel_index * z_voxel_num + z_voxel_index;

		float geo2[3] = {   (float(x_voxel_index) / (x_voxel_num - 1) * x_size) + min[0] + (x_stride / 2) ,
							(float(y_voxel_index) / (y_voxel_num - 1) * y_size) + min[1] + (y_stride / 2) ,
							(float(z_voxel_index) / (z_voxel_num - 1) * z_size) + min[2] + (z_stride / 2) };

		PPC_vec[point_idx]->SetGeometry(geo2);

		/*if (numberOfcolor_inVoxels[cube_index]!=0) {
			for (int cam = 0; cam < total_num_cameras; cam++) {

				Vec3b color_cur = PPC_vec[point_idx]->GetColor(cam);

				if (PPC_vec[point_idx]->CheckOcclusion(cam)) continue;
				else {
					int numColor = numberOfcolor_inVoxels[cube_index];
					Vec3b color_prev = ppc_in_voxels_map[cube_index]->GetColor(cam);

					color_prev[0] = color_prev[0] * numColor / (numColor + 1) + color_cur[0] / (numColor + 1);
					color_prev[1] = color_prev[1] * numColor / (numColor + 1) + color_cur[1] / (numColor + 1);
					color_prev[2] = color_prev[2] * numColor / (numColor + 1) + color_cur[2] / (numColor + 1);

					ppc_in_voxels_map[cube_index]->SetColor(color_prev, cam);
				}
			}
		}
		else {
			numberOfcolor_inVoxels[cube_index]++;

			float geo2[3] = { (float(x_voxel_index) / (x_voxel_num - 1) * x_size) + min[0] + (x_stride / 2) ,
							(float(y_voxel_index) / (y_voxel_num - 1) * y_size) + min[1] + (y_stride / 2) ,
							(float(z_voxel_index) / (z_voxel_num - 1) * z_size) + min[2] + (z_stride / 2) };
			PPC_vec[point_idx]->SetGeometry(geo2);

			ppc_in_voxels_map.insert(ppc_in_voxels_map.end(), pair<unsigned long long, PPC*>(cube_index, PPC_vec[point_idx]));

		}
		*/

	}
	cout << " 55 " << endl;

	//PPC_vec.clear();
	//for (auto it = ppc_in_voxels_map.begin(); it != ppc_in_voxels_map.end(); it++) {
	//	PPC_vec.push_back(it->second);
	//}

	return PPC_vec;
}

vector<PointCloud<PointXYZRGB>::Ptr> make_all_PC(
	vector<Mat> color_imgs, 
	vector<Mat> depth_imgs) {

	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds;
	for (int cam = 0; cam < total_num_cameras; cam++) {
		PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);
		pointcloud = make_PC(cam, color_imgs[cam], depth_imgs[cam]);

		pointclouds.push_back(pointcloud);
	}

	return pointclouds;
}

void make_proj_img_vec_ppc(
	vector<PPC*> PPC,
	vector<Mat> &proj_img_vec,
	vector<PointCloud<PointXYZRGB>::Ptr> &pointclouds,
	int nNeighbor)
{
	for (int i = 0; i < total_num_cameras; i++) {
		pointclouds[i] = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);
	}

	PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);
	cout << "before backprojection1 .." << endl;

	Mat dist_img(_height, _width, CV_64F, -1);
	vector<Mat> dist_imgs(total_num_cameras, dist_img);


	//PC 8�� ����
	PointXYZRGB temp;
	//for (int i = 0; i < PPC.size(); i++)
	//{
	//	if( i % 100 ==0) 
	//		cout << "PPC num : " << i << endl;

	//	for (int j = 0; j < total_num_cameras; j++)
	//	{
	//		if (!PPC[i]->CheckOcclusion(j)) {
	//			//geometry
	//			float* geo = PPC[i]->GetGeometry();
	//			temp.x = geo[0];
	//			temp.y = geo[1];
	//			temp.z = geo[2];

	//			//color
	//			Vec3b color = PPC[i]->GetColor(j);
	//			temp.r = color[0];
	//			temp.g = color[1];
	//			temp.b = color[2];

	//			//projection_bypoint(temp, j, proj_img_vec[j], dist_imgs[j]);
	//			//pointclouds[j]->points.push_back(temp);
	//			pointcloud->points.push_back(temp);
	//		}
	//	}
	//	//back_projection(pointcloud, i, color_img, nNeighbor);

	//}

	for (int j = 0; j < total_num_cameras; j++)
	{
		cout << "cam_num :" << j << endl;
		Mat color_img(_height, _width, CV_8UC3, Scalar(0));
		Mat depth_value_img(_height, _width, CV_64F, -1);

		for (int i = 0; i < PPC.size(); i++)
		{
			if (!PPC[i]->CheckOcclusion(j)) {
				//geometry
				float* geo = PPC[i]->GetGeometry();
				temp.x = geo[0];
				temp.y = geo[1];
				temp.z = geo[2];

				//color
				Vec3b color = PPC[i]->GetColor(j);
				temp.r = color[0];
				temp.g = color[1];
				temp.b = color[2];

				//projection_bypoint(temp, j, proj_img_vec[j], dist_imgs[j]);
				//pointclouds[j]->points.push_back(temp);
				pointcloud->points.push_back(temp);
			}
		}
		back_projection(pointcloud, j, color_img, nNeighbor);
		proj_img_vec[j] = color_img;
		pointcloud->points.clear();
	}






	cout << "before backprojection2 .." << endl;
	//for (int i = 0; i < total_num_cameras; i++)
	//{
	//	Mat color_img(_height, _width, CV_8UC3, Scalar(0));
	//	Mat depth_value_img(_height, _width, CV_64F, -1);
	//	//back_projection(pointclouds[i], i, color_img, depth_value_img);
	//	//projection(pointclouds[i], i, color_img, depth_value_img);
	//	back_projection(pointclouds[i], i, color_img, nNeighbor);

	//	proj_img_vec[i] = color_img;
	//}

}

void projection_PPC_with_hole_filling(
	vector<PPC*> Plen_PC,
	vector<Mat> &projection_imgs,
	vector<Mat> &filled_imgs,
	vector<PointCloud<PointXYZRGB>::Ptr> &pointclouds_,
	int nNeighbor,
	int window_size)
{
	//make_proj_img_vec_by_seq(Plen_PC, projection_imgs, depth_value_imgs);

	make_proj_img_vec_ppc(Plen_PC, projection_imgs, pointclouds_, nNeighbor);

	hole_filling_PPC(projection_imgs, filled_imgs, window_size);
}

void hole_filling_PPC(
	vector<Mat> proj_imgs,
	vector<Mat> &filled_imgs,
	int window_size)
{
	Mat hole_image;

	for (int num = 0; num < total_num_cameras; num++) {
		hole_image = find_hole_PPC(proj_imgs[num]);
		filled_imgs[num] = make_filled_image_PPC(proj_imgs[num], hole_image, window_size);
	}
	cout << 4 << endl;

}

Mat find_hole_PPC(Mat projection_img)
{
	Mat hole_image(_height, _width, CV_8U, 1);
	for (int rownum = 0; rownum < _height; rownum++)
	{
		for (int colnum = 0; colnum < _width; colnum++)
		{
			if (projection_img.at<Vec3b>(rownum, colnum) == Vec3b(0, 0, 0))
				hole_image.at<uchar>(rownum, colnum) = 0;
		}
	}
	return hole_image;
}

Mat make_filled_image_PPC(Mat colorimg, Mat hole_image, int window_size)
{
	Mat filled_image(_height, _width, CV_8UC3, Scalar::all(0));

	for (int rownum = 0; rownum < _height; rownum++)
	{
		for (int colnum = 0; colnum < _width; colnum++)
		{
			bool is_not_hole = hole_image.at<uchar>(rownum, colnum);

			if (is_not_hole)
			{
				filled_image.at<Vec3b>(rownum, colnum)[0] = colorimg.at<Vec3b>(rownum, colnum)[0];
				filled_image.at<Vec3b>(rownum, colnum)[1] = colorimg.at<Vec3b>(rownum, colnum)[1];
				filled_image.at<Vec3b>(rownum, colnum)[2] = colorimg.at<Vec3b>(rownum, colnum)[2];
				continue;
			}
			else
			{
				vector<ushort> pixel_sum(3);
				int pixel_count = 0;

				for (int h = rownum - window_size; h <= rownum + window_size; h++)
				{
					for (int w = colnum - window_size; w <= colnum + window_size; w++)
					{
						if (h < 0 || w < 0 || h >= _height || w >= _width) continue;
						else if (hole_image.at<uchar>(h, w) == 0) continue;
						else
						{
							pixel_sum[0] += colorimg.at<Vec3b>(h, w)[0];
							pixel_sum[1] += colorimg.at<Vec3b>(h, w)[1];
							pixel_sum[2] += colorimg.at<Vec3b>(h, w)[2];
							pixel_count++;
						}
					}
				}

				if (pixel_count == 0)
				{
					filled_image.at<Vec3b>(rownum, colnum)[0] = 0;
					filled_image.at<Vec3b>(rownum, colnum)[1] = 0;
					filled_image.at<Vec3b>(rownum, colnum)[2] = 0;
				}
				else
				{
					filled_image.at<Vec3b>(rownum, colnum)[0] = uchar(pixel_sum[0] / pixel_count);
					filled_image.at<Vec3b>(rownum, colnum)[1] = uchar(pixel_sum[1] / pixel_count);
					filled_image.at<Vec3b>(rownum, colnum)[2] = uchar(pixel_sum[2] / pixel_count);
				}
			}
		}
	}
	return filled_image;
}

void save_ppc(vector<PPC*> ppc, string filename) {

	ofstream fout(filename, ios::binary);

	for (vector<PPC*>::iterator vit = ppc.begin(), vend = ppc.end(); vit != vend; vit++){
		float* geo = new float(3);
		geo = (*vit)->GetGeometry();
		fout.write((char*)&geo[0], sizeof(float));
		fout.write((char*)&geo[1], sizeof(float));
		fout.write((char*)&geo[2], sizeof(float));

		if (version == 2.1) {
			uchar refV = (*vit)->GetrefV();
			uchar refU = (*vit)->GetrefU();
			fout.write((char*)&refV, sizeof(uchar));
			fout.write((char*)&refU, sizeof(uchar));

			vector<uchar> VU = (*vit)->GetVU();
			vector<uchar> Y = (*vit)->GetY();

			for (int i = 0; i < total_num_cameras; i++) {
				fout.write((char*)&VU[i], sizeof(uchar));
			}
			for (int i = 0; i < total_num_cameras; i++) {
				fout.write((char*)&Y[i], sizeof(uchar));
			}
		}
		else if (version == 2.2) {
			uchar avrV = (*vit)->GetV();
			uchar avrU = (*vit)->GetU();
			fout.write((char*)&avrV, sizeof(uchar));
			fout.write((char*)&avrU, sizeof(uchar));

			vector<uchar> Y = (*vit)->GetY();
			vector<bool> occ = (*vit)->GetOcclusion();
		
			fout.write((char*)&Y[0], total_num_cameras * sizeof(uchar));
			for (int i = 0; i < total_num_cameras; i++) {
				char* temp;
				if (i % 8 == 0) {
					temp = new char;
					*temp = occ[i];
				}
				else if (i % 8 == 7 ) {
					*temp <<= 1;
					*temp |= occ[i];
					fout.write((char*)temp, sizeof(char));
					delete temp;
				}
				else if (i == total_num_cameras - 1) {
					*temp <<= 1;
					*temp |= occ[i];
					*temp <<= 8 - (total_num_cameras % 8);
					fout.write((char*)temp, sizeof(char));
					delete temp;
				}
				else {
					*temp <<= 1;
					*temp |= occ[i];
				}
				//cout << bitset<8>(*temp) << endl;
			}
		}
		
	}

	fout.close();
	cout << "save pcc done..." << endl;
}

vector<PPC*> load_ppc(string filename) {
	cout << "before load_ppc ..." << endl;
	vector<PPC*> vec_ppc;
	ifstream fin(filename, ios::binary);

	while (!fin.eof()) {
		PPC* pc;
		if (version == 2.1) pc = new PPC_v2_1;
		else if (version == 2.2) pc = new PPC_v2_2;

		float* geo = (float*)malloc(3 * sizeof(float));
		fin.read((char*)(&geo[0]), sizeof(float));
		fin.read((char*)(&geo[1]), sizeof(float));
		fin.read((char*)(&geo[2]), sizeof(float));
		pc->SetGeometry(geo);

		if (version == 2.1) {
			uchar refV, refU;
			fin.read((char*)(&refV), sizeof(uchar));
			fin.read((char*)(&refU), sizeof(uchar));

			vector<uchar> VU(total_num_cameras), Y(total_num_cameras);
			for (int i = 0; i < total_num_cameras; i++) {
				fin.read((char*)&VU[i], sizeof(uchar));
			}
			for (int i = 0; i < total_num_cameras; i++) {
				fin.read((char*)&Y[i], sizeof(uchar));
			}
			pc->SetColor(refV, refU, VU, Y);
		}
		else if (version == 2.2) {
			uchar avrV, avrU;
			fin.read((char*)(&avrV), sizeof(uchar));
			fin.read((char*)(&avrU), sizeof(uchar));

			vector<uchar> Y(total_num_cameras);
			vector<bool> occlusion(total_num_cameras);

			for (int i = 0; i < total_num_cameras; i++) {
				fin.read((char*)&Y[i], sizeof(uchar));
			}

			for (int i = 0; i < total_num_cameras; i++) {
				char* temp;
				if (i % 8 == 0) {
					temp = new char;
					fin.read((char*)temp, sizeof(char));
					unsigned char t = *temp & 128; //1000 0000
					if (t == 128) {
						occlusion[i] = true;
					}
					else occlusion[i] = false;
					*temp <<= 1;
				}
				else if (i % 8 == 7 || i == total_num_frames - 1) {
					unsigned char t = *temp & 128; //1000 0000
					if (t == 128) {
						occlusion[i] = true;
					}
					else occlusion[i] = false;
					delete temp;
				}
				else {
					unsigned char t = *temp & 128; //1000 0000
					if (t == 128) {
						occlusion[i] = true;
					}
					else occlusion[i] = false;
					*temp <<= 1;
				}
			}
			pc->SetColor(avrV, avrU, Y, occlusion);
		}
		
		vec_ppc.push_back(pc);
	}
	
	fin.close();
	cout << "load pcc done..." << endl;
	return vec_ppc;
}
