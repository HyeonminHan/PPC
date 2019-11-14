#include "plenoptic_point_cloud.h"

vector<PPC> make_sequenced_Plen_PC(vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs) {

	vector<PPC> Plen_PC;
	clock_t start, end;
	int c_threshold = 7;

	// { 0,1,2,3,4,5,6,7 }; //{ 4,3,5,2,6,1,7,0 };
	vector<int> camera_order;
	if (!mode) camera_order = { 4, 3, 5, 2, 6, 1, 7, 0 };
	else camera_order = { 5,4,6,3,7,2,8,1,9,0 };


	start = clock();

	//첫번째 뷰 PPC생성 
	for (int point_idx = 0; point_idx < pointclouds[camera_order[0]]->points.size(); point_idx++) {
		PPC point_ppc_1;
		if (!mode)
			point_ppc_1 = { {0, 0, 0}, {{SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX},
						 {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}},
						{false, false, false, false, false, false, false, false} };
		else
			point_ppc_1 = { {0, 0, 0}, {{SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX},
						 {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}},
						{false, false, false, false, false, false, false, false, false, false} };

		point_ppc_1.geometry[0] = pointclouds[camera_order[0]]->points[point_idx].x;
		point_ppc_1.geometry[1] = pointclouds[camera_order[0]]->points[point_idx].y;
		point_ppc_1.geometry[2] = pointclouds[camera_order[0]]->points[point_idx].z;
		point_ppc_1.color[camera_order[0]][0] = (short)pointclouds[camera_order[0]]->points[point_idx].r;
		point_ppc_1.color[camera_order[0]][1] = (short)pointclouds[camera_order[0]]->points[point_idx].g;
		point_ppc_1.color[camera_order[0]][2] = (short)pointclouds[camera_order[0]]->points[point_idx].b;
		point_ppc_1.occlusion_pattern[camera_order[0]] = true;

		Plen_PC.push_back(point_ppc_1);
	}
	cout << "Plen_pc size 1 : " << Plen_PC.size() << endl;
	for (int i = 1; i < total_num_cameras; i++) {

		int cam = camera_order[i];

		double X;
		double Y;
		double Z;

		int u;
		int v;

		double dist;
		double w;

		Mat color_img(_height, _width, CV_8UC3, Scalar(0));
		Mat depth_img(_height, _width, CV_64F, -1);
		Mat confirm_img(_height, _width, CV_32S, -1);

		for (int point_idx = 0; point_idx < Plen_PC.size(); point_idx++) {

			X = Plen_PC[point_idx].geometry[0];
			Y = Plen_PC[point_idx].geometry[1];
			Z = Plen_PC[point_idx].geometry[2];

			w = projection_XYZ_2_UV(
				m_CalibParams[cam].m_ProjMatrix,
				X,
				Y,
				Z,
				u,
				v); //change1 -> ballet success

			dist = find_point_dist(w, cam);

			if ((u < 0) || (v < 0) || (u > _width - 1) || (v > _height - 1)) continue;

			if (depth_img.at<double>(v, u) == -1)
				depth_img.at<double>(v, u) = dist;
			else
			{
				if (dist < depth_img.at<double>(v, u))
					depth_img.at<double>(v, u) = dist;
				else continue;
			}
			confirm_img.at<int>(v, u) = point_idx;
		}
		//2, height-1
		for (int v = 0; v < _height; v++) {
			for (int u = 0; u < _width; u++) {

				Vec3b color = color_imgs[cam].at<Vec3b>(v, u);
				if (confirm_img.at<int>(v, u) == -1) {
					double Z;

					if (!mode) {
						Vec3b d = depth_imgs[cam].at<Vec3b>(v, u);
						Z = depth_level_2_Z(d[0]);
					}
					else {
						Vec3s d_s = depth_img.at<Vec3s>(v, u);
						Z = depth_level_2_Z_s(d_s[0]);

					}
					double X = 0.0;
					double Y = 0.0;

					if (!mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y);
					else Z = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z, &X, &Y);

					PPC point_ppc_2;
					if (!mode)
						point_ppc_2 = { {0, 0, 0}, {{SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX},
									 {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}},
									{false, false, false, false, false, false, false, false} };
					else
						point_ppc_2 = { {0, 0, 0}, {{SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX},
									 {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}},
									{false, false, false, false, false, false, false, false, false, false} };

					point_ppc_2.geometry[0] = X;
					point_ppc_2.geometry[1] = Y;
					point_ppc_2.geometry[2] = Z;
					point_ppc_2.color[cam][0] = (short)color[2];
					point_ppc_2.color[cam][1] = (short)color[1];
					point_ppc_2.color[cam][2] = (short)color[0];
					point_ppc_2.occlusion_pattern[cam] = true;

					Plen_PC.push_back(point_ppc_2);
				}
				else
				{
					int point_idx = confirm_img.at<int>(v, u);

					X = Plen_PC[point_idx].geometry[0];
					Y = Plen_PC[point_idx].geometry[1];
					Z = Plen_PC[point_idx].geometry[2];

					int u_, v_;

					if (!mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y); // 이 코드의 의미는?? 일단 지움 
					else Z = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z, &X, &Y);

					w = m_CalibParams[cam].m_ProjMatrix(2, 0) * X + m_CalibParams[cam].m_ProjMatrix(2, 1) * Y +
						m_CalibParams[cam].m_ProjMatrix(2, 2) * Z + m_CalibParams[cam].m_ProjMatrix(2, 3);

					double w_origin;
					Vec3b d = depth_imgs[cam].at<Vec3b>(v, u);
					double X_ = 0.0;
					double Y_ = 0.0;
					double Z_ = depth_level_2_Z(d[0]);

					w_origin = m_CalibParams[cam].m_ProjMatrix(2, 0) * X_ + m_CalibParams[cam].m_ProjMatrix(2, 1) * Y_ +
						m_CalibParams[cam].m_ProjMatrix(2, 2) * Z_ + m_CalibParams[cam].m_ProjMatrix(2, 3);

					int hmsh;
					int ref = total_num_cameras / 2;
					int occ_true_camnum = -1;
					if (cam < ref) hmsh = 1;
					else hmsh = -1;

					for (int c = ref; c < total_num_cameras, c >= 0; c += hmsh) {
						if (Plen_PC[point_idx].occlusion_pattern[c] == true) {
							occ_true_camnum = c;
							break;
						}
					}

					short sub_Y = color_imgs[cam].at<Vec3b>(v, u)[0] - Plen_PC[point_idx].color[occ_true_camnum][2];
					short sub_U = color_imgs[cam].at<Vec3b>(v, u)[1] - Plen_PC[point_idx].color[occ_true_camnum][1];
					short sub_V = color_imgs[cam].at<Vec3b>(v, u)[2] - Plen_PC[point_idx].color[occ_true_camnum][0];

					if (abs(sub_U) < c_threshold && abs(sub_V) < c_threshold) {
						Plen_PC[point_idx].color[cam][0] = (short)color[2];
						Plen_PC[point_idx].color[cam][1] = (short)color[1];
						Plen_PC[point_idx].color[cam][2] = (short)color[0];
						Plen_PC[point_idx].occlusion_pattern[cam] = true;
					}
					else {
						if (!mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z_, &X_, &Y_);
						else Z_ = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z_, &X_, &Y_);

						PPC point_ppc_3;
						if (!mode)
							point_ppc_3 = { {0, 0, 0}, {{SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX},
										 {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}},
										{false, false, false, false, false, false, false, false} };
						else
							point_ppc_3 = { {0, 0, 0}, {{SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX},
										 {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}, {SHRT_MAX, SHRT_MAX, SHRT_MAX}},
										{false, false, false, false, false, false, false, false, false, false} };

						point_ppc_3.geometry[0] = X_;
						point_ppc_3.geometry[1] = Y_;
						point_ppc_3.geometry[2] = Z_;
						point_ppc_3.color[cam][0] = (short)color[2];
						point_ppc_3.color[cam][1] = (short)color[1];
						point_ppc_3.color[cam][2] = (short)color[0];
						point_ppc_3.occlusion_pattern[cam] = true;
						Plen_PC.push_back(point_ppc_3);
					}
				}
			}
		}
	}
	end = clock();
	cout << "Plen_pc size 2 : " << Plen_PC.size() << endl;

	cout << "소요 시간 : " << (double)(end - start) / CLOCKS_PER_SEC << endl;

	return Plen_PC;
}

vector<PPC> make_voxelized_Plen_PC(
	PointCloud<PointXYZRGB>::Ptr registered_PC,
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	int voxel_div_num) {

	vector<float> min(3), max(3);

	find_min_max(registered_PC, min, max);

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

	vector<PPC> Plen_PC;

	// map의 iterator 코드
	// 색의 평균을 구하는 부분
	// first = key, second = value

	for (map<unsigned long long, vector<vector<Vec3b>> >::iterator it = color_map.begin(); it != color_map.end(); ++it) {
		PPC temp_PP;
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

				temp_PP.color[cam_num][0] = (short)avg_r;
				temp_PP.color[cam_num][1] = (short)avg_g;
				temp_PP.color[cam_num][2] = (short)avg_b;

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
		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
			if (it->second[cam_num].size() == 0) {
				//it->second[cam_num].push_back(Vec3b((uchar)inter_r, (uchar)inter_g, (uchar)inter_b));
				temp_PP.color[cam_num][0] = inter_r;
				temp_PP.color[cam_num][1] = inter_g;
				temp_PP.color[cam_num][2] = inter_b;
				temp_PP.occlusion_pattern[cam_num] = false;
			}

			else
				temp_PP.occlusion_pattern[cam_num] = true;
		}

		int x_idx = it->first / (y_voxel_num * z_voxel_num);
		int y_idx = (it->first % (y_voxel_num * z_voxel_num)) / z_voxel_num;
		int z_idx = (it->first % (y_voxel_num * z_voxel_num)) % z_voxel_num;

		temp_PP.geometry[0] = (float(x_idx) / (x_voxel_num - 1) * x_size) + min[0] + (x_stride / 2);
		temp_PP.geometry[1] = (float(y_idx) / (y_voxel_num - 1) * y_size) + min[1] + (y_stride / 2);
		temp_PP.geometry[2] = (float(z_idx) / (z_voxel_num - 1) * z_size) + min[2] + (z_stride / 2);
		Plen_PC.push_back(temp_PP);
	}

	color_map.clear();

	return Plen_PC;
}

vector<PPC> make_voxelized_Plen_PC(
	vector<PPC> PPC_vec,
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	int voxel_div_num) {

	vector<float> min(3), max(3);

	PointCloud<PointXYZRGB>::Ptr pointcloud_ppc(new PointCloud<PointXYZRGB>);

	for (int i = 0; i < PPC_vec.size(); i++) {
		PointXYZRGB p;
		p.x = PPC_vec[i].geometry[0];
		p.y = PPC_vec[i].geometry[1];
		p.z = PPC_vec[i].geometry[2];
		p.r = 0;
		p.g = 0;
		p.b = 0;

		pointcloud_ppc->points.push_back(p);
	}

	find_min_max(pointcloud_ppc, min, max);

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

	for (unsigned long long point_idx = 0; point_idx < PPC_vec.size(); point_idx++) {
		unsigned long long  x_voxel_index = (int)floor((PPC_vec[point_idx].geometry[0] - min[0]) / x_size * ((float)x_voxel_num - 1));
		unsigned long long  y_voxel_index = (int)floor((PPC_vec[point_idx].geometry[1] - min[1]) / y_size * ((float)y_voxel_num - 1));
		unsigned long long  z_voxel_index = (int)floor((PPC_vec[point_idx].geometry[2] - min[2]) / z_size * ((float)z_voxel_num - 1));
		unsigned long long cube_index = x_voxel_index * (y_voxel_num * z_voxel_num) + y_voxel_index * z_voxel_num + z_voxel_index;

		// find(key를 찾는거)를 했을때 return이 end()이면 해당 key는 map에 없음
		// 없으면(기존에 할당되지 않은 큐브 인덱스) 새로 추가
		if (color_map.find(cube_index) == color_map.end()) {
			vector<vector<Vec3b>> temp_color(total_num_cameras);
			for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
				if (PPC_vec[point_idx].occlusion_pattern[cam_num] == true) {
					temp_color[cam_num].push_back(Vec3b(PPC_vec[point_idx].color[cam_num][0],
						PPC_vec[point_idx].color[cam_num][1],
						PPC_vec[point_idx].color[cam_num][2]));
				}
			}
			color_map.insert(color_map.end(), pair<unsigned long long, vector<vector<Vec3b>>>(cube_index, temp_color));
		}
		// 기존 key 중에 존재하면, 해당 카메라에 색을 쌓음.
		else {
			for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
				if (PPC_vec[point_idx].occlusion_pattern[cam_num] == true) {
					color_map[cube_index][cam_num].push_back(Vec3b(PPC_vec[point_idx].color[cam_num][0],
						PPC_vec[point_idx].color[cam_num][1],
						PPC_vec[point_idx].color[cam_num][2]));
				}
				//if (PPC_vec[point_idx].color[cam_num][0] != SHRT_MAX) {
			}
		}
	}

	vector<PPC> Plen_PC;

	// map의 iterator 코드
	// 색의 평균을 구하는 부분
	// first = key, second = value
	for (map<unsigned long long, vector<vector<Vec3b>> >::iterator it = color_map.begin(); it != color_map.end(); ++it) {
		PPC temp_PP;
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

				temp_PP.color[cam_num][0] = (short)avg_r;
				temp_PP.color[cam_num][1] = (short)avg_g;
				temp_PP.color[cam_num][2] = (short)avg_b;

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
		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
			if (it->second[cam_num].size() == 0) {
				//it->second[cam_num].push_back(Vec3b((uchar)inter_r, (uchar)inter_g, (uchar)inter_b));

				temp_PP.color[cam_num][0] = inter_r;
				temp_PP.color[cam_num][1] = inter_g;
				temp_PP.color[cam_num][2] = inter_b;
				temp_PP.occlusion_pattern[cam_num] = false;
			}
			else
				temp_PP.occlusion_pattern[cam_num] = true;
		}

		int x_idx = it->first / (y_voxel_num * z_voxel_num);
		int y_idx = (it->first % (y_voxel_num * z_voxel_num)) / z_voxel_num;
		int z_idx = (it->first % (y_voxel_num * z_voxel_num)) % z_voxel_num;

		temp_PP.geometry[0] = (float(x_idx) / (x_voxel_num - 1) * x_size) + min[0] + (x_stride / 2);
		temp_PP.geometry[1] = (float(y_idx) / (y_voxel_num - 1) * y_size) + min[1] + (y_stride / 2);
		temp_PP.geometry[2] = (float(z_idx) / (z_voxel_num - 1) * z_size) + min[2] + (z_stride / 2);

		Plen_PC.push_back(temp_PP);
	}

	color_map.clear();

	return Plen_PC;
}

map<unsigned long long, PPC> make_voxelized_Plen_PC2(
	vector<PPC> PPC_vec,
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	int voxel_div_num) {

	vector<float> min(3), max(3);

	PointCloud<PointXYZRGB>::Ptr pointcloud_ppc(new PointCloud<PointXYZRGB>);

	for (int i = 0; i < PPC_vec.size(); i++) {
		PointXYZRGB p;
		p.x = PPC_vec[i].geometry[0];
		p.y = PPC_vec[i].geometry[1];
		p.z = PPC_vec[i].geometry[2];
		p.r = 0;
		p.g = 0;
		p.b = 0;

		pointcloud_ppc->points.push_back(p);
	}

	find_min_max(pointcloud_ppc, min, max);

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

	for (unsigned long long point_idx = 0; point_idx < PPC_vec.size(); point_idx++) {
		unsigned long long  x_voxel_index = (int)floor((PPC_vec[point_idx].geometry[0] - min[0]) / x_size * ((float)x_voxel_num - 1));
		unsigned long long  y_voxel_index = (int)floor((PPC_vec[point_idx].geometry[1] - min[1]) / y_size * ((float)y_voxel_num - 1));
		unsigned long long  z_voxel_index = (int)floor((PPC_vec[point_idx].geometry[2] - min[2]) / z_size * ((float)z_voxel_num - 1));
		unsigned long long cube_index = x_voxel_index * (y_voxel_num * z_voxel_num) + y_voxel_index * z_voxel_num + z_voxel_index;

		// find(key를 찾는거)를 했을때 return이 end()이면 해당 key는 map에 없음
		// 없으면(기존에 할당되지 않은 큐브 인덱스) 새로 추가
		if (color_map.find(cube_index) == color_map.end()) {
			vector<vector<Vec3b>> temp_color(total_num_cameras);
			for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
				if (PPC_vec[point_idx].occlusion_pattern[cam_num] == true) {
					temp_color[cam_num].push_back(Vec3b(PPC_vec[point_idx].color[cam_num][0],
						PPC_vec[point_idx].color[cam_num][1],
						PPC_vec[point_idx].color[cam_num][2]));
				}
			}
			color_map.insert(color_map.end(), pair<unsigned long long, vector<vector<Vec3b>>>(cube_index, temp_color));
		}
		// 기존 key 중에 존재하면, 해당 카메라에 색을 쌓음.
		else {
			for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
				if (PPC_vec[point_idx].occlusion_pattern[cam_num] == true) {
					color_map[cube_index][cam_num].push_back(Vec3b(PPC_vec[point_idx].color[cam_num][0],
						PPC_vec[point_idx].color[cam_num][1],
						PPC_vec[point_idx].color[cam_num][2]));
				}
				//if (PPC_vec[point_idx].color[cam_num][0] != SHRT_MAX) {
			}
		}
	}

	map<unsigned long long, PPC> Plen_PC;

	// map의 iterator 코드
	// 색의 평균을 구하는 부분
	// first = key, second = value
	for (map<unsigned long long, vector<vector<Vec3b>> >::iterator it = color_map.begin(); it != color_map.end(); ++it) {
		PPC temp_PP;
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

				temp_PP.color[cam_num][0] = (short)avg_r;
				temp_PP.color[cam_num][1] = (short)avg_g;
				temp_PP.color[cam_num][2] = (short)avg_b;

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
		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
			if (it->second[cam_num].size() == 0) {
				//it->second[cam_num].push_back(Vec3b((uchar)inter_r, (uchar)inter_g, (uchar)inter_b));

				temp_PP.color[cam_num][0] = inter_r;
				temp_PP.color[cam_num][1] = inter_g;
				temp_PP.color[cam_num][2] = inter_b;
				temp_PP.occlusion_pattern[cam_num] = false;
			}

			else
				temp_PP.occlusion_pattern[cam_num] = true;
			//cout << "temp_PP COLORS :: " << temp_PP.color[cam_num][0] << "\t" << temp_PP.color[cam_num][1] << "\t" << temp_PP.color[cam_num][2] << endl;

		}

		int x_idx = it->first / (y_voxel_num * z_voxel_num);
		int y_idx = (it->first % (y_voxel_num * z_voxel_num)) / z_voxel_num;
		int z_idx = (it->first % (y_voxel_num * z_voxel_num)) % z_voxel_num;

		temp_PP.geometry[0] = (float(x_idx) / (x_voxel_num - 1) * x_size) + min[0] + (x_stride / 2);
		temp_PP.geometry[1] = (float(y_idx) / (y_voxel_num - 1) * y_size) + min[1] + (y_stride / 2);
		temp_PP.geometry[2] = (float(z_idx) / (z_voxel_num - 1) * z_size) + min[2] + (z_stride / 2);


		//cout << "iterator first :: " << it->first << endl;
		Plen_PC[it->first] = temp_PP;
		//Plen_PC.insert(color_map.end(), pair<unsigned long long, PPC>(it->first, temp_PP));
	}

	color_map.clear();

	return Plen_PC;
}

void make_proj_img_vec_ppc(
	vector<PPC> PPC,
	vector<Mat> &proj_img_vec,
	vector<PointCloud<PointXYZRGB>::Ptr> &pointclouds,
	int nNeighbor)
{
	for (int i = 0; i < total_num_cameras; i++)
		pointclouds[i] = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);

	//PC 8개 생성
	int count = 0;
	int p_count[10] = { 0 };
	for (int i = 0; i < PPC.size(); i++)
	{
		count = 0;
		for (int j = 0; j < total_num_cameras; j++)
		{
			if (PPC[i].occlusion_pattern[j] == true) {
				PointXYZRGB temp;

				//geometry
				temp.x = PPC[i].geometry[0];
				temp.y = PPC[i].geometry[1];
				temp.z = PPC[i].geometry[2];
				//color
				temp.r = uchar(PPC[i].color[j][0]);
				temp.g = uchar(PPC[i].color[j][1]);
				temp.b = uchar(PPC[i].color[j][2]);

				pointclouds[j]->points.push_back(temp);

				count++;
			}
		}
		if (count == 1) p_count[0] += 1;
		else if (count == 2) p_count[1] += 1;
		else if (count == 3) p_count[2] += 1;
		else if (count == 4) p_count[3] += 1;
		else if (count == 5) p_count[4] += 1;
		else if (count == 6) p_count[5] += 1;
		else if (count == 7) p_count[6] += 1;
		else if (count == 8) p_count[7] += 1;
		else if (count == 9) p_count[8] += 1;
		else if (count == 10) p_count[9] += 1;

	}

	cout << "color 1 point num: " << p_count[0] << endl;
	cout << "color 2 point num: " << p_count[1] << endl;
	cout << "color 3 point num: " << p_count[2] << endl;
	cout << "color 4 point num: " << p_count[3] << endl;
	cout << "color 5 point num: " << p_count[4] << endl;
	cout << "color 6 point num: " << p_count[5] << endl;
	cout << "color 7 point num: " << p_count[6] << endl;
	cout << "color 8 point num: " << p_count[7] << endl;
	cout << "color 9 point num: " << p_count[8] << endl;
	cout << "color 10 point num: " << p_count[9] << endl;


	for (int i = 0; i < total_num_cameras; i++)
	{
		Mat color_img(_height, _width, CV_8UC3, Scalar(0));
		Mat depth_value_img(_height, _width, CV_64F, -1);

		//back_projection(pointclouds[i], i, color_img, depth_value_img);
		back_projection(pointclouds[i], i, color_img, nNeighbor);

		proj_img_vec[i] = color_img;
	}
}

void make_proj_img_vec_ppc(
	vector<PPC_S> PPC_S,
	vector<Mat>& proj_img_vec,
	vector<PointCloud<PointXYZRGB>::Ptr>& pointclouds,
	int nNeighbor)
{
	for (int i = 0; i < total_num_cameras; i++)
		pointclouds[i] = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);


	//PC 8개 생성
	int count = 0;
	int p_count[10] = { 0 };
	for (int i = 0; i < PPC_S.size(); i++)
	{
		count = 0;
		for (int j = 0; j < total_num_cameras; j++)
		{
			if (PPC_S[i].occlusion_pattern[j] == true) {
				PointXYZRGB temp;

				//cout << (float)denormalization_s(PPC_S[i].geometry[2], geo_min[2], geo_max[2]) << endl;

				//geometry
				temp.x = (float)denormalization_s(PPC_S[i].geometry[0], geo_min[0], geo_max[0]);
				temp.y = (float)denormalization_s(PPC_S[i].geometry[1], geo_min[1], geo_max[1]);
				temp.z = (float)denormalization_s(PPC_S[i].geometry[2], geo_min[2], geo_max[2]);

				//if (i == 12345) cout << "ddddddddddddddddddd\t" << temp.x << "\t" << temp.y << "\t" << temp.z << endl;

				//color
				temp.r = uchar(PPC_S[i].color[j][0]);
				temp.g = uchar(PPC_S[i].color[j][1]);
				temp.b = uchar(PPC_S[i].color[j][2]);

				pointclouds[j]->points.push_back(temp);

				count++;
			}
		}

		//view_PC(pointclouds[5]);
		if (count == 1) p_count[0] += 1;
		else if (count == 2) p_count[1] += 1;
		else if (count == 3) p_count[2] += 1;
		else if (count == 4) p_count[3] += 1;
		else if (count == 5) p_count[4] += 1;
		else if (count == 6) p_count[5] += 1;
		else if (count == 7) p_count[6] += 1;
		else if (count == 8) p_count[7] += 1;
		else if (count == 9) p_count[8] += 1;
		else if (count == 10) p_count[9] += 1;

	}

	cout << "color 1 point num: " << p_count[0] << endl;
	cout << "color 2 point num: " << p_count[1] << endl;
	cout << "color 3 point num: " << p_count[2] << endl;
	cout << "color 4 point num: " << p_count[3] << endl;
	cout << "color 5 point num: " << p_count[4] << endl;
	cout << "color 6 point num: " << p_count[5] << endl;
	cout << "color 7 point num: " << p_count[6] << endl;
	cout << "color 8 point num: " << p_count[7] << endl;
	cout << "color 9 point num: " << p_count[8] << endl;
	cout << "color 10 point num: " << p_count[9] << endl;


	for (int i = 0; i < total_num_cameras; i++)
	{
		Mat color_img(_height, _width, CV_8UC3, Scalar(0));
		Mat depth_value_img(_height, _width, CV_64F, -1);

		//back_projection(pointclouds[i], i, color_img, depth_value_img);
		back_projection(pointclouds[i], i, color_img, nNeighbor);

		proj_img_vec[i] = color_img;
	}
}


void projection_PPC_with_hole_filling(
	vector<PPC> Plen_PC,
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

void projection_PPC_with_hole_filling(
	vector<PPC_S> Plen_PC_S,
	vector<Mat>& projection_imgs,
	vector<Mat>& filled_imgs,
	vector<PointCloud<PointXYZRGB>::Ptr>& pointclouds_,
	int nNeighbor,
	int window_size)
{
	//make_proj_img_vec_by_seq(Plen_PC, projection_imgs, depth_value_imgs);

	make_proj_img_vec_ppc(Plen_PC_S, projection_imgs, pointclouds_, nNeighbor);

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

void save_ppc(vector<PPC> ppc) {
	ofstream fout;
	string filename = "ppc_data.txt";
	fout.open(filename.c_str());
	for (vector<PPC>::iterator vit = ppc.begin(), vend = ppc.end(); vit != vend; vit++)
	{

		fout << fixed;
		fout << setprecision(6) << vit->geometry[0] << " " << vit->geometry[1] << " " << vit->geometry[2] << setprecision(0)
			<< " " << vit->color[0][0] << " " << vit->color[0][1] << " " << vit->color[0][2] << " "
			<< " " << vit->color[1][0] << " " << vit->color[1][1] << " " << vit->color[1][2]
			<< " " << vit->color[2][0] << " " << vit->color[2][1] << " " << vit->color[2][2]
			<< " " << vit->color[3][0] << " " << vit->color[3][1] << " " << vit->color[3][2]
			<< " " << vit->color[4][0] << " " << vit->color[4][1] << " " << vit->color[4][2]
			<< " " << vit->color[5][0] << " " << vit->color[5][1] << " " << vit->color[5][2]
			<< " " << vit->color[6][0] << " " << vit->color[6][1] << " " << vit->color[6][2]
			<< " " << vit->color[7][0] << " " << vit->color[7][1] << " " << vit->color[7][2]
			<< endl;
	}
	fout.close();
}

vector<PPC> load_ppc(void) {
	vector<PPC> vec_ppc;
	ifstream fin;
	string filename = "ppc_data.txt";
	string s;
	fin.open(filename.c_str());

	while (getline(fin, s))//getline(fin, a) != NULL) {
	{
		PPC pc;
		stringstream ss;
		ss << s;
		ss >> pc.geometry[0];
		//cout << pc.geometry[0] << "," ;
		ss >> pc.geometry[1];
		//cout << pc.geometry[1] << ",";
		ss >> pc.geometry[2];
		//cout << pc.geometry[2] << ",";
		ss >> pc.color[0][0];
		ss >> pc.color[0][1];
		ss >> pc.color[0][2];
		ss >> pc.color[1][0];
		ss >> pc.color[1][1];
		ss >> pc.color[1][2];
		ss >> pc.color[2][0];
		ss >> pc.color[2][1];
		ss >> pc.color[2][2];
		ss >> pc.color[3][0];
		ss >> pc.color[3][1];
		ss >> pc.color[3][2];
		ss >> pc.color[4][0];
		ss >> pc.color[4][1];
		ss >> pc.color[4][2];
		ss >> pc.color[5][0];
		ss >> pc.color[5][1];
		ss >> pc.color[5][2];
		ss >> pc.color[6][0];
		ss >> pc.color[6][1];
		ss >> pc.color[6][2];
		ss >> pc.color[7][0];
		ss >> pc.color[7][1];
		ss >> pc.color[7][2];
		vec_ppc.push_back(pc);
	}
	fin.close();
	return vec_ppc;
}

void make_proj_img_vec_by_seq(
	vector<PPC> Plen_PC,
	vector<Mat> &proj_img_vec,
	vector<Mat> &depth_value_img_vec)
{
	for (int i = 0; i < total_num_cameras; i++)
	{
		Mat color_img(_height, _width, CV_8UC3, Scalar(0));
		Mat depth_value_img(_height, _width, CV_64F, -1);

		PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);

		for (int point_idx = 0; point_idx < Plen_PC.size(); point_idx++) {

			if (Plen_PC[point_idx].color[i][0] == -1) continue;
			else {
				PointXYZRGB p;

				p.x = Plen_PC[point_idx].geometry[0];
				p.y = Plen_PC[point_idx].geometry[1];
				p.z = Plen_PC[point_idx].geometry[2];
				p.b = Plen_PC[point_idx].color[i][0];
				p.g = Plen_PC[point_idx].color[i][1];
				p.r = Plen_PC[point_idx].color[i][2];

				pointcloud->points.push_back(p);
			}
		}


		//back_projection(pointcloud, i, color_img, 4);
		projection(pointcloud, i, color_img, depth_value_img);
		/*projection_new(pointcloud, i, color_img, depth_value_img);*/
		//projection_new_MPC(pointcloud, i, color_img, depth_value_img);

		proj_img_vec[i] = color_img;

		depth_value_img_vec[i] = depth_value_img;
	}
}

void make_frustum(
	PointCloud<PointXYZRGB>::Ptr source_PC,
	Point3f& min,
	Point3f& max,
	Point3f& frustum_maxXY_maxZ,
	Point3f& frustum_minxy_maxZ)
{

	min.x = 1000000000000000;
	min.y = 1000000000000000;
	min.z = 1000000000000000;

	max.x = -100000000000000;
	max.y = -100000000000000;
	max.z = -100000000000000;

	vector<float> z_minmaxXY(4);

	for (int i = 0; i < source_PC->points.size(); i++)
	{
		//z_minmaxXY[0] - minX일 때 z값
		//z_minmaxXY[1] - minY일 때 z값
		//z_minmaxXY[2] - maxX일 때 z값
		//z_minmaxXY[3] - maxY일 때 z값

		if (source_PC->points[i].x < min.x) {
			min.x = source_PC->points[i].x;
			z_minmaxXY[0] = source_PC->points[i].z;
		}
		if (source_PC->points[i].y < min.y) {
			min.y = source_PC->points[i].y;
			z_minmaxXY[1] = source_PC->points[i].z;
		}
		if (source_PC->points[i].z < min.z) {
			min.z = source_PC->points[i].z;
		}

		if (source_PC->points[i].x > max.x) {
			max.x = source_PC->points[i].x;
			z_minmaxXY[2] = source_PC->points[i].z;
		}
		if (source_PC->points[i].y > max.y) {
			max.y = source_PC->points[i].y;
			z_minmaxXY[3] = source_PC->points[i].z;
		}
		if (source_PC->points[i].z > max.z) {
			max.z = source_PC->points[i].z;
		}
	}
	int threshold;
	if (!mode) threshold = 30; //원래 값 10
	else threshold = 3; // 원래 값 10

	if (abs(z_minmaxXY[0] - z_minmaxXY[1]) <= 0) {
		frustum_minxy_maxZ.x = min.z * min.x / z_minmaxXY[0] - threshold;
		frustum_minxy_maxZ.y = min.z * min.y / z_minmaxXY[0] - threshold;
		frustum_minxy_maxZ.z = min.z;
	}
	else {
		frustum_minxy_maxZ.x = min.z * min.x / z_minmaxXY[1] - threshold;
		frustum_minxy_maxZ.y = min.z * min.y / z_minmaxXY[1] - threshold;
		frustum_minxy_maxZ.z = min.z;
	}

	if (abs(z_minmaxXY[2] - z_minmaxXY[3]) <= 0) {
		frustum_maxXY_maxZ.x = min.z * max.x / z_minmaxXY[2] + threshold;
		frustum_maxXY_maxZ.y = min.z * max.y / z_minmaxXY[2] + threshold;
		frustum_maxXY_maxZ.z = min.z;
	}
	else {
		frustum_maxXY_maxZ.x = min.z * max.x / z_minmaxXY[3] + threshold;
		frustum_maxXY_maxZ.y = min.z * max.y / z_minmaxXY[3] + threshold;
		frustum_maxXY_maxZ.z = min.z;
	}
}


vector<PPC> make_frustum_Plen_PC(
	PointCloud<PointXYZRGB>::Ptr registered_PC,
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	int voxel_div_num) {

	clock_t start, end;

	start = clock();
	vector<PPC> Plen_PC;

	Point3f min, max;
	Point3f frustum_maxXY_maxZ;
	Point3f frustum_minxy_maxZ;



	//view_PC(PC_transformed);

	make_frustum(
		registered_PC,
		min,
		max,
		frustum_maxXY_maxZ,
		frustum_minxy_maxZ);

	cout << "maxX: " << max.x << " minX : " << min.x << endl;
	cout << "maxY: " << max.y << " minY : " << min.y << endl;
	cout << "maxZ: " << max.z << " minZ : " << min.z << endl;

	cout << "===== " << endl;

	cout << frustum_minxy_maxZ.x << " " << frustum_maxXY_maxZ.x << endl;
	cout << frustum_minxy_maxZ.y << " " << frustum_maxXY_maxZ.y << endl;

	cout << " ===== " << endl;
	unsigned long long x_unit_frustum_num = voxel_div_num;
	unsigned long long y_unit_frustum_num = voxel_div_num;
	unsigned long long z_unit_frustum_num = voxel_div_num;

	unsigned long long total_voxel_num = x_unit_frustum_num * y_unit_frustum_num * z_unit_frustum_num;

	map<unsigned long long, vector<vector<Vec3b>>> color_map;

	float temp;
	//if (!mode) temp = max.z, max.z = min.z, min.z = temp;

	for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
		for (unsigned long long point_idx = 0; point_idx < pointclouds[cam_num]->points.size(); point_idx++) {


			float x_size = pointclouds[cam_num]->points[point_idx].z * (frustum_maxXY_maxZ.x - frustum_minxy_maxZ.x) / max.z;
			float y_size = pointclouds[cam_num]->points[point_idx].z * (frustum_maxXY_maxZ.y - frustum_minxy_maxZ.y) / max.z;
			float z_size = (max.z - min.z);

			float min_x = pointclouds[cam_num]->points[point_idx].z * frustum_minxy_maxZ.x / max.z;
			float min_y = pointclouds[cam_num]->points[point_idx].z * frustum_minxy_maxZ.y / max.z;
			float min_z = min.z;

			unsigned long long  x_frustum_index = (int)floor((pointclouds[cam_num]->points[point_idx].x - min_x) / x_size * ((float)x_unit_frustum_num - 1));
			unsigned long long  y_frustum_index = (int)floor((pointclouds[cam_num]->points[point_idx].y - min_y) / y_size * ((float)y_unit_frustum_num - 1));
			unsigned long long  z_frustum_index = (int)floor((pointclouds[cam_num]->points[point_idx].z - min_z) / z_size * ((float)z_unit_frustum_num - 1));
			unsigned long long cube_index = x_frustum_index * (y_unit_frustum_num * z_unit_frustum_num) + y_frustum_index * z_unit_frustum_num + z_frustum_index;

			if (color_map.find(cube_index) == color_map.end()) {
				vector<vector<Vec3b>> temp_color(total_num_cameras);
				temp_color[cam_num].push_back(Vec3b(pointclouds[cam_num]->points[point_idx].r,
					pointclouds[cam_num]->points[point_idx].g,
					pointclouds[cam_num]->points[point_idx].b));
				color_map.insert(color_map.end(), pair<unsigned long long, vector<vector<Vec3b>>>(cube_index, temp_color));
			}
			// 기존 key 중에 존재하면, 해당 카메라에 색을 쌓음.
			else {
				color_map[cube_index][cam_num].push_back(Vec3b(pointclouds[cam_num]->points[point_idx].r,
					pointclouds[cam_num]->points[point_idx].g,
					pointclouds[cam_num]->points[point_idx].b));
			}
		}
	}

	// map의 iterator 코드
	// 색의 평균을 구하는 부분
	// first = key, second = value
	for (map<unsigned long long, vector<vector<Vec3b>>>::iterator it = color_map.begin(); it != color_map.end(); ++it) {
		PPC temp_PP;
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

				temp_PP.color[cam_num][0] = (short)avg_r;
				temp_PP.color[cam_num][1] = (short)avg_g;
				temp_PP.color[cam_num][2] = (short)avg_b;

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
		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
			if (it->second[cam_num].size() == 0) {
				//it->second[cam_num].push_back(Vec3b((uchar)inter_r, (uchar)inter_g, (uchar)inter_b));

				temp_PP.color[cam_num][0] = inter_r;
				temp_PP.color[cam_num][1] = inter_g;
				temp_PP.color[cam_num][2] = inter_b;
				temp_PP.occlusion_pattern[cam_num] = false;
			}
			else
				temp_PP.occlusion_pattern[cam_num] = true;
		}


		int x_idx = it->first / (y_unit_frustum_num * z_unit_frustum_num);
		int y_idx = (it->first % (y_unit_frustum_num * z_unit_frustum_num)) / z_unit_frustum_num;
		int z_idx = (it->first % (y_unit_frustum_num * z_unit_frustum_num)) % z_unit_frustum_num;

		float z_stride = (max.z - min.z) / z_unit_frustum_num;

		//temp_PP.geometry[2] = max.z - (z_idx * ((max.z - min.z) / (z_unit_frustum_num - 1)) + z_stride / 2);
		temp_PP.geometry[2] = min.z + (z_idx * ((max.z - min.z) / (z_unit_frustum_num - 1)) + z_stride / 2);

		float min_x = temp_PP.geometry[2] * frustum_minxy_maxZ.x / max.z;
		float min_y = temp_PP.geometry[2] * frustum_minxy_maxZ.y / max.z;

		float x_size = temp_PP.geometry[2] * (frustum_maxXY_maxZ.x - frustum_minxy_maxZ.x) / max.z; //-130
		float y_size = temp_PP.geometry[2] * (frustum_maxXY_maxZ.y - frustum_minxy_maxZ.y) / max.z;
		float z_size = (max.z - min.z);

		float x_stride = x_size / x_unit_frustum_num;
		float y_stride = y_size / y_unit_frustum_num;

		//temp_PP.geometry[0] = min_x + x_idx * x_stride + x_stride / 2;
		//temp_PP.geometry[1] = min_y + y_idx * y_stride + y_stride / 2;
		temp_PP.geometry[0] = (float(x_idx) / (x_unit_frustum_num - 1) * x_size) + min_x + (x_stride / 2);
		temp_PP.geometry[1] = (float(y_idx) / (y_unit_frustum_num - 1) * y_size) + min_y + (y_stride / 2);

		Plen_PC.push_back(temp_PP);
	}

	color_map.clear();
	end = clock();
	cout << "소요 시간 : " << (double)(end - start) / CLOCKS_PER_SEC << endl;
	return Plen_PC;

}

/*
vector<PPC> make_frustum_Plen_PC(
   vector<PPC> PPC_vec,
   vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
   int voxel_div_num) {

   clock_t start, end;

   start = clock();
   vector<PPC> Plen_PC;


   Point3f min, max;
   Point3f frustum_maxXY_maxZ;
   Point3f frustum_minxy_maxZ;

   vector<PointCloud<PointXYZRGB>::Ptr> pointcloud_vec;
   for (int cam = 0; cam < total_num_cameras; cam++) {
	  PointCloud<PointXYZRGB>::Ptr pointcloud_ppc(new PointCloud<PointXYZRGB>);
	  for (int i = 0; i < PPC_vec.size(); i++) {
		 if (PPC_vec[i].occlusion_pattern[cam] == true) {
			PointXYZRGB p;
			p.x = PPC_vec[i].geometry[0];
			p.y = PPC_vec[i].geometry[1];
			p.z = PPC_vec[i].geometry[2];
			p.r = (uchar)PPC_vec[i].color[cam][0];
			p.g = (uchar)PPC_vec[i].color[cam][1];
			p.b = (uchar)PPC_vec[i].color[cam][2];

			pointcloud_ppc->points.push_back(p);
		 }
	  }
	  pointcloud_vec.push_back(pointcloud_ppc);
   }

   int sum = 0;
   for (int i = 0; i < total_num_cameras; i++) {
	  cout << pointcloud_vec[i]->points.size() << endl;
	  sum += pointcloud_vec[i]->points.size();
   }
   cout << "\n sum : " << sum << endl;
   PointCloud<PointXYZRGB>::Ptr PC_transformed(new PointCloud<PointXYZRGB>);

   int ref = total_num_cameras / 2;

   Matrix3d refR = m_CalibParams[ref].m_RotMatrix;
   Matrix3Xd refT(3, 1);
   refT = m_CalibParams[ref].m_Trans;

   Matrix3Xd refRT(3, 4);
   refRT.col(0) = m_CalibParams[ref].m_RotMatrix.col(0);
   refRT.col(1) = m_CalibParams[ref].m_RotMatrix.col(1);
   refRT.col(2) = m_CalibParams[ref].m_RotMatrix.col(2);
   refRT.col(3) = m_CalibParams[ref].m_Trans.col(0);

   Matrix4d refRT4x4;
   refRT4x4.row(0) = refRT.row(0);
   refRT4x4.row(1) = refRT.row(1);
   refRT4x4.row(2) = refRT.row(2);
   refRT4x4.row(3) << 0, 0, 0, 1;

   for (int cam_num = 0; cam_num < total_num_cameras; cam_num++)
   {
	  transformPointCloud(*pointcloud_vec[cam_num], *pointcloud_vec[cam_num], refRT4x4);

	  *PC_transformed += *pointcloud_vec[cam_num];

	  m_CalibParams[cam_num].m_Trans = m_CalibParams[cam_num].m_RotMatrix * (-refR.inverse() * refT) + m_CalibParams[cam_num].m_Trans;
	  m_CalibParams[cam_num].m_RotMatrix *= refR.inverse();

	  m_CalibParams[cam_num].m_ProjMatrix = compute_projection_matrices(cam_num);
   }

   make_frustum(
	  PC_transformed,
	  min,
	  max,
	  frustum_maxXY_maxZ,
	  frustum_minxy_maxZ);


   unsigned long long x_unit_frustum_num = voxel_div_num;
   unsigned long long y_unit_frustum_num = voxel_div_num;
   unsigned long long z_unit_frustum_num = voxel_div_num;

   unsigned long long total_voxel_num = x_unit_frustum_num * y_unit_frustum_num * z_unit_frustum_num;

   map<unsigned long long, vector<vector<Vec3b>>> color_map;

   vector<Mat> orig_gray;
   for (int i = 0; i < total_num_cameras; i++) {
	  Mat gray(_height, _width, CV_8UC1);
	  cvtColor(orig_imgs[i], gray, COLOR_BGR2GRAY);
	  orig_gray.push_back(gray);
   }

   for (unsigned long long point_idx = 0; point_idx < PPC_vec.size(); point_idx++) {

	  float x_size = PPC_vec[point_idx].geometry[2] * (frustum_maxXY_maxZ.x - frustum_minxy_maxZ.x) / min.z; //-130
	  float y_size = PPC_vec[point_idx].geometry[2] * (frustum_maxXY_maxZ.y - frustum_minxy_maxZ.y) / min.z;
	  float z_size = (max.z - min.z);

	  float min_x = PPC_vec[point_idx].geometry[2] * frustum_minxy_maxZ.x / min.z;
	  float min_y = PPC_vec[point_idx].geometry[2] * frustum_minxy_maxZ.y / min.z;
	  float min_z = max.z;

	  unsigned long long  x_voxel_index = (int)floor((PPC_vec[point_idx].geometry[0] - min_x) / x_size * ((float)x_unit_frustum_num - 1));
	  unsigned long long  y_voxel_index = (int)floor((PPC_vec[point_idx].geometry[1] - min_y) / y_size * ((float)y_unit_frustum_num - 1));
	  unsigned long long  z_voxel_index = (int)floor((min_z - PPC_vec[point_idx].geometry[2]) / z_size * ((float)z_unit_frustum_num - 1));
	  unsigned long long cube_index = x_voxel_index * (y_unit_frustum_num * z_unit_frustum_num) + y_voxel_index * z_unit_frustum_num + z_voxel_index;


	  if (color_map.find(cube_index) == color_map.end()) {
		 vector<vector<Vec3b>> temp_color(total_num_cameras);
		 for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
			if (PPC_vec[point_idx].occlusion_pattern[cam_num] == true) {
			   temp_color[cam_num].push_back(Vec3b(PPC_vec[point_idx].color[cam_num][0],
				  PPC_vec[point_idx].color[cam_num][1],
				  PPC_vec[point_idx].color[cam_num][2]));
			}
		 }
		 color_map.insert(color_map.end(), pair<unsigned long long, vector<vector<Vec3b>>>(cube_index, temp_color));
	  }
	  // 기존 key 중에 존재하면, 해당 카메라에 색을 쌓음.
	  else {
		 double X;
		 double Y;
		 double Z;

		 int u;
		 int v;
		 double w;
		 float tmp1, tmp2;
		 for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
			if (PPC_vec[point_idx].occlusion_pattern[cam_num] == true) {

			   X = PPC_vec[point_idx].geometry[0];
			   Y = PPC_vec[point_idx].geometry[1];
			   Z = PPC_vec[point_idx].geometry[2];

			   if (!mode) {
				  Z = -Z;
			   }

			   w = projection_XYZ_2_UV(
				  m_CalibParams[cam_num].m_ProjMatrix,
				  X,
				  Y,
				  Z,
				  u,
				  v);
			   cout << u << " " << v << endl;
			   if ((u < 0) || (v < 0) || (u > _width - 1) || (v > _height - 1)) continue;

			   Mat one_pixel_query(1, 1, CV_8UC3), one_pixel_0(1, 1, CV_8UC3);
			   one_pixel_0.at<Vec3b>(0, 0) = color_map[cube_index][cam_num][0];
			   cout << " 1 " << endl;

			   one_pixel_query.at<Vec3b>(0, 0) = Vec3b(PPC_vec[point_idx].color[cam_num][0],
				  PPC_vec[point_idx].color[cam_num][1],
				  PPC_vec[point_idx].color[cam_num][2]);
			   cout << " 2 " << endl;
			   cvtColor(one_pixel_0, one_pixel_0, COLOR_BGR2GRAY);
			   cvtColor(one_pixel_query, one_pixel_query, COLOR_BGR2GRAY);
			   tmp1 = abs(one_pixel_0.at<uchar>(0, 0) - orig_gray[cam_num].at<uchar>(v, u));
			   tmp2 = abs(one_pixel_query.at<uchar>(0, 0) - orig_gray[cam_num].at<uchar>(v, u));
			   cout << " 3 " << endl;

			   if(tmp1>tmp2)
				  /*color_map[cube_index][cam_num].push_back(Vec3b(PPC_vec[point_idx].color[cam_num][0],
				  //PPC_vec[point_idx].color[cam_num][1],
				  //PPC_vec[point_idx].color[cam_num][2]));
				  color_map[cube_index][cam_num][0]= (Vec3b(PPC_vec[point_idx].color[cam_num][0],
					 PPC_vec[point_idx].color[cam_num][1],
					 PPC_vec[point_idx].color[cam_num][2]));
			   cout << " 4 " << endl;

			}

		 }
	  }
   }


   cout << "color_map.size() : " << color_map.size() << endl;

   // map의 iterator 코드
   // 색의 평균을 구하는 부분
   // first = key, second = value
   for (map<unsigned long long, vector<vector<Vec3b>>>::iterator it = color_map.begin(); it != color_map.end(); ++it) {
	  PPC temp_PP;
	  int inter_r = 0, inter_g = 0, inter_b = 0;
	  int inter_cnt = 0;

	  for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
		 if (it->second[cam_num].size() > 0) {
			int avg_r = 0, avg_g = 0, avg_b = 0;
			Vec3b max_sim = it->second[cam_num][0];




			for (unsigned long long i = 0; i < it->second[cam_num].size(); i++) {
			   avg_r += int(it->second[cam_num][i][0]);
			   avg_g += int(it->second[cam_num][i][1]);
			   avg_b += int(it->second[cam_num][i][2]);


			}

			avg_r /= it->second[cam_num].size();
			avg_g /= it->second[cam_num].size();
			avg_b /= it->second[cam_num].size();

			temp_PP.color[cam_num][0] = (short)avg_r;
			temp_PP.color[cam_num][1] = (short)avg_g;
			temp_PP.color[cam_num][2] = (short)avg_b;

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
	  for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
		 if (it->second[cam_num].size() == 0) {
			//it->second[cam_num].push_back(Vec3b((uchar)inter_r, (uchar)inter_g, (uchar)inter_b));

			temp_PP.color[cam_num][0] = inter_r;
			temp_PP.color[cam_num][1] = inter_g;
			temp_PP.color[cam_num][2] = inter_b;
			temp_PP.occlusion_pattern[cam_num] = false;
		 }
		 else
			temp_PP.occlusion_pattern[cam_num] = true;
	  }


	  int x_idx = it->first / (y_unit_frustum_num * z_unit_frustum_num);
	  int y_idx = (it->first % (y_unit_frustum_num * z_unit_frustum_num)) / z_unit_frustum_num;
	  int z_idx = (it->first % (y_unit_frustum_num * z_unit_frustum_num)) % z_unit_frustum_num;

	  float z_stride = (max.z - min.z) / z_unit_frustum_num;

	  temp_PP.geometry[2] = max.z - (z_idx * ((max.z - min.z) / (z_unit_frustum_num - 1)) + z_stride / 2);

	  float min_x = temp_PP.geometry[2] * frustum_minxy_maxZ.x / min.z;
	  float min_y = temp_PP.geometry[2] * frustum_minxy_maxZ.y / min.z;

	  float x_size = temp_PP.geometry[2] * (frustum_maxXY_maxZ.x - frustum_minxy_maxZ.x) / min.z; //-130
	  float y_size = temp_PP.geometry[2] * (frustum_maxXY_maxZ.y - frustum_minxy_maxZ.y) / min.z;
	  float z_size = (max.z - min.z);

	  float x_stride = x_size / x_unit_frustum_num;
	  float y_stride = y_size / y_unit_frustum_num;

	  //temp_PP.geometry[0] = min_x + x_idx * x_stride + x_stride / 2;
	  //temp_PP.geometry[1] = min_y + y_idx * y_stride + y_stride / 2;
	  temp_PP.geometry[0] = (float(x_idx) / (x_unit_frustum_num - 1) * x_size) + min_x + (x_stride / 2);
	  temp_PP.geometry[1] = (float(y_idx) / (y_unit_frustum_num - 1) * y_size) + min_y + (y_stride / 2);

	  Plen_PC.push_back(temp_PP);

   }
   vector<int> sum_true(8), sum_false(8);
   for (int i = 0; i < Plen_PC.size(); i++) {
	  for (int cam = 0; cam < total_num_cameras; cam++) {

		 if (Plen_PC[i].occlusion_pattern[cam] == true)
			sum_true[cam]++;
		 else sum_false[cam]++;
	  }
   }
   for (int i = 0; i < 8; i++) {

	  cout << "sum_true : " << sum_true[i] << endl;
	  cout << "sum_false : " << sum_false[i] << endl;
   }

   color_map.clear();
   end = clock();
   cout << "소요 시간 : " << (double)(end - start) / CLOCKS_PER_SEC << endl;
   return Plen_PC;

}
*/

vector<PPC> make_frustum_Plen_PC(
	vector<PPC> PPC_vec,
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	int voxel_div_num) {

	clock_t start, end;

	start = clock();
	vector<PPC> Plen_PC;


	Point3f min, max;
	Point3f frustum_maxXY_maxZ;
	Point3f frustum_minxy_maxZ;

	PointCloud<PointXYZRGB>::Ptr pointcloud_ppc(new PointCloud<PointXYZRGB>);

	for (int i = 0; i < PPC_vec.size(); i++) {
		PointXYZRGB p;
		p.x = PPC_vec[i].geometry[0];
		p.y = PPC_vec[i].geometry[1];
		p.z = PPC_vec[i].geometry[2];
		p.r = 0;
		p.g = 0;
		p.b = 0;

		pointcloud_ppc->points.push_back(p);
	}

	/*int sum = 0;
	for (int i = 0; i < total_num_cameras; i++) {
	   cout << pointcloud_vec[i]->points.size() << endl;
	   sum += pointcloud_vec[i]->points.size();
	}
	cout << "\n sum : " << sum << endl;
	*/

	make_frustum(
		pointcloud_ppc,
		min,
		max,
		frustum_maxXY_maxZ,
		frustum_minxy_maxZ);
	cout << "1" << endl;

	unsigned long long x_unit_frustum_num = voxel_div_num;
	unsigned long long y_unit_frustum_num = voxel_div_num;
	unsigned long long z_unit_frustum_num = voxel_div_num;

	unsigned long long total_voxel_num = x_unit_frustum_num * y_unit_frustum_num * z_unit_frustum_num;

	map<unsigned long long, vector<vector<Vec3b>>> color_map;


	for (unsigned long long point_idx = 0; point_idx < PPC_vec.size(); point_idx++) {

		float x_size = PPC_vec[point_idx].geometry[2] * (frustum_maxXY_maxZ.x - frustum_minxy_maxZ.x) / max.z;
		float y_size = PPC_vec[point_idx].geometry[2] * (frustum_maxXY_maxZ.y - frustum_minxy_maxZ.y) / max.z;
		float z_size = (max.z - min.z);

		float min_x = PPC_vec[point_idx].geometry[2] * frustum_minxy_maxZ.x / max.z;
		float min_y = PPC_vec[point_idx].geometry[2] * frustum_minxy_maxZ.y / max.z;
		float min_z = min.z;

		unsigned long long  x_frustum_index = (int)floor((PPC_vec[point_idx].geometry[0] - min_x) / x_size * ((float)x_unit_frustum_num - 1));
		unsigned long long  y_frustum_index = (int)floor((PPC_vec[point_idx].geometry[1] - min_y) / y_size * ((float)y_unit_frustum_num - 1));
		unsigned long long  z_frustum_index = (int)floor((PPC_vec[point_idx].geometry[2] - min_z) / z_size * ((float)z_unit_frustum_num - 1));
		unsigned long long cube_index = x_frustum_index * (y_unit_frustum_num * z_unit_frustum_num) + y_frustum_index * z_unit_frustum_num + z_frustum_index;

		if (color_map.find(cube_index) == color_map.end()) {
			vector<vector<Vec3b>> temp_color(total_num_cameras);
			for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
				if (PPC_vec[point_idx].occlusion_pattern[cam_num] == true) {
					temp_color[cam_num].push_back(Vec3b(PPC_vec[point_idx].color[cam_num][0],
						PPC_vec[point_idx].color[cam_num][1],
						PPC_vec[point_idx].color[cam_num][2]));
				}
			}
			color_map.insert(color_map.end(), pair<unsigned long long, vector<vector<Vec3b>>>(cube_index, temp_color));
		}
		// 기존 key 중에 존재하면, 해당 카메라에 색을 쌓음.
		else {
			for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
				if (PPC_vec[point_idx].occlusion_pattern[cam_num] == true) {
					color_map[cube_index][cam_num].push_back(Vec3b(PPC_vec[point_idx].color[cam_num][0],
						PPC_vec[point_idx].color[cam_num][1],
						PPC_vec[point_idx].color[cam_num][2]));
				}
			}
		}
	}
	cout << "2" << endl;

	// map의 iterator 코드
	// 색의 평균을 구하는 부분
	// first = key, second = value
	for (map<unsigned long long, vector<vector<Vec3b>>>::iterator it = color_map.begin(); it != color_map.end(); ++it) {
		PPC temp_PP;
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

				temp_PP.color[cam_num][0] = (short)avg_r;
				temp_PP.color[cam_num][1] = (short)avg_g;
				temp_PP.color[cam_num][2] = (short)avg_b;

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
		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
			if (it->second[cam_num].size() == 0) {
				//it->second[cam_num].push_back(Vec3b((uchar)inter_r, (uchar)inter_g, (uchar)inter_b));

				temp_PP.color[cam_num][0] = inter_r;
				temp_PP.color[cam_num][1] = inter_g;
				temp_PP.color[cam_num][2] = inter_b;
				temp_PP.occlusion_pattern[cam_num] = false;
			}
			else
				temp_PP.occlusion_pattern[cam_num] = true;
		}


		int x_idx = it->first / (y_unit_frustum_num * z_unit_frustum_num);
		int y_idx = (it->first % (y_unit_frustum_num * z_unit_frustum_num)) / z_unit_frustum_num;
		int z_idx = (it->first % (y_unit_frustum_num * z_unit_frustum_num)) % z_unit_frustum_num;

		float z_stride = (max.z - min.z) / z_unit_frustum_num;

		//temp_PP.geometry[2] = max.z - (z_idx * ((max.z - min.z) / (z_unit_frustum_num - 1)) + z_stride / 2);
		temp_PP.geometry[2] = min.z + (z_idx * ((max.z - min.z) / (z_unit_frustum_num - 1)) + z_stride / 2);

		float min_x = temp_PP.geometry[2] * frustum_minxy_maxZ.x / max.z;
		float min_y = temp_PP.geometry[2] * frustum_minxy_maxZ.y / max.z;

		float x_size = temp_PP.geometry[2] * (frustum_maxXY_maxZ.x - frustum_minxy_maxZ.x) / max.z; //-130
		float y_size = temp_PP.geometry[2] * (frustum_maxXY_maxZ.y - frustum_minxy_maxZ.y) / max.z;
		float z_size = (max.z - min.z);

		float x_stride = x_size / x_unit_frustum_num;
		float y_stride = y_size / y_unit_frustum_num;

		//temp_PP.geometry[0] = min_x + x_idx * x_stride + x_stride / 2;
		//temp_PP.geometry[1] = min_y + y_idx * y_stride + y_stride / 2;
		temp_PP.geometry[0] = (float(x_idx) / (x_unit_frustum_num - 1) * x_size) + min_x + (x_stride / 2);
		temp_PP.geometry[1] = (float(y_idx) / (y_unit_frustum_num - 1) * y_size) + min_y + (y_stride / 2);

		Plen_PC.push_back(temp_PP);
	}
	cout << "3" << endl;

	vector<int> sum_true(8), sum_false(8);
	for (int i = 0; i < Plen_PC.size(); i++) {
		for (int cam = 0; cam < total_num_cameras; cam++) {

			if (Plen_PC[i].occlusion_pattern[cam] == true)
				sum_true[cam]++;
			else sum_false[cam]++;
		}
	}
	for (int i = 0; i < 8; i++) {

		cout << "sum_true : " << sum_true[i] << endl;
		cout << "sum_false : " << sum_false[i] << endl;
	}

	color_map.clear();
	end = clock();
	cout << "소요 시간 : " << (double)(end - start) / CLOCKS_PER_SEC << endl;
	return Plen_PC;

}

vector<PPC_S> make_normalized_Plen_PC(vector<PPC> Plen_PC) {

	//Plen_PC에 있던 것을 Plen_PC_S에 하나씩 값을 넣을건데요 잠깜나요 트름좀 그걸왜적는데
	// 그다음에 그까 지오메트리는 노믈라이제시션해서넣고 컬러는 그냥넣고
	// 노멀라이제이션 방법은  cmm 맨위 함수

	vector<PPC_S> Plen_PC_S(Plen_PC.size());

	PPC_S temp_s;
	vector<float> temp_min(3), temp_max(3);

	find_min_max(Plen_PC, temp_min, temp_max);
	//cout << "temp_min , temp_max " << endl;
	//cout << temp_min[2] << "\t" << temp_max[2] << endl;

	geo_min = temp_min;
	geo_max = temp_max;

	//normalization_s
	for (int index = 0; index < Plen_PC.size(); index++) {
		for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++) {
			temp_s.color[cam_idx][0] = Plen_PC[index].color[cam_idx][0];
			temp_s.color[cam_idx][1] = Plen_PC[index].color[cam_idx][1];
			temp_s.color[cam_idx][2] = Plen_PC[index].color[cam_idx][2];

			temp_s.occlusion_pattern[cam_idx] = Plen_PC[index].occlusion_pattern[cam_idx];
		}
		temp_s.geometry[0] = normalization_s(Plen_PC[index].geometry[0], geo_min[0], geo_max[0]);
		temp_s.geometry[1] = normalization_s(Plen_PC[index].geometry[1], geo_min[1], geo_max[1]);
		temp_s.geometry[2] = normalization_s(Plen_PC[index].geometry[2], geo_min[2], geo_max[2]);

		Plen_PC_S[index] = temp_s;
	}

	return Plen_PC_S;
}