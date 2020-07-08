#include "plenoptic_point_cloud.h"

vector<PPC> make_sequenced_Plen_PC(
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs,
	int d_threshold,
	int c_threshold) {

	vector<PPC> Plen_PC;
	clock_t start, end;

	// ���԰� ��ģ �κ�.
	vector<int> camera_order;
	int referenceView = total_num_cameras / 2;
	camera_order.push_back(referenceView);
	for (int i = 0; i < referenceView; i++) {
		if (referenceView - i - 1 >= 0) {
			camera_order.push_back(referenceView - i - 1);
		}
		if (referenceView + i + 1 < total_num_cameras) {
			camera_order.push_back(referenceView + i + 1);
		}
	}

	for (int i = 0; i < total_num_cameras; i++) {
		cout << camera_order[i] << endl;
	}

	start = clock();

	//ù��° �� PPC���� 
	for (int point_idx = 0; point_idx < pointclouds[0]->points.size(); point_idx++) {
		//PPC point_ppc_1;

		PPC* point_ppc = new PPC_v2_1();

		point_ppc->SetGeometry(pointclouds[0]->points[point_idx]);
		point_ppc->SetRefColor(pointclouds[0]->points[point_idx], 0);
		
		Plen_PC.push_back(*point_ppc);
	}
	//cout << "Plen_pc size 1 : " << Plen_PC.size() << endl;

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
			
			float* geometry = Plen_PC[point_idx].GetGeometry();
			X = geometry[0];
			Y = geometry[1];
			Z = geometry[2];

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
				// ���� ������ ������ �ȵ� ���
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

					PPC* point_ppc = new PPC_v2_1();


					float geo[3] = { X, Y, Z };

					point_ppc->SetGeometry(geo);
					//point_ppc.geometry[0] = X;
					//point_ppc.geometry[1] = Y;
					//point_ppc.geometry[2] = Z;
					point_ppc->SetRefColor(color, cam);

					Plen_PC.push_back(*point_ppc);

					//cout << point_ppc.color[cam][0] << " " << point_ppc.color[cam][1] << " " << point_ppc.color[cam][2] << " ";
				}
				// ������ �� ���
				else
				{
					int point_idx = confirm_img.at<int>(v, u);

					float* geometry = Plen_PC[point_idx].GetGeometry();
					X = geometry[0];
					Y = geometry[1];
					Z = geometry[2];

					int u_, v_;

					if (!mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y); // �� �ڵ��� �ǹ̴�?? �ϴ� ���� 
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

					//Mat hsv_new(1, 1, CV_8UC3);
					//hsv_new.at<Vec3b>(0, 0) = color;
					//cvtColor(hsv_new, hsv_new, CV_BGR2HSV);

					// ���԰� ��ħ.
					int nearCamNum = 0;
					for (int c = cam-1; c >= 0; c--) {
						if (!Plen_PC[point_idx].CheckOcclusion(c)) {
							nearCamNum = c;
							break;
						}
					}

					//Mat hsv_preexisted(1, 1, CV_8UC3);
					//hsv_preexisted.at<Vec3b>(0, 0)[0] = Plen_PC[point_idx].color[occ_true_camnum][2];
					//hsv_preexisted.at<Vec3b>(0, 0)[1] = Plen_PC[point_idx].color[occ_true_camnum][1];
					//hsv_preexisted.at<Vec3b>(0, 0)[2] = Plen_PC[point_idx].color[occ_true_camnum][0];
					//cvtColor(hsv_preexisted, hsv_preexisted, CV_BGR2HSV);

					//short sub_H = hsv_new.at<Vec3b>(0, 0)[0] - hsv_preexisted.at<Vec3b>(0, 0)[0];
					int sub_V = color[2] - Plen_PC[point_idx].GetColor(nearCamNum)[0];
					int sub_U = color[1] - Plen_PC[point_idx].GetColor(nearCamNum)[1];

					// depth thres, color thres ok
					if (abs(find_point_dist(w, cam) - find_point_dist(w_origin, cam)) < d_threshold && abs(sub_U) <= c_threshold && abs(sub_V) <= c_threshold) {
						Plen_PC[point_idx].SetColor(color, cam);
					}
					else {
						if (!mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z_, &X_, &Y_);
						else Z_ = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z_, &X_, &Y_);

						PPC* point_ppc = new PPC_v2_1();

						point_ppc->SetGeometry(pointclouds[0]->points[point_idx]);
						point_ppc->SetRefColor(pointclouds[0]->points[point_idx], 0);

						Plen_PC.push_back(*point_ppc);
					}
				}
			}
		}
	}
	end = clock();
	//cout << "Plen_pc size 2 : " << Plen_PC.size() << endl;

	/*vector<int> sum_true(8), sum_false(8);
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
	}*/

	//cout << "�ҿ� �ð� : " << (double)(end - start) / CLOCKS_PER_SEC << endl;

	return Plen_PC;
}

/////////////////////here///////////////////////
vector<PPC*> make_sequenced_Plen_PC(
	vector<Mat> color_imgs,
	vector<Mat> depth_imgs)
{
	vector<PPC*> Plen_PC;
	int c_threshold = 7;

	vector<int> camera_order;
	int refView = total_num_cameras / 2;
	camera_order.push_back(refView);

	for (int i = 0; i < refView; i++) {
		if (refView - i - 1 >= 0) {
			camera_order.push_back(refView - i - 1);
		}
		if (refView + i + 1 < total_num_cameras) {
			camera_order.push_back(refView + i + 1);
		}
	}

	PointCloud<PointXYZRGB>::Ptr firstPC;
	if (version == 2.1 || version == 3.1) {
		firstPC = make_PC(0, color_imgs[0], depth_imgs[0]);
	}
	else if (version == 2.2 || version == 3.2) {
		//firstPC = make_PC(total_num_cameras / 2, color_imgs[total_num_cameras / 2], depth_imgs[total_num_cameras / 2]);
		firstPC = make_PC(0, color_imgs[0], depth_imgs[0]);
	}

	// 첫번째 view 
	for (int point_idx = 0; point_idx < firstPC->points.size(); point_idx++) {
		PPC* point_ppc;

		if(version==2.1 || version == 3.1) point_ppc = new PPC_v2_1();
		else if(version == 2.2 || version == 3.2) point_ppc = new PPC_v2_2();

		
		point_ppc->SetGeometry(firstPC->points[point_idx]);

		if(version == 2.1 || version == 3.1) point_ppc->SetRefColor(firstPC->points[point_idx], 0);
		else if(version == 2.2 || version == 3.2) point_ppc->SetColor(firstPC->points[point_idx], 0);  //point_ppc->SetColor(firstPC->points[point_idx], refView); 

		Plen_PC.push_back(point_ppc);
	}

	cout << "first view done ... " << endl;
	for (int i = 1; i < total_num_cameras; i++) {
		int cam; 
		if (version == 2.) cam = i;
		else cam = i; //cam = camera_order[i];cam = i;

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

				switch (mode) {
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

				if (!mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y);
				else Z = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z, &X, &Y);

				// 투영이 안됬을 때
				if (confirm_img.at<int>(v, u) == -1) {
					
					PPC* point_ppc;
					if (version == 2.1 || version == 3.1) point_ppc = new PPC_v2_1();
					else if (version == 2.2 || version == 3.2) point_ppc = new PPC_v2_2();

					float geo[3] = { X,Y,Z };
					point_ppc->SetGeometry(geo);
					if (version == 2.1 || version == 3.1) point_ppc->SetRefColor(color, cam);
					else if (version == 2.2 || version == 3.2) point_ppc->SetColor(color, cam);

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
						if (version == 2.1 || version == 3.1) point_ppc = new PPC_v2_1();
						else if (version == 2.2|| version == 3.2) point_ppc = new PPC_v2_2();

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

	if (version == 3.1) {
	vector<PPC*> Plen_PC_v3_1;
	for (int i = 0; i < Plen_PC.size(); i++) {
		PPC* point_ppc = new PPC_v3_1(Plen_PC[i], degree); // new PPC_v2_5(*Plen_PC[i]);
		Plen_PC_v3_1.push_back(point_ppc);
	}
	Plen_PC = Plen_PC_v3_1;
	}
	
	else if (version == 3.2) {
		vector<PPC*> Plen_PC_v3_2;
		for (int i = 0; i < Plen_PC.size(); i++) {
			//cout << " i : " << i << endl;
			PPC* point_ppc = new PPC_v3_2(Plen_PC[i]); // new PPC_v2_5(*Plen_PC[i]);
			Plen_PC_v3_2.push_back(point_ppc);
			//cout << "====" << endl;
		}
		Plen_PC = Plen_PC_v3_2;
	}

	
	Plen_PC.shrink_to_fit();

	return Plen_PC;
}

//vector<PPC> make_sequenced_Plen_PC(
//	PointCloud<PointXYZRGB>::Ptr firstPC,
//	vector<Mat> color_imgs,
//	vector<Mat> depth_imgs) 
//{
//	vector<PPC> Plen_PC;
//	int c_threshold = 7;
//
//	clock_t start, end;
//
//	start = clock();
//
//	//ù��° �� PPC���� 
//	for (int point_idx = 0; point_idx < firstPC->points.size(); point_idx++) {
//		PPC point_ppc_1;
//
//		point_ppc_1.geometry[0] = firstPC->points[point_idx].x;
//		point_ppc_1.geometry[1] = firstPC->points[point_idx].y;
//		point_ppc_1.geometry[2] = firstPC->points[point_idx].z;
//		point_ppc_1.SetRefColor(firstPC->points[point_idx], 0);
//
//		Plen_PC.push_back(point_ppc_1);
//	}
//	//cout << "Plen_pc size 1 : " << Plen_PC.size() << endl;
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
//		Mat color_img(_height, _width, CV_8UC3, Scalar(0));
//		Mat depth_img(_height, _width, CV_64F, -1);
//		Mat confirm_img(_height, _width, CV_32S, -1);
//
//		for (int point_idx = 0; point_idx < Plen_PC.size(); point_idx++) {
//			X = Plen_PC[point_idx].geometry[0];
//			Y = Plen_PC[point_idx].geometry[1];
//			Z = Plen_PC[point_idx].geometry[2];
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
//				// ���� ������ ������ �ȵ� ���
//				if (confirm_img.at<int>(v, u) == -1) {
//					double Z;
//
//					if (!mode) {
//						Vec3b d = depth_imgs[cam].at<Vec3b>(v, u);
//						Z = depth_level_2_Z(d[0]);
//					}
//					else {
//						Vec3s d_s = depth_img.at<Vec3s>(v, u);
//						Z = depth_level_2_Z_s(d_s[0]);
//
//					}
//					double X = 0.0;
//					double Y = 0.0;
//
//					if (!mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y);
//					else Z = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z, &X, &Y);
//
//					PPC point_ppc_2;
//
//					point_ppc_2.geometry[0] = X;
//					point_ppc_2.geometry[1] = Y;
//					point_ppc_2.geometry[2] = Z;
//					point_ppc_2.SetRefColor(color, cam);
//
//					Plen_PC.push_back(point_ppc_2);
//
//					//cout << point_ppc.color[cam][0] << " " << point_ppc.color[cam][1] << " " << point_ppc.color[cam][2] << " ";
//				}
//				// ������ �� ���
//				else
//				{
//					int point_idx = confirm_img.at<int>(v, u);
//
//					X = Plen_PC[point_idx].geometry[0];
//					Y = Plen_PC[point_idx].geometry[1];
//					Z = Plen_PC[point_idx].geometry[2];
//
//					int u_, v_;
//
//					if (!mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z, &X, &Y); // �� �ڵ��� �ǹ̴�?? �ϴ� ���� 
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
//					for (int c = cam - 1; c >= 0; c--) {
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
//
//					int sub_V = color[2] - Plen_PC[point_idx].GetColor(nearCamNum)[0];
//					int sub_U = color[1] - Plen_PC[point_idx].GetColor(nearCamNum)[1];
//
//					// depth thres, color thres ok
//					if (abs(sub_U) < c_threshold && abs(sub_V) < c_threshold) {
//						Plen_PC[point_idx].SetColor(color, cam);
//					}
//					else {
//						if (!mode) projection_UVZ_2_XY_PC(m_CalibParams[cam].m_ProjMatrix, u, v, Z_, &X_, &Y_);
//						else Z_ = MVG(m_CalibParams[cam].m_K, m_CalibParams[cam].m_RotMatrix, m_CalibParams[cam].m_Trans, u, v, Z_, &X_, &Y_);
//
//						PPC point_ppc_3;
//
//						point_ppc_3.geometry[0] = X_;
//						point_ppc_3.geometry[1] = Y_;
//						point_ppc_3.geometry[2] = Z_;
//						point_ppc_3.SetRefColor(color, cam);
//
//						Plen_PC.push_back(point_ppc_3);
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

void make_proj_img_vec_ppc(
	vector<PPC*> PPC,
	vector<Mat> &proj_img_vec,
	vector<PointCloud<PointXYZRGB>::Ptr> &pointclouds,
	int nNeighbor)
{
	for (int i = 0; i < total_num_cameras; i++)
		pointclouds[i] = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);

	//PC 8�� ����
	PointXYZRGB temp;
	for (int i = 0; i < PPC.size(); i++)
	{
		for (int j = 0; j < total_num_cameras; j++)
		{
			// ���԰� ��ħ.
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

				pointclouds[j]->points.push_back(temp);
			}
		}
	}
	for (int i = 0; i < total_num_cameras; i++)
	{
		Mat color_img(_height, _width, CV_8UC3, Scalar(0));
		Mat depth_value_img(_height, _width, CV_64F, -1);

		//back_projection(pointclouds[i], i, color_img, depth_value_img);
		//projection(pointclouds[i], i, color_img, depth_value_img);
		back_projection(pointclouds[i], i, color_img, nNeighbor);

		proj_img_vec[i] = color_img;
	}
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

	//for (vector<PPC*>::iterator vit = ppc.begin(), vend = ppc.end(); vit != vend; vit++){
	//	float* geo = new float(3);
	//	geo = (*vit)->GetGeometry();
	//	fout.write((char*)&geo[0], sizeof(float));
	//	fout.write((char*)&geo[1], sizeof(float));
	//	fout.write((char*)&geo[2], sizeof(float));

	//	if (version == 2.1) {
	//		uchar refV = (*vit)->GetrefV();
	//		uchar refU = (*vit)->GetrefU();
	//		fout.write((char*)&refV, sizeof(uchar));
	//		fout.write((char*)&refU, sizeof(uchar));

	//		vector<uchar> VU = (*vit)->GetVU();
	//		vector<uchar> Y = (*vit)->GetY();

	//		for (int i = 0; i < total_num_cameras; i++) {
	//			fout.write((char*)&VU[i], sizeof(uchar));
	//		}
	//		for (int i = 0; i < total_num_cameras; i++) {
	//			fout.write((char*)&Y[i], sizeof(uchar));
	//		}
	//	}
	//	else if (version == 2.2) {
	//		uchar avrV = (*vit)->GetV();
	//		uchar avrU = (*vit)->GetU();
	//		fout.write((char*)&avrV, sizeof(uchar));
	//		fout.write((char*)&avrU, sizeof(uchar));

	//		vector<uchar> Y = (*vit)->GetY();
	//		vector<bool> occ = (*vit)->GetOcclusion();
	//	
	//		fout.write((char*)&Y[0], total_num_cameras * sizeof(uchar));
	//		for (int i = 0; i < total_num_cameras; i++) {
	//			char* temp;
	//			if (i % 8 == 0) {
	//				temp = new char;
	//				*temp = occ[i];
	//			}
	//			else if (i % 8 == 7 ) {
	//				*temp <<= 1;
	//				*temp |= occ[i];
	//				fout.write((char*)temp, sizeof(char));
	//				delete temp;
	//			}
	//			else if (i == total_num_cameras - 1) {
	//				*temp <<= 1;
	//				*temp |= occ[i];
	//				*temp <<= 8 - (total_num_cameras % 8);
	//				fout.write((char*)temp, sizeof(char));
	//				delete temp;
	//			}
	//			else {
	//				*temp <<= 1;
	//				*temp |= occ[i];
	//			}
	//			//cout << bitset<8>(*temp) << endl;
	//		}
	//	}
	//	else if (version == 3.1) { //version == 2.6
	//		uchar refV = (*vit)->GetrefV();
	//		uchar refU = (*vit)->GetrefU();
	//		fout.write((char*)&refV, sizeof(uchar));
	//		fout.write((char*)&refU, sizeof(uchar));

	//		vector<uchar> VU = (*vit)->GetVU();
	//		for (int i = 0; i < total_num_cameras; i++) {
	//			fout.write((char*)&VU[i], sizeof(uchar));
	//		}

	//		MatrixXf f;
	//		if (degree == 2) {
	//			//f.resize(3, 1);
	//			f = (*vit)->GetCoef();
	//			float ff1, ff2, ff3;
	//			ff1 = f(0, 0);
	//			fout.write((char*)&ff1, sizeof(float));
	//			ff2 = f(1, 0);
	//			fout.write((char*)&ff2, sizeof(float));
	//			ff3 = f(2, 0);
	//			fout.write((char*)&ff3, sizeof(float));
	//		}
	//		else if (degree == 3) {
	//			//f.resize(4, 1);
	//			f = (*vit)->GetCoef();
	//			float ff1, ff2, ff3, ff4;
	//			ff1 = f(0, 0);
	//			fout.write((char*)&ff1, sizeof(float));
	//			ff2 = f(1, 0);
	//			fout.write((char*)&ff2, sizeof(float));
	//			ff3 = f(2, 0);
	//			fout.write((char*)&ff3, sizeof(float));
	//			ff4 = f(3, 0);
	//			fout.write((char*)&ff4, sizeof(float));
	//		}

	//	}
	//	else if (version == 3.2) {
	//		uchar avrV = (*vit)->GetV();
	//		uchar avrU = (*vit)->GetU();
	//		fout.write((char*)&avrV, sizeof(uchar));
	//		fout.write((char*)&avrU, sizeof(uchar));

	//		MatrixXf f;
	//		if (degree == 2) {
	//			//f.resize(3, 1);
	//			f = (*vit)->GetCoef();
	//			float ff1, ff2, ff3;
	//			ff1 = f(0, 0);
	//			fout.write((char*)&ff1, sizeof(float));
	//			ff2 = f(1, 0);
	//			fout.write((char*)&ff2, sizeof(float));
	//			ff3 = f(2, 0);
	//			fout.write((char*)&ff3, sizeof(float));
	//		}
	//		else if (degree == 3) {
	//			//f.resize(4, 1);
	//			f = (*vit)->GetCoef();
	//			float ff1, ff2, ff3, ff4;
	//			ff1 = f(0, 0);
	//			fout.write((char*)&ff1, sizeof(float));
	//			ff2 = f(1, 0);
	//			fout.write((char*)&ff2, sizeof(float));
	//			ff3 = f(2, 0);
	//			fout.write((char*)&ff3, sizeof(float));
	//			ff4 = f(3, 0);
	//			fout.write((char*)&ff4, sizeof(float));
	//		}
	//		

	//		vector<bool> occ = (*vit)->GetOcclusion();
	//		for (int i = 0; i < total_num_cameras; i++) {
	//			char* temp;
	//			if (i % 8 == 0) {
	//				temp = new char;
	//				*temp = occ[i];
	//			}
	//			else if (i % 8 == 7) {
	//				*temp <<= 1;
	//				*temp |= occ[i];
	//				fout.write((char*)temp, sizeof(char));
	//				delete temp;
	//			}
	//			else if (i == total_num_cameras - 1) {
	//				*temp <<= 1;
	//				*temp |= occ[i];
	//				*temp <<= 8 - (total_num_cameras % 8);
	//				fout.write((char*)temp, sizeof(char));
	//				delete temp;
	//			}
	//			else {
	//				*temp <<= 1;
	//				*temp |= occ[i];
	//			}
	//			//cout << bitset<8>(*temp) << endl;
	//		}
	//	}
	//	
	//}

	fout.close();
	cout << "save pcc done..." << endl;
}

vector<PPC*> load_ppc(string filename) {
	vector<PPC*> vec_ppc;
	ifstream fin(filename, ios::binary);

	//while (!fin.eof()) {
	//	PPC* pc;
	//	if (version == 2.) pc = new PPC_v2_1;
	//	else if (version == 2.2) pc = new PPC_v2_2;
	//	else if (version == 2.5) pc = new PPC_v3_2;

	//	float* geo = (float*)malloc(3 * sizeof(float));
	//	fin.read((char*)(&geo[0]), sizeof(float));
	//	fin.read((char*)(&geo[1]), sizeof(float));
	//	fin.read((char*)(&geo[2]), sizeof(float));
	//	pc->SetGeometry(geo);

	//	if (version == 2.1) {
	//		uchar refV, refU;
	//		fin.read((char*)(&refV), sizeof(uchar));
	//		fin.read((char*)(&refU), sizeof(uchar));

	//		vector<uchar> VU(total_num_cameras), Y(total_num_cameras);
	//		for (int i = 0; i < total_num_cameras; i++) {
	//			fin.read((char*)&VU[i], sizeof(uchar));
	//		}
	//		for (int i = 0; i < total_num_cameras; i++) {
	//			fin.read((char*)&Y[i], sizeof(uchar));
	//		}
	//		pc->SetColor(refV, refU, VU, Y);
	//	}
	//	else if (version == 2.2) {
	//		uchar avrV, avrU;
	//		fin.read((char*)(&avrV), sizeof(uchar));
	//		fin.read((char*)(&avrU), sizeof(uchar));

	//		vector<uchar> Y(total_num_cameras);
	//		vector<bool> occlusion(total_num_cameras);

	//		for (int i = 0; i < total_num_cameras; i++) {
	//			fin.read((char*)&Y[i], sizeof(uchar));
	//		}

	//		for (int i = 0; i < total_num_cameras; i++) {
	//			char* temp;
	//			if (i % 8 == 0) {
	//				temp = new char;
	//				fin.read((char*)temp, sizeof(char));
	//				unsigned char t = *temp & 128; //1000 0000
	//				if (t == 128) {
	//					occlusion[i] = true;
	//				}
	//				else occlusion[i] = false;
	//				*temp <<= 1;
	//			}
	//			else if (i % 8 == 7 || i == total_num_frames - 1) {
	//				unsigned char t = *temp & 128; //1000 0000
	//				if (t == 128) {
	//					occlusion[i] = true;
	//				}
	//				else occlusion[i] = false;
	//				delete temp;
	//			}
	//			else {
	//				unsigned char t = *temp & 128; //1000 0000
	//				if (t == 128) {
	//					occlusion[i] = true;
	//				}
	//				else occlusion[i] = false;
	//				*temp <<= 1;
	//			}
	//		}
	//		pc->SetColor(avrV, avrU, Y, occlusion);
	//	}
	//	else if (version == 3.1) {
	//		uchar refV, refU;
	//		fin.read((char*)(&refV), sizeof(uchar));
	//		fin.read((char*)(&refU), sizeof(uchar));

	//		vector<uchar> VU(total_num_cameras);
	//		for (int i = 0; i < total_num_cameras; i++) {
	//			fin.read((char*)&VU[i], sizeof(uchar));
	//		}

	//		MatrixXf coef;
	//		if (degree == 2) {
	//			coef.resize(3, 1);

	//			float f1, f2, f3;
	//			fin.read((char*)(&f1), sizeof(float));
	//			fin.read((char*)(&f2), sizeof(float));
	//			fin.read((char*)(&f3), sizeof(float));

	//			coef(0, 0) = f1;
	//			coef(1, 0) = f2;
	//			coef(2, 0) = f3;
	//		}
	//		else if (degree == 3) {
	//			coef.resize(4, 1);

	//			float f1, f2, f3, f4;
	//			fin.read((char*)(&f1), sizeof(float));
	//			fin.read((char*)(&f2), sizeof(float));
	//			fin.read((char*)(&f3), sizeof(float));
	//			fin.read((char*)(&f4), sizeof(float));

	//			coef(0, 0) = f1;
	//			coef(1, 0) = f2;
	//			coef(2, 0) = f3;
	//			coef(3, 0) = f4;

	//		}

	//		pc->SetColor(refV, refU, VU, coef);
	//	}
	//	else if (version == 3.2) {
	//		uchar avrV, avrU;
	//		fin.read((char*)(&avrV), sizeof(uchar));
	//		fin.read((char*)(&avrU), sizeof(uchar));

	//		MatrixXf coef;
	//		if (degree == 2) {
	//			coef.resize(3, 1);
	//			float f1, f2, f3;
	//			fin.read((char*)(&f1), sizeof(float));
	//			fin.read((char*)(&f2), sizeof(float));
	//			fin.read((char*)(&f3), sizeof(float));

	//			coef(0, 0) = f1;
	//			coef(1, 0) = f2;
	//			coef(2, 0) = f3;
	//		}
	//		else if (degree == 3) {
	//			coef.resize(4, 1);

	//			float f1, f2, f3, f4;
	//			fin.read((char*)(&f1), sizeof(float));
	//			fin.read((char*)(&f2), sizeof(float));
	//			fin.read((char*)(&f3), sizeof(float));
	//			fin.read((char*)(&f4), sizeof(float));

	//			coef(0, 0) = f1;
	//			coef(1, 0) = f2;
	//			coef(2, 0) = f3;
	//			coef(3, 0) = f4;
	//		}

	//		vector<bool> occlusion(total_num_cameras);
	//		for (int i = 0; i < total_num_cameras; i++) {
	//			char* temp;
	//			if (i % 8 == 0) {
	//				temp = new char;
	//				fin.read((char*)temp, sizeof(char));
	//				unsigned char t = *temp & 128; //1000 0000
	//				if (t == 128) {
	//					occlusion[i] = true;
	//				}
	//				else occlusion[i] = false;
	//				*temp <<= 1;
	//			}
	//			else if (i % 8 == 7 || i == total_num_frames - 1) {
	//				unsigned char t = *temp & 128; //1000 0000
	//				if (t == 128) {
	//					occlusion[i] = true;
	//				}
	//				else occlusion[i] = false;
	//				delete temp;
	//			}
	//			else {
	//				unsigned char t = *temp & 128; //1000 0000
	//				if (t == 128) {
	//					occlusion[i] = true;
	//				}
	//				else occlusion[i] = false;
	//				*temp <<= 1;
	//			}
	//		}
	//		pc->SetColor(avrV, avrU, coef, occlusion);
	//	}
	//	vec_ppc.push_back(pc);
	//}
	//
	fin.close();
	cout << "load pcc done..." << endl;
	return vec_ppc;
}
