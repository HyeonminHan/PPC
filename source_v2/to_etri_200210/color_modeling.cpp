//#include "color_modeling.h"
//
//// 2nd order polynomial model
//vector<PPC2> encodeColorModelingPPC(vector<PPC>& Plen_PC) {
//	vector<PPC2> color_modeling_Plen_PC;
//
//	vector<int> x2(total_num_cameras), x3(total_num_cameras), x4(total_num_cameras);
//
//	for (int i = 0; i < total_num_cameras; i++) {
//		x2[i] = i * i;
//		x3[i] = i * x2[i];
//		x4[i] = x2[i] * x2[i];
//	}
//
//	for (int i = 0; i < Plen_PC.size(); i++) {
//		PPC2 temp_PP;
//
//		temp_PP.geometry[0] = Plen_PC[i].geometry[0];
//		temp_PP.geometry[1] = Plen_PC[i].geometry[1];
//		temp_PP.geometry[2] = Plen_PC[i].geometry[2];
//
//		bool start = false;
//		int n = 0;
//		int sum_x = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0;
//		int sum_y[3] = { 0, }, sum_xy[3] = { 0, }, sum_x2y[3] = { 0, };
//		int r_start, g_start, b_start;
//		vector<int> dummy_binary(total_num_cameras, 0);
//		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
//			// 진규가 고침.
//			if (!Plen_PC[i].color[cam_num][0] != SHRT_MAX) {
//				dummy_binary[cam_num] = 1;
//
//				int r = int(Plen_PC[i].color[cam_num][0]);
//				int g = int(Plen_PC[i].color[cam_num][1]);
//				int b = int(Plen_PC[i].color[cam_num][2]);
//
//				if (!start) {
//					start = true;
//					r_start = r;
//					g_start = g;
//					b_start = b;
//				}
//
//				sum_x += cam_num;
//				sum_x2 += x2[cam_num];
//				sum_x3 += x3[cam_num];
//				sum_x4 += x4[cam_num];
//
//				sum_y[0] += r;
//				sum_y[1] += g;
//				sum_y[2] += b;
//				sum_xy[0] += cam_num * r;
//				sum_xy[1] += cam_num * g;
//				sum_xy[2] += cam_num * b;
//				sum_x2y[0] += x2[cam_num] * r;
//				sum_x2y[1] += x2[cam_num] * g;
//				sum_x2y[2] += x2[cam_num] * b;
//
//				n++;
//			}
//		}
//
//		int dummy = 0;
//		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
//			dummy += dummy_binary[cam_num] * pow(2, total_num_cameras - cam_num - 1);
//		}
//		temp_PP.dummy = (short)dummy;
//
//		// 색이 1,2개일때는 gaussian 모델링 미적용.
//		if (n == 1) {
//			temp_PP.a0[0] = r_start;
//			temp_PP.a0[1] = g_start;
//			temp_PP.a0[2] = b_start;
//		}
//		else if (n == 2) {
//			Mat A(2, 2, CV_32F);
//			Mat b(2, 3, CV_32F);
//
//			A.at<float>(0, 0) = n;
//			A.at<float>(0, 1) = sum_x;
//			A.at<float>(1, 0) = sum_x;
//			A.at<float>(1, 1) = sum_x2;
//
//			b.at<float>(0, 0) = sum_y[0];
//			b.at<float>(0, 1) = sum_y[1];
//			b.at<float>(0, 2) = sum_y[2];
//			b.at<float>(1, 0) = sum_xy[0];
//			b.at<float>(1, 1) = sum_xy[1];
//			b.at<float>(1, 2) = sum_xy[2];
//
//			Mat X = A.inv() * b;
//
//			for (int i = 0; i < 3; i++) {
//				temp_PP.a0[i] = X.at<float>(0, i);
//				temp_PP.a1[i] = X.at<float>(1, i);
//				temp_PP.a2[i] = 0.f;
//			}
//		}
//		else {
//			Mat A(3, 3, CV_32F);
//			Mat b(3, 3, CV_32F);
//
//			A.at<float>(0, 0) = n;
//			A.at<float>(0, 1) = sum_x;
//			A.at<float>(0, 2) = sum_x2;
//			A.at<float>(1, 0) = sum_x;
//			A.at<float>(1, 1) = sum_x2;
//			A.at<float>(1, 2) = sum_x3;
//			A.at<float>(2, 0) = sum_x2;
//			A.at<float>(2, 1) = sum_x3;
//			A.at<float>(2, 2) = sum_x4;
//
//			b.at<float>(0, 0) = sum_y[0];
//			b.at<float>(0, 1) = sum_y[1];
//			b.at<float>(0, 2) = sum_y[2];
//			b.at<float>(1, 0) = sum_xy[0];
//			b.at<float>(1, 1) = sum_xy[1];
//			b.at<float>(1, 2) = sum_xy[2];
//			b.at<float>(2, 0) = sum_x2y[0];
//			b.at<float>(2, 1) = sum_x2y[1];
//			b.at<float>(2, 2) = sum_x2y[2];
//
//			Mat X = A.inv() * b;
//
//			for (int i = 0; i < 3; i++) {
//				temp_PP.a0[i] = X.at<float>(0, i);
//				temp_PP.a1[i] = X.at<float>(1, i);
//				temp_PP.a2[i] = X.at<float>(2, i);
//			}
//		}
//
//		color_modeling_Plen_PC.push_back(temp_PP);
//	}
//
//	return color_modeling_Plen_PC;
//}
//
//vector<PPC> decodeColorModelingPPC(vector<PPC2>& color_modeling_Plen_PC) {
//	vector<PPC> Plen_PC;
//
//	for (int i = 0; i < color_modeling_Plen_PC.size(); i++) {
//		PPC temp_PP;
//		temp_PP.geometry[0] = color_modeling_Plen_PC[i].geometry[0];
//		temp_PP.geometry[1] = color_modeling_Plen_PC[i].geometry[1];
//		temp_PP.geometry[2] = color_modeling_Plen_PC[i].geometry[2];
//
//		int dummy = int(color_modeling_Plen_PC[i].dummy);
//		vector<int> dummy_binary(total_num_cameras, 0);
//
//		int non_dummy_cnt = 0;
//		for (int j = total_num_cameras - 1; j >= 0; j--) {
//			if (dummy == 0) {
//				break;
//			}
//			else if (dummy == 1) {
//				dummy_binary[j] = 1;
//				non_dummy_cnt++;
//				break;
//			}
//			dummy_binary[j] = dummy % 2;
//			dummy /= 2;
//			if (dummy_binary[j] == 1) {
//				non_dummy_cnt++;
//			}
//		}
//
//		if (non_dummy_cnt == 1) {
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					temp_PP.color[j][0] = short(color_modeling_Plen_PC[i].a0[0]);
//					temp_PP.color[j][1] = short(color_modeling_Plen_PC[i].a0[1]);
//					temp_PP.color[j][2] = short(color_modeling_Plen_PC[i].a0[2]);
//				}
//				else {
//					temp_PP.color[j][0] = SHRT_MAX;
//					temp_PP.color[j][1] = SHRT_MAX;
//					temp_PP.color[j][2] = SHRT_MAX;
//				}
//			}
//		}
//		else {
//			float a0[3], a1[3], a2[3];
//			a0[0] = color_modeling_Plen_PC[i].a0[0];
//			a0[1] = color_modeling_Plen_PC[i].a0[1];
//			a0[2] = color_modeling_Plen_PC[i].a0[2];
//			a1[0] = color_modeling_Plen_PC[i].a1[0];
//			a1[1] = color_modeling_Plen_PC[i].a1[1];
//			a1[2] = color_modeling_Plen_PC[i].a1[2];
//			a2[0] = color_modeling_Plen_PC[i].a2[0];
//			a2[1] = color_modeling_Plen_PC[i].a2[1];
//			a2[2] = color_modeling_Plen_PC[i].a2[2];
//
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					short r = short(cvRound(a0[0] + a1[0] * j + a2[0] * j * j));
//					short g = short(cvRound(a0[1] + a1[1] * j + a2[1] * j * j));
//					short b = short(cvRound(a0[2] + a1[2] * j + a2[2] * j * j));
//
//					if ((short)r < 0) {
//						r = (short)0;
//					}
//					else if ((short)r > 255) {
//						r = (short)255;
//					}
//					if ((short)g < 0) {
//						g = (short)0;
//					}
//					else if ((short)g > 255) {
//						g = (short)255;
//					}
//					if ((short)b < 0) {
//						b = (short)0;
//					}
//					else if ((short)b > 255) {
//						b = (short)255;
//					}
//
//					temp_PP.color[j][0] = r;
//					temp_PP.color[j][1] = g;
//					temp_PP.color[j][2] = b;
//				}
//				else {
//					temp_PP.color[j][0] = SHRT_MAX;
//					temp_PP.color[j][1] = SHRT_MAX;
//					temp_PP.color[j][2] = SHRT_MAX;
//				}
//			}
//		}
//
//		Plen_PC.push_back(temp_PP);
//	}
//
//	return Plen_PC;
//}
//
//// Linear model
//vector<PPC3> encodeColorModelingPPC2(vector<PPC>& Plen_PC) {
//	vector<PPC3> color_modeling_Plen_PC;
//
//	vector<int> x2(total_num_cameras), x3(total_num_cameras), x4(total_num_cameras);
//
//	for (int i = 0; i < total_num_cameras; i++) {
//		x2[i] = i * i;
//	}
//
//	for (int i = 0; i < Plen_PC.size(); i++) {
//		PPC3 temp_PP;
//
//		temp_PP.geometry[0] = Plen_PC[i].geometry[0];
//		temp_PP.geometry[1] = Plen_PC[i].geometry[1];
//		temp_PP.geometry[2] = Plen_PC[i].geometry[2];
//
//		bool start = false;
//		int n = 0;
//		int sum_x = 0, sum_x2 = 0;
//		int sum_y[3] = { 0, }, sum_xy[3] = { 0, };
//		int r_start, g_start, b_start;
//		vector<int> dummy_binary(total_num_cameras, 0);
//		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
//			if (Plen_PC[i].color[cam_num][0] != SHRT_MAX) {
//				dummy_binary[cam_num] = 1;
//
//				int r = int(Plen_PC[i].color[cam_num][0]);
//				int g = int(Plen_PC[i].color[cam_num][1]);
//				int b = int(Plen_PC[i].color[cam_num][2]);
//
//				if (!start) {
//					start = true;
//					r_start = r;
//					g_start = g;
//					b_start = b;
//				}
//
//				sum_x += cam_num;
//				sum_x2 += x2[cam_num];
//
//				sum_y[0] += r;
//				sum_y[1] += g;
//				sum_y[2] += b;
//				sum_xy[0] += cam_num * r;
//				sum_xy[1] += cam_num * g;
//				sum_xy[2] += cam_num * b;
//
//				n++;
//			}
//		}
//
//		int dummy = 0;
//		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
//			dummy += dummy_binary[cam_num] * pow(2, total_num_cameras - cam_num - 1);
//		}
//		temp_PP.dummy = (ushort)dummy;
//
//		// 색이 1,2개일때는 gaussian 모델링 미적용.
//		if (n == 1) {
//			temp_PP.a0[0] = r_start;
//			temp_PP.a0[1] = g_start;
//			temp_PP.a0[2] = b_start;
//		}
//		else {
//			Mat A(2, 2, CV_32F);
//			Mat b(2, 3, CV_32F);
//
//			A.at<float>(0, 0) = n;
//			A.at<float>(0, 1) = sum_x;
//			A.at<float>(1, 0) = sum_x;
//			A.at<float>(1, 1) = sum_x2;
//
//			b.at<float>(0, 0) = sum_y[0];
//			b.at<float>(0, 1) = sum_y[1];
//			b.at<float>(0, 2) = sum_y[2];
//			b.at<float>(1, 0) = sum_xy[0];
//			b.at<float>(1, 1) = sum_xy[1];
//			b.at<float>(1, 2) = sum_xy[2];
//
//			Mat X = A.inv() * b;
//
//			for (int i = 0; i < 3; i++) {
//				temp_PP.a0[i] = X.at<float>(0, i);
//				temp_PP.a1[i] = X.at<float>(1, i);
//			}
//		}
//
//		color_modeling_Plen_PC.push_back(temp_PP);
//	}
//
//	return color_modeling_Plen_PC;
//}
//
//vector<PPC> decodeColorModelingPPC2(vector<PPC3>& color_modeling_Plen_PC) {
//	vector<PPC> Plen_PC;
//
//	for (int i = 0; i < color_modeling_Plen_PC.size(); i++) {
//		PPC temp_PP;
//		temp_PP.geometry[0] = color_modeling_Plen_PC[i].geometry[0];
//		temp_PP.geometry[1] = color_modeling_Plen_PC[i].geometry[1];
//		temp_PP.geometry[2] = color_modeling_Plen_PC[i].geometry[2];
//
//		int dummy = int(color_modeling_Plen_PC[i].dummy);
//		vector<int> dummy_binary(total_num_cameras, 0);
//
//		int non_dummy_cnt = 0;
//		for (int j = total_num_cameras - 1; j >= 0; j--) {
//			if (dummy == 0) {
//				break;
//			}
//			else if (dummy == 1) {
//				dummy_binary[j] = 1;
//				non_dummy_cnt++;
//				break;
//			}
//			dummy_binary[j] = dummy % 2;
//			dummy /= 2;
//			if (dummy_binary[j] == 1) {
//				non_dummy_cnt++;
//			}
//		}
//
//		if (non_dummy_cnt == 1) {
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					temp_PP.color[j][0] = short(color_modeling_Plen_PC[i].a0[0]);
//					temp_PP.color[j][1] = short(color_modeling_Plen_PC[i].a0[1]);
//					temp_PP.color[j][2] = short(color_modeling_Plen_PC[i].a0[2]);
//				}
//				else {
//					temp_PP.color[j][0] = SHRT_MAX;
//					temp_PP.color[j][1] = SHRT_MAX;
//					temp_PP.color[j][2] = SHRT_MAX;
//				}
//			}
//		}
//		else {
//			float a0[3], a1[3];
//			a0[0] = color_modeling_Plen_PC[i].a0[0];
//			a0[1] = color_modeling_Plen_PC[i].a0[1];
//			a0[2] = color_modeling_Plen_PC[i].a0[2];
//			a1[0] = color_modeling_Plen_PC[i].a1[0];
//			a1[1] = color_modeling_Plen_PC[i].a1[1];
//			a1[2] = color_modeling_Plen_PC[i].a1[2];
//
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					short r = short(cvRound(a0[0] + a1[0] * j));
//					short g = short(cvRound(a0[1] + a1[1] * j));
//					short b = short(cvRound(a0[2] + a1[2] * j));
//
//					if ((short)r < 0) {
//						r = (short)0;
//					}
//					else if ((short)r > 255) {
//						r = (short)255;
//					}
//					if ((short)g < 0) {
//						g = (short)0;
//					}
//					else if ((short)g > 255) {
//						g = (short)255;
//					}
//					if ((short)b < 0) {
//						b = (short)0;
//					}
//					else if ((short)b > 255) {
//						b = (short)255;
//					}
//
//					temp_PP.color[j][0] = r;
//					temp_PP.color[j][1] = g;
//					temp_PP.color[j][2] = b;
//				}
//				else {
//					temp_PP.color[j][0] = SHRT_MAX;
//					temp_PP.color[j][1] = SHRT_MAX;
//					temp_PP.color[j][2] = SHRT_MAX;
//				}
//			}
//		}
//
//		Plen_PC.push_back(temp_PP);
//	}
//
//	return Plen_PC;
//}
//
//// Diffuse(linear model) + specular(2nd order polynomial model)
//vector<PPC4> encodeColorModelingPPC3(vector<PPC>& Plen_PC) {
//	vector<PPC3> Plen_PC_linear_color_modeling = encodeColorModelingPPC2(Plen_PC);
//
//	vector<PPC4> color_modeling_Plen_PC;
//
//	vector<int> x2(total_num_cameras), x3(total_num_cameras), x4(total_num_cameras);
//
//	for (int i = 0; i < total_num_cameras; i++) {
//		x2[i] = i * i;
//		x3[i] = i * x2[i];
//		x4[i] = x2[i] * x2[i];
//	}
//
//	for (int i = 0; i < Plen_PC.size(); i++) {
//		PPC4 temp_PP;
//
//		temp_PP.geometry[0] = Plen_PC[i].geometry[0];
//		temp_PP.geometry[1] = Plen_PC[i].geometry[1];
//		temp_PP.geometry[2] = Plen_PC[i].geometry[2];
//
//		for (int j = 0; j < 3; j++) {
//			temp_PP.diffuse[j][0] = Plen_PC_linear_color_modeling[i].a0[j];
//			temp_PP.diffuse[j][1] = Plen_PC_linear_color_modeling[i].a1[j];
//		}
//
//		int dummy = int(Plen_PC_linear_color_modeling[i].dummy);
//		temp_PP.dummy = dummy;
//		vector<int> dummy_binary(total_num_cameras, 0);
//
//		int non_dummy_cnt = 0;
//		for (int j = total_num_cameras - 1; j >= 0; j--) {
//			if (dummy == 0) {
//				break;
//			}
//			else if (dummy == 1) {
//				dummy_binary[j] = 1;
//				non_dummy_cnt++;
//				break;
//			}
//			dummy_binary[j] = dummy % 2;
//			dummy /= 2;
//			if (dummy_binary[j] == 1) {
//				non_dummy_cnt++;
//			}
//		}
//
//		bool start = false;
//		int sum_x = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0;
//		int sum_y[3] = { 0, }, sum_xy[3] = { 0, }, sum_x2y[3] = { 0, };
//
//		if (non_dummy_cnt > 2) {
//			float a0[3], a1[3];
//			a0[0] = Plen_PC_linear_color_modeling[i].a0[0];
//			a0[1] = Plen_PC_linear_color_modeling[i].a0[1];
//			a0[2] = Plen_PC_linear_color_modeling[i].a0[2];
//			a1[0] = Plen_PC_linear_color_modeling[i].a1[0];
//			a1[1] = Plen_PC_linear_color_modeling[i].a1[1];
//			a1[2] = Plen_PC_linear_color_modeling[i].a1[2];
//
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					short estimated_r = short(cvRound(a0[0] + a1[0] * j));
//					short estimated_g = short(cvRound(a0[1] + a1[1] * j));
//					short estimated_b = short(cvRound(a0[2] + a1[2] * j));
//
//					int diff_r = int(Plen_PC[i].color[j][0] - estimated_r);
//					int diff_g = int(Plen_PC[i].color[j][1] - estimated_g);
//					int diff_b = int(Plen_PC[i].color[j][2] - estimated_b);
//
//					sum_x += j;
//					sum_x2 += x2[j];
//					sum_x3 += x3[j];
//					sum_x4 += x4[j];
//
//					sum_y[0] += diff_r;
//					sum_y[1] += diff_g;
//					sum_y[2] += diff_b;
//					sum_xy[0] += j * diff_r;
//					sum_xy[1] += j * diff_g;
//					sum_xy[2] += j * diff_b;
//					sum_x2y[0] += x2[j] * diff_r;
//					sum_x2y[1] += x2[j] * diff_g;
//					sum_x2y[2] += x2[j] * diff_b;
//				}
//			}
//
//			// 색이 1,2개일때는 gaussian 모델링 미적용.
//			Mat A(3, 3, CV_32F);
//			Mat b(3, 3, CV_32F);
//
//			A.at<float>(0, 0) = non_dummy_cnt;
//			A.at<float>(0, 1) = sum_x;
//			A.at<float>(0, 2) = sum_x2;
//			A.at<float>(1, 0) = sum_x;
//			A.at<float>(1, 1) = sum_x2;
//			A.at<float>(1, 2) = sum_x3;
//			A.at<float>(2, 0) = sum_x2;
//			A.at<float>(2, 1) = sum_x3;
//			A.at<float>(2, 2) = sum_x4;
//
//			b.at<float>(0, 0) = sum_y[0];
//			b.at<float>(0, 1) = sum_y[1];
//			b.at<float>(0, 2) = sum_y[2];
//			b.at<float>(1, 0) = sum_xy[0];
//			b.at<float>(1, 1) = sum_xy[1];
//			b.at<float>(1, 2) = sum_xy[2];
//			b.at<float>(2, 0) = sum_x2y[0];
//			b.at<float>(2, 1) = sum_x2y[1];
//			b.at<float>(2, 2) = sum_x2y[2];
//
//			Mat X = A.inv() * b;
//
//			for (int j = 0; j < 3; j++) {
//				temp_PP.specular[j][0] = X.at<float>(0, j);
//				temp_PP.specular[j][1] = X.at<float>(1, j);
//				temp_PP.specular[j][2] = X.at<float>(2, j);
//			}
//		}
//
//		color_modeling_Plen_PC.push_back(temp_PP);
//	}
//
//	return color_modeling_Plen_PC;
//}
//
//vector<PPC> decodeColorModelingPPC3(vector<PPC4>& color_modeling_Plen_PC) {
//	vector<PPC> Plen_PC;
//
//	for (int i = 0; i < color_modeling_Plen_PC.size(); i++) {
//		PPC temp_PP;
//		temp_PP.geometry[0] = color_modeling_Plen_PC[i].geometry[0];
//		temp_PP.geometry[1] = color_modeling_Plen_PC[i].geometry[1];
//		temp_PP.geometry[2] = color_modeling_Plen_PC[i].geometry[2];
//
//		int dummy = int(color_modeling_Plen_PC[i].dummy);
//		vector<int> dummy_binary(total_num_cameras, 0);
//
//		int non_dummy_cnt = 0;
//		for (int j = total_num_cameras - 1; j >= 0; j--) {
//			if (dummy == 0) {
//				break;
//			}
//			else if (dummy == 1) {
//				dummy_binary[j] = 1;
//				non_dummy_cnt++;
//				break;
//			}
//			dummy_binary[j] = dummy % 2;
//			dummy /= 2;
//			if (dummy_binary[j] == 1) {
//				non_dummy_cnt++;
//			}
//		}
//
//		if (non_dummy_cnt == 1) {
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					temp_PP.color[j][0] = short(color_modeling_Plen_PC[i].diffuse[0][0]);
//					temp_PP.color[j][1] = short(color_modeling_Plen_PC[i].diffuse[1][0]);
//					temp_PP.color[j][2] = short(color_modeling_Plen_PC[i].diffuse[2][0]);
//				}
//				else {
//					temp_PP.color[j][0] = SHRT_MAX;
//					temp_PP.color[j][1] = SHRT_MAX;
//					temp_PP.color[j][2] = SHRT_MAX;
//				}
//			}
//		}
//		else if (non_dummy_cnt == 2) {
//			float a0[3], a1[3];
//			a0[0] = color_modeling_Plen_PC[i].diffuse[0][0];
//			a0[1] = color_modeling_Plen_PC[i].diffuse[1][0];
//			a0[2] = color_modeling_Plen_PC[i].diffuse[2][0];
//			a1[0] = color_modeling_Plen_PC[i].diffuse[0][1];
//			a1[1] = color_modeling_Plen_PC[i].diffuse[1][1];
//			a1[2] = color_modeling_Plen_PC[i].diffuse[2][1];
//
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					short r = short(cvRound(a0[0] + a1[0] * j));
//					short g = short(cvRound(a0[1] + a1[1] * j));
//					short b = short(cvRound(a0[2] + a1[2] * j));
//
//					if ((short)r < 0) {
//						r = (short)0;
//					}
//					else if ((short)r > 255) {
//						r = (short)255;
//					}
//					if ((short)g < 0) {
//						g = (short)0;
//					}
//					else if ((short)g > 255) {
//						g = (short)255;
//					}
//					if ((short)b < 0) {
//						b = (short)0;
//					}
//					else if ((short)b > 255) {
//						b = (short)255;
//					}
//
//					temp_PP.color[j][0] = r;
//					temp_PP.color[j][1] = g;
//					temp_PP.color[j][2] = b;
//				}
//				else {
//					temp_PP.color[j][0] = SHRT_MAX;
//					temp_PP.color[j][1] = SHRT_MAX;
//					temp_PP.color[j][2] = SHRT_MAX;
//				}
//			}
//		}
//		else {
//			float diffuse[3][2] = { 0.f, }, specular[3][3] = { 0.f };
//			diffuse[0][0] = color_modeling_Plen_PC[i].diffuse[0][0];
//			diffuse[1][0] = color_modeling_Plen_PC[i].diffuse[1][0];
//			diffuse[2][0] = color_modeling_Plen_PC[i].diffuse[2][0];
//			diffuse[0][1] = color_modeling_Plen_PC[i].diffuse[0][1];
//			diffuse[1][1] = color_modeling_Plen_PC[i].diffuse[1][1];
//			diffuse[2][1] = color_modeling_Plen_PC[i].diffuse[2][1];
//
//			specular[0][0] = color_modeling_Plen_PC[i].specular[0][0];
//			specular[1][0] = color_modeling_Plen_PC[i].specular[1][0];
//			specular[2][0] = color_modeling_Plen_PC[i].specular[2][0];
//			specular[0][1] = color_modeling_Plen_PC[i].specular[0][1];
//			specular[1][1] = color_modeling_Plen_PC[i].specular[1][1];
//			specular[2][1] = color_modeling_Plen_PC[i].specular[2][1];
//			specular[0][2] = color_modeling_Plen_PC[i].specular[0][2];
//			specular[1][2] = color_modeling_Plen_PC[i].specular[1][2];
//			specular[2][2] = color_modeling_Plen_PC[i].specular[2][2];
//
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					short r = short(cvRound((diffuse[0][0] + diffuse[0][1] * j) + (specular[0][0] + specular[0][1] * j + specular[0][2] * j * j)));
//					short g = short(cvRound((diffuse[1][0] + diffuse[1][1] * j) + (specular[1][0] + specular[1][1] * j + specular[1][2] * j * j)));
//					short b = short(cvRound((diffuse[2][0] + diffuse[2][1] * j) + (specular[2][0] + specular[2][1] * j + specular[2][2] * j * j)));
//
//					if ((short)r < 0) {
//						r = (short)0;
//					}
//					else if ((short)r > 255) {
//						r = (short)255;
//					}
//					if ((short)g < 0) {
//						g = (short)0;
//					}
//					else if ((short)g > 255) {
//						g = (short)255;
//					}
//					if ((short)b < 0) {
//						b = (short)0;
//					}
//					else if ((short)b > 255) {
//						b = (short)255;
//					}
//
//					temp_PP.color[j][0] = r;
//					temp_PP.color[j][1] = g;
//					temp_PP.color[j][2] = b;
//				}
//				else {
//					temp_PP.color[j][0] = SHRT_MAX;
//					temp_PP.color[j][1] = SHRT_MAX;
//					temp_PP.color[j][2] = SHRT_MAX;
//				}
//			}
//		}
//
//		Plen_PC.push_back(temp_PP);
//	}
//
//	return Plen_PC;
//}
//
//// 2nd order polynomial model
//vector<PPC5> encodeColorModelingPPC4(vector<PPC>& Plen_PC) {
//	vector<PPC5> color_modeling_Plen_PC;
//
//	vector<int> x2(total_num_cameras), x3(total_num_cameras), x4(total_num_cameras);
//
//	for (int i = 0; i < total_num_cameras; i++) {
//		x2[i] = i * i;
//		x3[i] = i * x2[i];
//		x4[i] = x2[i] * x2[i];
//	}
//
//	for (int i = 0; i < Plen_PC.size(); i++) {
//		PPC5 temp_PP;
//
//		temp_PP.geometry[0] = Plen_PC[i].geometry[0];
//		temp_PP.geometry[1] = Plen_PC[i].geometry[1];
//		temp_PP.geometry[2] = Plen_PC[i].geometry[2];
//
//		bool start = false;
//		int n = 0;
//		int sum_x = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0;
//		int sum_y[3] = { 0, }, sum_xy[3] = { 0, }, sum_x2y[3] = { 0, };
//		int r_start, g_start, b_start;
//		vector<int> dummy_binary(total_num_cameras, 0);
//		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
//			if (Plen_PC[i].color[cam_num][0] != SHRT_MAX) {
//				dummy_binary[cam_num] = 1;
//
//				int r = int(Plen_PC[i].color[cam_num][0]);
//				int g = int(Plen_PC[i].color[cam_num][1]);
//				int b = int(Plen_PC[i].color[cam_num][2]);
//
//				if (!start) {
//					start = true;
//					r_start = r;
//					g_start = g;
//					b_start = b;
//				}
//
//				sum_x += cam_num;
//				sum_x2 += x2[cam_num];
//				sum_x3 += x3[cam_num];
//				sum_x4 += x4[cam_num];
//
//				sum_y[0] += r;
//				sum_y[1] += g;
//				sum_y[2] += b;
//				sum_xy[0] += cam_num * r;
//				sum_xy[1] += cam_num * g;
//				sum_xy[2] += cam_num * b;
//				sum_x2y[0] += x2[cam_num] * r;
//				sum_x2y[1] += x2[cam_num] * g;
//				sum_x2y[2] += x2[cam_num] * b;
//
//				n++;
//			}
//		}
//
//		int dummy = 0;
//		for (int cam_num = 0; cam_num < total_num_cameras; cam_num++) {
//			dummy += dummy_binary[cam_num] * pow(2, total_num_cameras - cam_num - 1);
//		}
//		temp_PP.dummy = (ushort)dummy;
//
//		// 색이 1,2개일때는 gaussian 모델링 미적용.
//		if (n == 1) {
//			temp_PP.a0[0] = r_start;
//			temp_PP.a0[1] = g_start;
//			temp_PP.a0[2] = b_start;
//		}
//		else if (n == 2) {
//			Mat A(2, 2, CV_32F);
//			Mat b(2, 1, CV_32F);
//
//			A.at<float>(0, 0) = n;
//			A.at<float>(0, 1) = sum_x;
//			A.at<float>(1, 0) = sum_x;
//			A.at<float>(1, 1) = sum_x2;
//
//			b.at<float>(0, 0) = float(sum_y[0] + sum_y[1] + sum_y[2]) / 3;
//			b.at<float>(1, 0) = float(sum_xy[0] + sum_xy[1] + sum_xy[2]) / 3;
//
//			Mat X = A.inv() * b;
//
//			temp_PP.a1 = X.at<float>(1, 0);
//			temp_PP.a2 = 0.f;
//
//			temp_PP.a0[0] = (sum_y[0] - temp_PP.a1*sum_x) / n;
//			temp_PP.a0[1] = (sum_y[1] - temp_PP.a1*sum_x) / n;
//			temp_PP.a0[2] = (sum_y[2] - temp_PP.a1*sum_x) / n;
//		}
//		else {
//			Mat A(3, 3, CV_32F);
//			Mat b(3, 1, CV_32F);
//
//			A.at<float>(0, 0) = n;
//			A.at<float>(0, 1) = sum_x;
//			A.at<float>(0, 2) = sum_x2;
//			A.at<float>(1, 0) = sum_x;
//			A.at<float>(1, 1) = sum_x2;
//			A.at<float>(1, 2) = sum_x3;
//			A.at<float>(2, 0) = sum_x2;
//			A.at<float>(2, 1) = sum_x3;
//			A.at<float>(2, 2) = sum_x4;
//
//			b.at<float>(0, 0) = float(sum_y[0] + sum_y[1] + sum_y[2]) / 3;
//			b.at<float>(1, 0) = float(sum_xy[0] + sum_xy[1] + sum_xy[2]) / 3;
//			b.at<float>(2, 0) = float(sum_x2y[0] + sum_x2y[1] + sum_x2y[2]) / 3;
//
//			Mat X = A.inv() * b;
//
//			temp_PP.a1 = X.at<float>(1, 0);
//			temp_PP.a2 = X.at<float>(2, 0);
//
//			temp_PP.a0[0] = (sum_y[0] - temp_PP.a1*sum_x - temp_PP.a2*sum_x2) / n;
//			temp_PP.a0[1] = (sum_y[1] - temp_PP.a1*sum_x - temp_PP.a2*sum_x2) / n;
//			temp_PP.a0[2] = (sum_y[2] - temp_PP.a1*sum_x - temp_PP.a2*sum_x2) / n;
//		}
//
//		color_modeling_Plen_PC.push_back(temp_PP);
//	}
//
//	return color_modeling_Plen_PC;
//}
//
//vector<PPC> decodeColorModelingPPC4(vector<PPC5>& color_modeling_Plen_PC) {
//	vector<PPC> Plen_PC;
//
//	for (int i = 0; i < color_modeling_Plen_PC.size(); i++) {
//		PPC temp_PP;
//		temp_PP.geometry[0] = color_modeling_Plen_PC[i].geometry[0];
//		temp_PP.geometry[1] = color_modeling_Plen_PC[i].geometry[1];
//		temp_PP.geometry[2] = color_modeling_Plen_PC[i].geometry[2];
//
//		int dummy = int(color_modeling_Plen_PC[i].dummy);
//		vector<int> dummy_binary(total_num_cameras, 0);
//
//		int non_dummy_cnt = 0;
//		for (int j = total_num_cameras - 1; j >= 0; j--) {
//			if (dummy == 0) {
//				break;
//			}
//			else if (dummy == 1) {
//				dummy_binary[j] = 1;
//				non_dummy_cnt++;
//				break;
//			}
//			dummy_binary[j] = dummy % 2;
//			dummy /= 2;
//			if (dummy_binary[j] == 1) {
//				non_dummy_cnt++;
//			}
//		}
//
//		if (non_dummy_cnt == 1) {
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					temp_PP.color[j][0] = short(color_modeling_Plen_PC[i].a0[0]);
//					temp_PP.color[j][1] = short(color_modeling_Plen_PC[i].a0[1]);
//					temp_PP.color[j][2] = short(color_modeling_Plen_PC[i].a0[2]);
//				}
//				else {
//					temp_PP.color[j][0] = SHRT_MAX;
//					temp_PP.color[j][1] = SHRT_MAX;
//					temp_PP.color[j][2] = SHRT_MAX;
//				}
//			}
//		}
//		else {
//			float a0[3], a1, a2;
//			a0[0] = color_modeling_Plen_PC[i].a0[0];
//			a0[1] = color_modeling_Plen_PC[i].a0[1];
//			a0[2] = color_modeling_Plen_PC[i].a0[2];
//			a1 = color_modeling_Plen_PC[i].a1;
//			a2 = color_modeling_Plen_PC[i].a2;
//
//			for (int j = 0; j < total_num_cameras; j++) {
//				if (dummy_binary[j] == 1) {
//					short r = short(cvRound(a0[0] + a1 * j + a2 * j * j));
//					short g = short(cvRound(a0[1] + a1 * j + a2 * j * j));
//					short b = short(cvRound(a0[2] + a1 * j + a2 * j * j));
//
//					if ((short)r < 0) {
//						r = (short)0;
//					}
//					else if ((short)r > 255) {
//						r = (short)255;
//					}
//					if ((short)g < 0) {
//						g = (short)0;
//					}
//					else if ((short)g > 255) {
//						g = (short)255;
//					}
//					if ((short)b < 0) {
//						b = (short)0;
//					}
//					else if ((short)b > 255) {
//						b = (short)255;
//					}
//
//					temp_PP.color[j][0] = r;
//					temp_PP.color[j][1] = g;
//					temp_PP.color[j][2] = b;
//				}
//				else {
//					temp_PP.color[j][0] = SHRT_MAX;
//					temp_PP.color[j][1] = SHRT_MAX;
//					temp_PP.color[j][2] = SHRT_MAX;
//				}
//			}
//		}
//
//		Plen_PC.push_back(temp_PP);
//	}
//
//	return Plen_PC;
//}