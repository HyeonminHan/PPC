//#include "ppc_prime.h"
//
//
///*PPC proj*/
//double model_function(double input_I, double cos_theta)
//{
//	double output_I = input_I * 1 / cos_theta;
//	return output_I;
//}
//
///*PPC proj*/
//double model_inv_function(double input_I, double cos_theta)
//{
//	double output_I = input_I * cos_theta;
//	return output_I;
//}
//
////in: normal_PP[G,I0...In], PP[G, C0...Cn], cos_theta
//PPC_N make_err_vector(PPC input_norm_PP, PPC input_PP, vector<double> cos_theta)
//{
//	PPC_N output_PPC_N;//[G, I_Norm]
//	output_PPC_N.geometry[0] = input_PP.geometry[0];//G
//	output_PPC_N.geometry[1] = input_PP.geometry[1];
//	output_PPC_N.geometry[2] = input_PP.geometry[2];
//
//	short I_N_R = 0;
//	short I_N_G = 0;
//	short I_N_B = 0;
//	int cnt = 0;
//	for (int i = 0; i < total_num_cameras; i++)
//	{
//		if (input_norm_PP.color[i][0] != SHRT_MAX && input_norm_PP.color[i][1] != SHRT_MAX && input_norm_PP.color[i][2] != SHRT_MAX)
//		{
//			I_N_R += input_norm_PP.color[i][0];
//			I_N_G += input_norm_PP.color[i][1];
//			I_N_B += input_norm_PP.color[i][2];
//			cnt++;
//		}
//		else
//		{
//			I_N_R += 0;
//			I_N_G += 0;
//			I_N_B += 0;
//		}
//	}
//
//	//I_Norm
//	output_PPC_N.norm_color[0] = I_N_R / cnt;
//	output_PPC_N.norm_color[1] = I_N_R / cnt;
//	output_PPC_N.norm_color[2] = I_N_R / cnt;
//
//	cnt = 0;
//	I_N_R = 0;
//	I_N_G = 0;
//	I_N_B = 0;
//
//	//make err PPC_N     
//	short err_R = 0;
//	short err_G = 0;
//	short err_B = 0;
//	for (int idx = 0; idx < total_num_cameras; idx++)
//	{
//		//[C'0 ... C'n] - [C0 ... Cn]
//		if (input_norm_PP.color[idx][0] != SHRT_MAX && input_norm_PP.color[idx][1] != SHRT_MAX && input_norm_PP.color[idx][2] != SHRT_MAX)
//		{
//			err_R = (short)model_inv_function(output_PPC_N.norm_color[0], cos_theta[idx]) - input_PP.color[idx][0];//C'n-Cn
//			err_G = (short)model_inv_function(output_PPC_N.norm_color[1], cos_theta[idx]) - input_PP.color[idx][1];
//			err_B = (short)model_inv_function(output_PPC_N.norm_color[2], cos_theta[idx]) - input_PP.color[idx][2];
//		}
//		else
//		{
//			err_R = SHRT_MAX;//In-Cn
//			err_G = SHRT_MAX;
//			err_B = SHRT_MAX;
//		}
//
//		output_PPC_N.res[idx][0] = err_R;
//		output_PPC_N.res[idx][1] = err_G;
//		output_PPC_N.res[idx][2] = err_B;
//	}
//
//	return output_PPC_N;
//}
//
///*PPC proj*/
//PPC calc_plen_color(PPC input_PP, pcl::Normal Pt_norm, vector<double>& cos_T)
//{
//	//cout << "===========================================================" << endl;
//
//	PPC out_PP = input_PP;
//
//	//normal 없을 때 예외처리
//	if (_isnan(Pt_norm.normal_x) || _isnan(Pt_norm.normal_y) || _isnan(Pt_norm.normal_z))
//		return out_PP;//todo n-v가장 큰 색상값을 반환으로
//
//	vector < Matrix<double, 3, 1> > camera_center_vector;
//	Matrix<double, 3, 1> camera_center_temp;
//	for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//	{
//		camera_center_temp = -(m_CalibParams[cam_idx].m_RotMatrix.transpose()) * m_CalibParams[cam_idx].m_Trans;
//		camera_center_vector.push_back(camera_center_temp);
//		//cout << "camera_center:: \n idx:: " << cam_idx << "    ,  \n" << camera_center_vector[cam_idx] << endl;
//		//cout << "\n" << endl;
//	}
//
//
//	Matrix<double, 3, 1> Pt_geo_vec;
//	Pt_geo_vec << input_PP.geometry[0], input_PP.geometry[1], input_PP.geometry[2];
//	//cout << "Pt_geo:: " << input_PP.geometry[0] << " " << input_PP.geometry[1] << " " << input_PP.geometry[2] << endl;
//	//cout << "Pt_geo_vec:: " << Pt_geo_vec << endl;
//	//cout << "\n" << endl;
//
//
//	Matrix<double, 3, 1> Pt_norm_vec;
//	Pt_norm_vec << Pt_norm.normal_x, Pt_norm.normal_y, Pt_norm.normal_z;
//	//cout << "Pt_norm:: " << Pt_norm.normal_x << " " << Pt_norm.normal_y << " " << Pt_norm.normal_z << endl;
//	//cout << "Pt_norm_vec:: " << Pt_norm_vec << endl;
//	//cout << "\n" << endl;
//
//
//	vector < Matrix<double, 3, 1> > ray_vector;
//	Matrix<double, 3, 1> ray_temp;
//	for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//	{
//		ray_temp = camera_center_vector[cam_idx] - Pt_geo_vec;
//		ray_vector.push_back(ray_temp);
//		//cout << "ray_vector:: " << cam_idx << "    ,   \n" << ray_vector[cam_idx] << endl;
//		//cout << "\n" << endl;
//	}
//
//	//angle, calc cos_theta
//	vector<double> cos_theta_vector;
//	double cos_theta, cos_theta_temp, cos_theta_temp_inv;
//	double theta, theta_temp, theta_temp_inv;
//	for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//	{
//		//cos_theta, theta
//		cos_theta_temp = (Pt_norm_vec.dot(ray_vector[cam_idx])) / (Pt_norm_vec.norm() * ray_vector[cam_idx].norm());
//		cos_theta_temp_inv = (-1 * Pt_norm_vec).dot(ray_vector[cam_idx]) / (Pt_norm_vec.norm() * ray_vector[cam_idx].norm());
//
//		theta_temp = acos(cos_theta_temp);
//		theta_temp_inv = acos(cos_theta_temp_inv);
//
//		//select diredction
//		theta = min(theta_temp, theta_temp_inv);
//		cos_theta = cos(theta);/*!!!*/
//
//		cos_theta_vector.push_back(cos_theta_temp);
//
//		//cout << "cos_theta_temp:: " << "    ,   \n" << cos_theta_temp << "\t/\t" << cos_theta_temp_inv << endl;
//		//cout << "theta:: " << "    ,   \n" << theta_temp * 180 / 3.14159 << "\t/\t" << theta_temp_inv * 180 / 3.14159 << endl;
//		//cout << "selected cos_theta_vector:: " << cam_idx << "    ,   \n[" << cos_theta_vector[cam_idx] << "]" << endl;
//
//		//cout << "\n" << endl;
//	}
//	cos_T = cos_theta_vector;
//
//	double temp_R, temp_G, temp_B;
//	for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//	{
//		if (input_PP.color[cam_idx][0] != SHRT_MAX && input_PP.color[cam_idx][1] != SHRT_MAX && input_PP.color[cam_idx][2] != SHRT_MAX)
//		{
//			temp_R = model_function(input_PP.color[cam_idx][0], cos_theta_vector[cam_idx]);
//			temp_G = model_function(input_PP.color[cam_idx][1], cos_theta_vector[cam_idx]);
//			temp_B = model_function(input_PP.color[cam_idx][2], cos_theta_vector[cam_idx]);
//
//			/* No Color threshold*/
//			   //out_PP.color[cam_idx][0] = short(temp_R);
//			   //out_PP.color[cam_idx][1] = short(temp_G);
//			   //out_PP.color[cam_idx][2] = short(temp_B);
//
//
//			/* Color threshold */
//			if (temp_R >= 255)
//				out_PP.color[cam_idx][0] = 255;
//			else if (temp_R <= -255)
//				out_PP.color[cam_idx][0] = -255;
//			else
//				out_PP.color[cam_idx][0] = (temp_R);
//
//			if (temp_G >= 255)
//				out_PP.color[cam_idx][1] = 255;
//			else if (temp_G <= -255)
//				out_PP.color[cam_idx][1] = 255;
//			else
//				out_PP.color[cam_idx][1] = (temp_G);
//
//			if (temp_B >= 255)
//				out_PP.color[cam_idx][2] = 255;
//			else if (temp_B <= -255)
//				out_PP.color[cam_idx][2] = -255;
//			else
//				out_PP.color[cam_idx][2] = (temp_B);
//		}
//		else
//		{
//			out_PP.color[cam_idx][0] = SHRT_MAX;
//			out_PP.color[cam_idx][1] = SHRT_MAX;
//			out_PP.color[cam_idx][2] = SHRT_MAX;
//		}
//
//
//
//		//cout << "index:; " << cam_idx << "~~" << endl;
//		//cout << "out_PP.rgb:: " << double(out_PP.color[cam_idx][0]) << " " << double(out_PP.color[cam_idx][1]) << " " << double(out_PP.color[cam_idx][2]) << endl;
//		//cout << "ture.rgb:: " << double(input_PP.color[cam_idx][0]) * (1 / cos_theta_vector[cam_idx]) << " " << double(input_PP.color[cam_idx][1]) * (1 / cos_theta_vector[cam_idx]) << " " << double(input_PP.color[cam_idx][2]) * (1 / cos_theta_vector[cam_idx]) << endl;
//		//cout << "input_PP.rgb:: " << double(input_PP.color[cam_idx][0]) << " " << double(input_PP.color[cam_idx][1]) << " " << double(input_PP.color[cam_idx][2]) << endl;
//		//cout << "\n" << endl;
//	}
//
//
//	return out_PP;
//	//cout << "===========================================================" << endl;
//
//}
//
//// input:: PPC_N -> output:: PPC(normed color)
//vector<PPC_N> Encode(vector<PPC> input_PPC, PointCloud<pcl::Normal>::Ptr cloud_normal, vector<vector<double>>& cos_T_vector)
//{
//	//out:: [G, I0, ... In], vec<cos_theta>
//	vector<vector<double>> cos_T_PPC;
//	vector<double> cos_T_PP;
//
//	vector<PPC> output_PPC;
//	PPC PPC_temp;
//
//	for (int idx = 0; idx < input_PPC.size(); idx++)
//	{
//		//calc_PPC_N_pseudo      vs      calc_PPC_N_pseudo_each
//		PPC_temp = calc_plen_color(input_PPC[idx], cloud_normal->points[idx], cos_T_PP);//i-th norm
//		output_PPC.push_back(PPC_temp);
//		cos_T_PPC.push_back(cos_T_PP);
//	}
//	cos_T_vector = cos_T_PPC;
//
//
//
//	//out:: [G, I_Norm, (C'0-C0), ... (C'n-Cn))
//	vector<PPC_N> output_PPC_N;
//	PPC_N PPC_N_temp;
//
//	for (int idx = 0; idx < input_PPC.size(); idx++)
//	{
//		PPC_N_temp = make_err_vector(output_PPC[idx], input_PPC[idx], cos_T_PPC[idx]);
//		output_PPC_N.push_back(PPC_N_temp);
//	}
//
//
//	return output_PPC_N;
//}
//
//void print_class_PPC_N2PPC(vector<PPC_N> input_PPC_N)
//{
//	cout << "=====print_class_PPC_N2PPC=====" << endl;
//	double cl_1_R = 0, cl_2_R = 0, cl_3_R = 0, cl_4_R = 0, cl_5_R = 0, cl_6_R = 0;
//	double cl_1_G = 0, cl_2_G = 0, cl_3_G = 0, cl_4_G = 0, cl_5_G = 0, cl_6_G = 0;
//	double cl_1_B = 0, cl_2_B = 0, cl_3_B = 0, cl_4_B = 0, cl_5_B = 0, cl_6_B = 0;
//
//	int c0 = 0;
//	int c1 = 4;
//	int c2 = 8;
//	int c3 = 12;
//	int c4 = 16;
//	int c5 = 20;
//
//	//RGB
//	double temp_R = 0, avg_R = 0;
//	double temp_G = 0, avg_G = 0;
//	double temp_B = 0, avg_B = 0;
//	int cnt = 0;
//	for (int idx = 0; idx < input_PPC_N.size(); idx++)
//	{
//		for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//		{
//			if (input_PPC_N[idx].res[cam_idx][0] != SHRT_MAX && input_PPC_N[idx].res[cam_idx][1] != SHRT_MAX && input_PPC_N[idx].res[cam_idx][2] != SHRT_MAX)
//			{
//				temp_R += abs(input_PPC_N[idx].res[cam_idx][0]);
//				temp_G += abs(input_PPC_N[idx].res[cam_idx][1]);
//				temp_B += abs(input_PPC_N[idx].res[cam_idx][2]);
//				cnt++;
//			}
//			else
//			{
//				temp_R += 0;
//				temp_G += 0;
//				temp_B += 0;
//			}
//		}
//		avg_R = temp_R / cnt;
//		avg_G = temp_G / cnt;
//		avg_B = temp_B / cnt;
//		//cout << "avg_R:: " << avg_R* cnt << endl;
//		//cout << "avg_G:: " << avg_G* cnt << endl;
//		//cout << "avg_B:: " << avg_B* cnt << endl;
//
//
//
//		//R
//		if (c0 <= avg_R && avg_R < c1)
//		{
//			cl_1_R = cl_1_R + 1;
//		}
//		else if (c1 <= avg_R && avg_R < c2)
//		{
//			cl_2_R = cl_2_R + 1;
//		}
//		else if (c2 <= avg_R && avg_R < c3)
//		{
//			cl_3_R = cl_3_R + 1;
//		}
//		else if (c3 <= avg_R && avg_R < c4)
//		{
//			cl_4_R = cl_4_R + 1;
//		}
//		else if (c4 <= avg_R && avg_R < c5)
//		{
//			cl_5_R = cl_5_R + 1;
//		}
//		else if (100 <= avg_R)
//		{
//			cl_6_R = cl_6_R + 1;
//		}
//
//		//G
//		if (c0 <= avg_G && avg_G < c1)
//		{
//			cl_1_G = cl_1_G + 1;
//		}
//		else if (c1 <= avg_G && avg_G < c2)
//		{
//			cl_2_G = cl_2_G + 1;
//		}
//		else if (c2 <= avg_G && avg_G < c3)
//		{
//			cl_3_G = cl_3_G + 1;
//		}
//		else if (c3 <= avg_G && avg_G < c4)
//		{
//			cl_4_G = cl_4_G + 1;
//		}
//		else if (c4 <= avg_G && avg_G < c5)
//		{
//			cl_5_G = cl_5_G + 1;
//		}
//		else if (c5 <= avg_G)
//		{
//			cl_6_G = cl_6_G + 1;
//		}
//
//		//B
//		if (c0 <= avg_B && avg_B < c1)
//		{
//			cl_1_B = cl_1_B + 1;
//		}
//		else if (c1 <= avg_B && avg_B < c2)
//		{
//			cl_2_B = cl_2_B + 1;
//		}
//		else if (c2 <= avg_B && avg_B < c3)
//		{
//			cl_3_B = cl_3_B + 1;
//		}
//		else if (c3 <= avg_B && avg_B < c4)
//		{
//			cl_4_B = cl_4_B + 1;
//		}
//		else if (c4 <= avg_B && avg_B < c5)
//		{
//			cl_5_B = cl_5_B + 1;
//		}
//		else if (c5 <= avg_B)
//		{
//			cl_6_B = cl_6_B + 1;
//		}
//
//		avg_R = 0; temp_R = 0;
//		avg_G = 0; temp_G = 0;
//		avg_B = 0; temp_B = 0;
//		cnt = 0;
//	}
//
//	double Num_R = cl_1_R + cl_2_R + cl_3_R + cl_4_R + cl_5_R + cl_6_R;
//	double Num_G = cl_1_G + cl_2_G + cl_3_G + cl_4_G + cl_5_G + cl_6_G;
//	double Num_B = cl_1_B + cl_2_B + cl_3_B + cl_4_B + cl_5_B + cl_6_B;
//
//
//	cout << "[" << c0 << "~" << c1 << "] R: " << cl_1_R << "(" << cl_1_R * 100 / Num_R << "%)" << "  G: " << cl_1_G << "(" << cl_1_G / Num_G * 100 << "%)" << "  B: " << cl_1_B << "(" << cl_1_B / Num_B * 100 << "%)" << endl;
//	cout << "[" << c1 << "~" << c2 << "] R: " << cl_2_R << "(" << cl_2_R * 100 / Num_R << "%)" << "  G: " << cl_2_G << "(" << cl_2_G / Num_G * 100 << "%)" << "  B: " << cl_2_B << "(" << cl_2_B / Num_B * 100 << "%)" << endl;
//	cout << "[" << c2 << "~" << c3 << "] R: " << cl_3_R << "(" << cl_3_R * 100 / Num_R << "%)" << "  G: " << cl_3_G << "(" << cl_3_G / Num_G * 100 << "%)" << "  B: " << cl_3_B << "(" << cl_3_B / Num_B * 100 << "%)" << endl;
//	cout << "[" << c3 << "~" << c4 << "] R: " << cl_4_R << "(" << cl_4_R * 100 / Num_R << "%)" << "  G: " << cl_4_G << "(" << cl_4_G / Num_G * 100 << "%)" << "  B: " << cl_4_B << "(" << cl_4_B / Num_B * 100 << "%)" << endl;
//	cout << "[" << c4 << "~" << c5 << "] R: " << cl_5_R << "(" << cl_5_R * 100 / Num_R << "%)" << "  G: " << cl_5_G << "(" << cl_5_G / Num_G * 100 << "%)" << "  B: " << cl_5_B << "(" << cl_5_B / Num_B * 100 << "%)" << endl;
//	cout << "[" << c5 << "~] R: " << cl_6_R << "(" << cl_6_R * 100 / Num_R << "%)" << "  G: " << cl_6_G << "(" << cl_6_G / Num_G * 100 << "%)" << "  B: " << cl_6_B << "(" << cl_6_B / Num_B * 100 << "%)" << endl;
//	cout << "[" << c0 << "~" << c5 << "](%) R: " << 100 - cl_6_R * 100 / Num_R << "%)" << "  G: " << 100 - cl_6_G * 100 / Num_G << "%)" << "  B: " << 100 - cl_6_B * 100 / Num_B << "%)" << endl;
//
//	cout << "\nNum R: " << Num_R << endl;
//	cout << "Num G: " << Num_G << endl;
//	cout << "Num B: " << Num_B << endl << endl;
//
//
//}
//
//void print_class_PPC2PPC(vector<PPC> input_PPC_A, vector<PPC> input_PPC_B)
//{
//	cout << "=====print_class_PPC2PPC=====" << endl;
//	double cl_1_R = 0, cl_2_R = 0, cl_3_R = 0, cl_4_R = 0, cl_5_R = 0, cl_6_R = 0;
//	double cl_1_G = 0, cl_2_G = 0, cl_3_G = 0, cl_4_G = 0, cl_5_G = 0, cl_6_G = 0;
//	double cl_1_B = 0, cl_2_B = 0, cl_3_B = 0, cl_4_B = 0, cl_5_B = 0, cl_6_B = 0;
//
//	//RGB
//	double temp_R = 0, avg_R = 0;
//	double temp_G = 0, avg_G = 0;
//	double temp_B = 0, avg_B = 0;
//	int cnt = 0;
//	for (int idx = 0; idx < input_PPC_A.size(); idx++)
//	{
//		//C0 - C0
//		for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//		{
//			if (input_PPC_A[idx].color[cam_idx][0] != SHRT_MAX && input_PPC_A[idx].color[cam_idx][1] != SHRT_MAX && input_PPC_A[idx].color[cam_idx][2] != SHRT_MAX)
//			{
//				temp_R += abs(input_PPC_A[idx].color[cam_idx][0] - input_PPC_B[idx].color[cam_idx][0]);
//				temp_G += abs(input_PPC_A[idx].color[cam_idx][1] - input_PPC_B[idx].color[cam_idx][1]);
//				temp_B += abs(input_PPC_A[idx].color[cam_idx][2] - input_PPC_B[idx].color[cam_idx][2]);
//				cnt++;
//			}
//			else
//			{
//				temp_R += 0;
//				temp_G += 0;
//				temp_B += 0;
//			}
//		}
//		avg_R = temp_R / cnt;
//		avg_G = temp_G / cnt;
//		avg_B = temp_B / cnt;
//
//		//R
//		if (0 == avg_R)
//		{
//			cl_1_R = cl_1_R + 1;
//		}
//		else if (0 < avg_R && avg_R < 1)
//		{
//			cl_2_R = cl_2_R + 1;
//		}
//		else if (1 <= avg_R && avg_R < 2)
//		{
//			cl_3_R = cl_3_R + 1;
//		}
//		else if (2 <= avg_R && avg_R < 4)
//		{
//			cl_4_R = cl_4_R + 1;
//		}
//		else if (4 <= avg_R && avg_R < 8)
//		{
//			cl_5_R = cl_5_R + 1;
//		}
//		else if (8 <= avg_R)
//		{
//			cl_6_R = cl_6_R + 1;
//		}
//
//		//G
//		if (0 == avg_G)
//		{
//			cl_1_G = cl_1_G + 1;
//		}
//		else if (0 < avg_G && avg_G < 1)
//		{
//			cl_2_G = cl_2_G + 1;
//		}
//		else if (1 <= avg_G && avg_G < 2)
//		{
//			cl_3_G = cl_3_G + 1;
//		}
//		else if (2 <= avg_G && avg_G < 4)
//		{
//			cl_4_G = cl_4_G + 1;
//		}
//		else if (4 <= avg_G && avg_G < 8)
//		{
//			cl_5_G = cl_5_G + 1;
//		}
//		else if (8 <= avg_G)
//		{
//			cl_6_G = cl_6_G + 1;
//		}
//
//		//B
//		if (0 == avg_B)
//		{
//			cl_1_B = cl_1_B + 1;
//		}
//		else if (0 < avg_B && avg_B < 1)
//		{
//			cl_2_B = cl_2_B + 1;
//		}
//		else if (1 <= avg_B && avg_B < 2)
//		{
//			cl_3_B = cl_3_B + 1;
//		}
//		else if (2 <= avg_B && avg_B < 4)
//		{
//			cl_4_B = cl_4_B + 1;
//		}
//		else if (4 <= avg_B && avg_B < 8)
//		{
//			cl_5_B = cl_5_B + 1;
//		}
//		else if (8 <= avg_B)
//		{
//			cl_6_B = cl_6_B + 1;
//		}
//
//		avg_R = 0; temp_R = 0;
//		avg_G = 0; temp_G = 0;
//		avg_B = 0; temp_B = 0;
//		cnt = 0;
//	}
//
//	double Num_R = cl_1_R + cl_2_R + cl_3_R + cl_4_R + cl_5_R + cl_6_R;
//	double Num_G = cl_1_G + cl_2_G + cl_3_G + cl_4_G + cl_5_G + cl_6_G;
//	double Num_B = cl_1_B + cl_2_B + cl_3_B + cl_4_B + cl_5_B + cl_6_B;
//
//
//	cout << "[0~20] R: " << cl_1_R << "(" << cl_1_R * 100 / Num_R << "%)" << "  G: " << cl_1_G << "(" << cl_1_G / Num_G * 100 << "%)" << "  B: " << cl_1_B << "(" << cl_1_B / Num_B * 100 << "%)" << endl;
//	cout << "[20~40] R: " << cl_2_R << "(" << cl_2_R * 100 / Num_R << "%)" << "  G: " << cl_2_G << "(" << cl_2_G / Num_G * 100 << "%)" << "  B: " << cl_2_B << "(" << cl_2_B / Num_B * 100 << "%)" << endl;
//	cout << "[40~60] R: " << cl_3_R << "(" << cl_3_R * 100 / Num_R << "%)" << "  G: " << cl_3_G << "(" << cl_3_G / Num_G * 100 << "%)" << "  B: " << cl_3_B << "(" << cl_3_B / Num_B * 100 << "%)" << endl;
//	cout << "[60~80] R: " << cl_4_R << "(" << cl_4_R * 100 / Num_R << "%)" << "  G: " << cl_4_G << "(" << cl_4_G / Num_G * 100 << "%)" << "  B: " << cl_4_B << "(" << cl_4_B / Num_B * 100 << "%)" << endl;
//	cout << "[80~100] R: " << cl_5_R << "(" << cl_5_R * 100 / Num_R << "%)" << "  G: " << cl_5_G << "(" << cl_5_G / Num_G * 100 << "%)" << "  B: " << cl_5_B << "(" << cl_5_B / Num_B * 100 << "%)" << endl;
//	cout << "[100~] R: " << cl_6_R << "(" << cl_6_R * 100 / Num_R << "%)" << "  G: " << cl_6_G << "(" << cl_6_G / Num_G * 100 << "%)" << "  B: " << cl_6_B << "(" << cl_6_B / Num_B * 100 << "%)" << endl;
//
//	cout << "\nNum R: " << Num_R << endl;
//	cout << "Num G: " << Num_G << endl;
//	cout << "Num B: " << Num_B << endl << endl;
//
//
//}
//
//vector<PPC> Decode(vector<PPC_N> input_PPC_N, vector<vector<double>> cos_theta)
//{
//	vector<PPC> output_PPC;
//	PPC output_PP;
//
//	//iterate all points
//	for (int idx = 0; idx < input_PPC_N.size(); idx++)
//	{
//		output_PP.geometry[0] = input_PPC_N[idx].geometry[0];
//		output_PP.geometry[1] = input_PPC_N[idx].geometry[1];
//		output_PP.geometry[2] = input_PPC_N[idx].geometry[2];
//
//		//[C'0 ... C'n] - [(C'0-C0) ... (C'n-Cn)] = []
//		for (int camidx = 0; camidx < total_num_cameras; camidx++)
//		{
//			if (input_PPC_N[idx].res[camidx][0] != SHRT_MAX && input_PPC_N[idx].res[camidx][1] != SHRT_MAX && input_PPC_N[idx].res[camidx][2] != SHRT_MAX)
//			{
//				output_PP.color[camidx][0] = (short)model_inv_function(input_PPC_N[idx].norm_color[0], cos_theta[idx][camidx]) - input_PPC_N[idx].res[camidx][0];
//				output_PP.color[camidx][1] = (short)model_inv_function(input_PPC_N[idx].norm_color[1], cos_theta[idx][camidx]) - input_PPC_N[idx].res[camidx][1];
//				output_PP.color[camidx][2] = (short)model_inv_function(input_PPC_N[idx].norm_color[2], cos_theta[idx][camidx]) - input_PPC_N[idx].res[camidx][2];
//			}
//			else
//			{
//				output_PP.color[camidx][0] = SHRT_MAX;
//				output_PP.color[camidx][1] = SHRT_MAX;
//				output_PP.color[camidx][2] = SHRT_MAX;
//			}
//		}
//		output_PPC.push_back(output_PP);
//	}
//
//	return output_PPC;
//}
//
//// input:: PPC  -> ouput:: Normal
//pcl::PointCloud<pcl::Normal>::Ptr make_Normal(vector<PPC> input_PPC)
//{
//	int view_index = 1;
//
//
//	//PC저장 공간 1개 생성
//	PointCloud<PointXYZRGB>::Ptr pointclouds_norm(new PointCloud<PointXYZRGB>);
//
//	//PC 8개 생성
//	PointXYZRGB temp;
//
//	//PC 8개에 넣기 <- geo, color(order)
//	cout << "PC size:: " << input_PPC.size() << endl;
//	for (int i = 0; i < input_PPC.size(); i++)
//	{
//		//geo <- 고치기
//		temp.x = input_PPC[i].geometry[0];
//		temp.y = input_PPC[i].geometry[1];
//		temp.z = input_PPC[i].geometry[2];
//
//		//color
//		temp.r = uchar(input_PPC[i].color[view_index - 1][0]);
//		temp.g = uchar(input_PPC[i].color[view_index - 1][1]);
//		temp.b = uchar(input_PPC[i].color[view_index - 1][2]);
//
//		pointclouds_norm->points.push_back(temp);
//	}
//
//	////////////////////////////////////////////////////////////////////get normal
//	cout << "|| normal part ||" << endl;
//
//	// Create the normal estimation class, and pass the input dataset to it
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_est;
//	normal_est.setInputCloud(pointclouds_norm);
//
//	// Create an empty kdtree representation, and pass it to the normal estimation object.
//	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
//	normal_est.setSearchMethod(tree);
//
//	// Output datasets
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//
//	// Use all neighbors in a sphere of radius 
//	normal_est.setRadiusSearch(0.2);
//
//	// Compute the features
//	normal_est.compute(*cloud_normals);
//
//	int idx = 500;
//	cout << "normal size:: " << cloud_normals->points.size() << endl;
//	cout << "normal index0:: " << cloud_normals->points[idx] << endl;
//	cout << "pointclouds_norm size:: " << pointclouds_norm->points.size() << endl;
//	cout << "pointXYZRGB index0:: " << pointclouds_norm->points[idx] << endl;
//	////////////////////////////////////////////////////////////////////truncate nan normal
//
//	PointCloud<Normal>::iterator Normit = cloud_normals->points.begin();
//	PointCloud<Normal>::iterator Normend = cloud_normals->points.end();
//
//	int nan_cnt = 0;
//	for (; Normit < Normend; Normit++)
//	{
//		if (_isnan(Normit->normal_x) || _isnan(Normit->normal_y) || _isnan(Normit->normal_z))
//			nan_cnt++;
//
//		//cout << "Normit.nx:: " << Normit->normal_x << "    y:: " << Normit->normal_y << "   z:: "<< Normit->normal_z<< endl;
//	}
//	cout << "nan count:: " << nan_cnt << endl;
//	cout << "|| normal part end||" << endl;
//
//
//	return cloud_normals;
//}
