//#include "Codec_KLT.h"
//
//extern int _width, _height, total_num_cameras, total_num_frames;
//extern double MinZ, MaxZ;
//extern string path;
//extern vector<CalibStruct> m_CalibParams;
//
//using namespace pcl;
//using namespace std;
//using namespace cv;
//using namespace Eigen;
//
//
//Vec3f get_mean(PPC input_PP, int& num)
//{
//	Vec3f mean_vec;
//	int cnt = 0;
//
//	for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//	{
//		if (input_PP.color[cam_idx][0] != SHRT_MAX || input_PP.color[cam_idx][1] != SHRT_MAX || input_PP.color[cam_idx][2] != SHRT_MAX)
//		{
//			mean_vec[0] += input_PP.color[cam_idx][0];
//			mean_vec[1] += input_PP.color[cam_idx][1];
//			mean_vec[2] += input_PP.color[cam_idx][2];
//
//			cnt++;
//		}
//		else
//		{
//			mean_vec[0] += 0;
//			mean_vec[1] += 0;
//			mean_vec[2] += 0;
//		}
//	}
//
//	mean_vec[0] /= cnt;
//	mean_vec[1] /= cnt;
//	mean_vec[2] /= cnt;
//	num = cnt;
//
//	return mean_vec;
//}
//
//vector<uint> get_cnt_vec(vector<PPC> input_PPC)
//{
//	vector<uint> cnt_vec;
//	Vec3f temp_mean_color;
//	int cnt = 0;
//
//	for (int idx = 0; idx < input_PPC.size(); idx++)
//	{
//		cnt = 0;
//		temp_mean_color = get_mean(input_PPC[idx], cnt);
//		cnt_vec.push_back(cnt);
//	}
//
//	return cnt_vec;
//}
//
//vector<vector<PPC>> classify_PPC(vector<PPC> input_PPC, vector<uint> cnt_vec)
//{
//	vector<vector<PPC>> PPC_class_set(total_num_cameras);
//
//
//	for (int idx = 0; idx < input_PPC.size(); idx++)
//	{
//		for (int i = 0; i < total_num_cameras; i++) {
//			if (cnt_vec[idx] == i + 1) {
//				PPC_class_set[i].push_back(input_PPC[idx]);
//			}
//		}
//	}
//
//
//	return PPC_class_set;
//}
//
//
//vector<PPC> get_merged(vector<PPC> input_PPC, vector<vector<bool>>& PPC_flag)
//{
//	vector<PPC> merged_PPC;
//
//
//	int cnt = 0;
//	for (int idx = 0; idx < input_PPC.size(); idx++)
//	{
//		PPC temp;
//		vector<bool> temp_flag;
//		cnt = 0;
//
//		for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//		{
//			if (input_PPC[idx].color[cam_idx][0] == SHRT_MAX || input_PPC[idx].color[cam_idx][1] == SHRT_MAX || input_PPC[idx].color[cam_idx][2] == SHRT_MAX)
//			{
//				temp_flag.push_back(false);
//			}
//			else
//			{
//				temp.color[cnt][0] = input_PPC[idx].color[cam_idx][0];
//				temp.color[cnt][1] = input_PPC[idx].color[cam_idx][1];
//				temp.color[cnt][2] = input_PPC[idx].color[cam_idx][2];
//
//				temp_flag.push_back(true);
//				cnt++;
//			}
//		}
//
//		merged_PPC.push_back(temp);
//		PPC_flag.push_back(temp_flag);
//	}
//
//	return merged_PPC;
//}
//
//
//
//vector<vector< VectorXd>> KLT_trans(vector<PPC> input_PPC, int cnt, vector<MatrixXd>& cov_mat, int principal_axis)
//{
//	cout << " ============= Start KLT_trans ============= " << endl;
//
//	vector<vector<double>> mean_(3, vector<double>(cnt));
//
//	for (int cam_idx = 0; cam_idx < cnt; cam_idx++)
//	{
//		for (int idx = 0; idx < input_PPC.size(); idx++)
//		{
//			for (int chan_idx = 0; chan_idx < 3; chan_idx++)
//			{
//				mean_[chan_idx][cam_idx] += input_PPC[idx].color[cam_idx][chan_idx];
//			}
//
//		}
//
//		for (int chan_idx = 0; chan_idx < 3; chan_idx++)
//		{
//			mean_[chan_idx][cam_idx] /= input_PPC.size();
//		}
//	}
//
//
//	const int cnt_size = cnt;
//	MatrixXd cov_1(cnt_size, cnt_size);
//	MatrixXd cov_2(cnt_size, cnt_size);
//	MatrixXd cov_3(cnt_size, cnt_size);
//
//	Vec3f color_;
//
//	for (int i = 0; i < cnt_size; i++)
//	{
//		for (int j = 0; j < cnt_size; j++)
//		{
//			color_(0) = 0;
//			color_(1) = 0;
//			color_(2) = 0;
//
//			for (int idx = 0; idx < input_PPC.size(); idx++)
//			{
//				for (int chan_idx = 0; chan_idx < 3; chan_idx++)
//				{
//					color_(chan_idx) += (input_PPC[idx].color[i][chan_idx] - mean_[chan_idx][i]) * (input_PPC[idx].color[j][chan_idx] - mean_[chan_idx][j]);
//				}
//			}
//
//			if (color_(0) > DBL_MAX || color_(1) > DBL_MAX || color_(2) > DBL_MAX)
//			{
//				cout << "Max value err!" << endl;
//				break;
//			}
//
//			cov_1(i, j) = color_(0) / (input_PPC.size() - 1);
//			cov_2(i, j) = color_(1) / (input_PPC.size() - 1);
//			cov_3(i, j) = color_(2) / (input_PPC.size() - 1);
//		}
//	}
//
//	cov_mat.push_back(cov_1);
//	cov_mat.push_back(cov_2);
//	cov_mat.push_back(cov_3);
//
//
//	SelfAdjointEigenSolver<MatrixXd> eigensolver_1(cov_1);
//	SelfAdjointEigenSolver<MatrixXd> eigensolver_2(cov_2);
//	SelfAdjointEigenSolver<MatrixXd> eigensolver_3(cov_3);
//
//	MatrixXd KLT_value_1(cnt_size, cnt_size);
//	MatrixXd KLT_value_2(cnt_size, cnt_size);
//	MatrixXd KLT_value_3(cnt_size, cnt_size);
//	KLT_value_1 = eigensolver_1.eigenvalues();
//	KLT_value_2 = eigensolver_2.eigenvalues();
//	KLT_value_3 = eigensolver_3.eigenvalues();
//
//	MatrixXd KLT_1(principal_axis, cnt_size);
//	MatrixXd KLT_2(principal_axis, cnt_size);
//	MatrixXd KLT_3(principal_axis, cnt_size);
//	KLT_1 = get_KLT(eigensolver_1.eigenvectors(), principal_axis);
//	KLT_2 = get_KLT(eigensolver_2.eigenvectors(), principal_axis);
//	KLT_3 = get_KLT(eigensolver_3.eigenvectors(), principal_axis);
//
//	vector<vector< VectorXd>> ppc_prime;
//	vector<VectorXd> ppc_prime_R;
//	vector<VectorXd> ppc_prime_G;
//	vector<VectorXd> ppc_prime_B;
//	VectorXd pp_prime_R(principal_axis);
//	VectorXd pp_prime_G(principal_axis);
//	VectorXd pp_prime_B(principal_axis);
//
//	VectorXd temp_R(cnt_size);
//	VectorXd temp_G(cnt_size);
//	VectorXd temp_B(cnt_size);
//
//
//
//	for (int idx = 0; idx < input_PPC.size(); idx++)
//	{
//
//		for (int cam_idx = 0; cam_idx < cnt; cam_idx++)
//		{
//			temp_R(cam_idx) = input_PPC[idx].color[cam_idx][0];
//			temp_G(cam_idx) = input_PPC[idx].color[cam_idx][1];
//			temp_B(cam_idx) = input_PPC[idx].color[cam_idx][2];
//		}
//
//	
//		pp_prime_R = KLT_1.transpose() * temp_R;
//		pp_prime_G = KLT_2.transpose() * temp_G;
//		pp_prime_B = KLT_3.transpose() * temp_B;
//
//		ppc_prime_R.push_back(pp_prime_R);
//		ppc_prime_G.push_back(pp_prime_G);
//		ppc_prime_B.push_back(pp_prime_B);
//
//	}
//	cout << "PPC_PRIME_R size: " << ppc_prime_R.size() << " " << input_PPC.size() << endl;
//	cout << "PPC_PRIME_G size: " << ppc_prime_G.size() << " " << input_PPC.size() << endl;
//	cout << "PPC_PRIME_B size: " << ppc_prime_B.size() << " " << input_PPC.size() << endl;
//
//
//	ppc_prime.push_back(ppc_prime_R);
//	ppc_prime.push_back(ppc_prime_G);
//	ppc_prime.push_back(ppc_prime_B);
//
//	cout << " ============= End KLT_trans ============= " << endl;
//	return ppc_prime;
//}
//
//
//MatrixXd get_KLT(MatrixXd KLT, int cnt)
//{
//	MatrixXd KLT_cnt;
//
//	KLT_cnt = KLT.rightCols(cnt);
//
//	return KLT_cnt;
//}
//
//
//
//
//vector<vector< VectorXd>> decode_KLT(vector<vector< VectorXd>> Sn, vector<MatrixXd> cov_mat)
//{
//	cout << " ============= Start decode_KLT ============= " << endl;
//	vector<vector< VectorXd>> ppc_;
//
//	vector<VectorXd> ppc_R;
//	vector<VectorXd> ppc_G;
//	vector<VectorXd> ppc_B;
//	VectorXd pp_R(cov_mat[0].rows());//8*1
//	VectorXd pp_G(cov_mat[0].rows());//8*1
//	VectorXd pp_B(cov_mat[0].rows());//8*1
//
//
//	SelfAdjointEigenSolver<MatrixXd> eigensolver_1(cov_mat[0]);
//	SelfAdjointEigenSolver<MatrixXd> eigensolver_2(cov_mat[1]);
//	SelfAdjointEigenSolver<MatrixXd> eigensolver_3(cov_mat[2]);
//
//
//	MatrixXd KLT_value_1(cov_mat[0].rows(), cov_mat[0].rows());
//	MatrixXd KLT_value_2(cov_mat[0].rows(), cov_mat[0].rows());
//	MatrixXd KLT_value_3(cov_mat[0].rows(), cov_mat[0].rows());
//	KLT_value_1 = eigensolver_1.eigenvalues();
//	KLT_value_2 = eigensolver_2.eigenvalues();
//	KLT_value_3 = eigensolver_3.eigenvalues();
//
//	MatrixXd KLT_1(Sn[0][0].rows(), cov_mat[0].rows());
//	MatrixXd KLT_2(Sn[0][0].rows(), cov_mat[0].rows());
//	MatrixXd KLT_3(Sn[0][0].rows(), cov_mat[0].rows());
//	KLT_1 = get_KLT(eigensolver_1.eigenvectors(), Sn[0][0].rows());
//	KLT_2 = get_KLT(eigensolver_2.eigenvectors(), Sn[0][0].rows());
//	KLT_3 = get_KLT(eigensolver_3.eigenvectors(), Sn[0][0].rows());
//
//
//	for (int idx = 0; idx < Sn[0].size(); idx++)
//	{
//		pp_R = KLT_1 * Sn[0][idx]; 
//		pp_G = KLT_2 * Sn[1][idx]; 
//		pp_B = KLT_3 * Sn[2][idx]; 
//
//		ppc_R.push_back(pp_R);
//		ppc_G.push_back(pp_G);
//		ppc_B.push_back(pp_B);
//	}
//
//	ppc_.push_back(ppc_R);
//	ppc_.push_back(ppc_G);
//	ppc_.push_back(ppc_B);
//
//
//	cout << " ============= End decode_KLT ============= " << endl;
//	return ppc_;
//}
//
//
//
//vector<vector< VectorXd>> decode_PPC_prime(vector<vector< VectorXd>> PPC_KLT, vector<vector<bool>> PPC_flag)
//{
//	vector<vector< VectorXd>> PPC_prime;
//	vector<VectorXd> PPC_prime_R;
//	vector<VectorXd> PPC_prime_G;
//	vector<VectorXd> PPC_prime_B;
//	VectorXd temp_R(total_num_cameras);
//	VectorXd temp_G(total_num_cameras);
//	VectorXd temp_B(total_num_cameras);
//
//
//
//	int cnt = 0;
//
//	for (int idx = 0; idx < PPC_flag.size(); idx++)
//	{
//		cnt = 0;
//		for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//		{
//			if (PPC_flag[idx][cam_idx] == true)
//			{
//				temp_R(cam_idx) = PPC_KLT[0][idx][cnt];
//				temp_G(cam_idx) = PPC_KLT[1][idx][cnt];
//				temp_B(cam_idx) = PPC_KLT[2][idx][cnt];
//				cnt++;
//			}
//			else
//			{
//				temp_R(cam_idx) = SHRT_MAX;
//				temp_G(cam_idx) = SHRT_MAX;
//				temp_B(cam_idx) = SHRT_MAX;
//			}
//		}
//		PPC_prime_R.push_back(temp_R);
//		PPC_prime_G.push_back(temp_G);
//		PPC_prime_B.push_back(temp_B);
//	}
//	PPC_prime.push_back(PPC_prime_R);
//	PPC_prime.push_back(PPC_prime_G);
//	PPC_prime.push_back(PPC_prime_B);
//
//	return PPC_prime;
//}
//
//
//
//vector<PPC> PPC_double2uchar(vector<vector<vector< VectorXd>>> ppc_double, vector<vector<PPC>> class_PPC, int start_idx)
//{
//	vector<PPC> out_PPC;
//	PPC temp_pp;
//	cout << 1 << endl;
//	for (int num_idx = 0; num_idx < ppc_double.size(); num_idx++)//plen pt num
//	{
//		cout << 3 << endl;
//
//		for (int idx = 0; idx < ppc_double[num_idx][0].size(); idx++)//num of pt
//		{
//			for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)//8 color
//			{
//				temp_pp.color[cam_idx][0] = ppc_double[num_idx][0][idx][cam_idx];
//				temp_pp.color[cam_idx][1] = ppc_double[num_idx][1][idx][cam_idx];
//				temp_pp.color[cam_idx][2] = ppc_double[num_idx][2][idx][cam_idx];
//
//				temp_pp.geometry[0] = class_PPC[num_idx + (start_idx - 1)][idx].geometry[0];
//				temp_pp.geometry[1] = class_PPC[num_idx + (start_idx - 1)][idx].geometry[1];
//				temp_pp.geometry[2] = class_PPC[num_idx + (start_idx - 1)][idx].geometry[2];
//			}
//			out_PPC.push_back(temp_pp);
//			cout << 4 << endl;
//
//		}
//
//	}
//	cout << 2 << endl;
//
//	return out_PPC;
//}
//
//vector<PPC> PPC_double2uchar(vector<vector<vector< VectorXd>>> ppc_double, vector<vector<PPC>> class_PPC, vector<int>& En_idx_vec)
//{
//	vector<PPC> out_PPC;
//	PPC temp_pp;
//	for (int num_idx = 0; num_idx < ppc_double.size(); num_idx++)//plen pt num
//	{
//		for (int idx = 0; idx < ppc_double[num_idx][0].size(); idx++)//num of pt
//		{
//			for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)//8 color
//			{
//				temp_pp.color[cam_idx][0] = ppc_double[num_idx][0][idx][cam_idx];
//				temp_pp.color[cam_idx][1] = ppc_double[num_idx][1][idx][cam_idx];
//				temp_pp.color[cam_idx][2] = ppc_double[num_idx][2][idx][cam_idx];
//
//				temp_pp.geometry[0] = class_PPC[En_idx_vec[num_idx]-1][idx].geometry[0];
//				temp_pp.geometry[1] = class_PPC[En_idx_vec[num_idx]-1][idx].geometry[1];
//				temp_pp.geometry[2] = class_PPC[En_idx_vec[num_idx]-1][idx].geometry[2];
//			}
//			out_PPC.push_back(temp_pp);
//		}
//	}
//
//	return out_PPC;
//}
//
//
//
//vector<double> get_err_norm1(vector<PPC> ppc_, vector<vector<VectorXd>> ppc_prime)
//{
//	cout << " ============= get_err_norm1 ============= " << endl;
//	vector<double> err_vec;
//	double err_R = 0;
//	double err_G = 0;
//	double err_B = 0;
//
//	vector<VectorXd> ppc_R;
//	vector<VectorXd> ppc_G;
//	vector<VectorXd> ppc_B;
//	VectorXd pp_R(total_num_cameras);
//	VectorXd pp_G(total_num_cameras);
//	VectorXd pp_B(total_num_cameras);
//
//	for (int idx = 0; idx < ppc_.size(); idx++)
//	{
//		for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//		{
//			pp_R(cam_idx) = ppc_[idx].color[cam_idx][0];
//			pp_G(cam_idx) = ppc_[idx].color[cam_idx][1];
//			pp_B(cam_idx) = ppc_[idx].color[cam_idx][2];
//		}
//		ppc_R.push_back(pp_R);
//		ppc_G.push_back(pp_G);
//		ppc_B.push_back(pp_B);
//	}
//
//
//	//norm 1
//	for (int idx = 0; idx < ppc_.size(); idx++)
//	{
//		for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//		{
//			err_R += abs(ppc_prime[0][idx](cam_idx) - double(ppc_R[idx](cam_idx)));
//			err_G += abs(ppc_prime[1][idx](cam_idx) - double(ppc_G[idx](cam_idx)));
//			err_B += abs(ppc_prime[2][idx](cam_idx) - double(ppc_B[idx](cam_idx)));
//		}
//		if (err_R > DBL_MAX || err_G > DBL_MAX || err_B > DBL_MAX)
//		{
//			cout << "(get_err_norm1)Max value err!" << endl;
//			break;
//		}
//	}
//
//	err_R /= ppc_.size();
//	err_G /= ppc_.size();
//	err_B /= ppc_.size();
//
//	err_vec.push_back(err_R);
//	err_vec.push_back(err_G);
//	err_vec.push_back(err_B);
//
//	return err_vec;
//}
//
//vector<double> get_err_norm2(vector<PPC> ppc_, vector<vector<VectorXd>> ppc_prime)
//{
//	cout << " ============= get_err_norm2 ============= " << endl;
//	vector<double> err_vec;
//	double err_R = 0;
//	double err_G = 0;
//	double err_B = 0;
//
//	vector<VectorXd> ppc_R;
//	vector<VectorXd> ppc_G;
//	vector<VectorXd> ppc_B;
//	VectorXd pp_R(total_num_cameras);
//	VectorXd pp_G(total_num_cameras);
//	VectorXd pp_B(total_num_cameras);
//
//	for (int idx = 0; idx < ppc_.size(); idx++)
//	{
//		for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//		{
//			pp_R(cam_idx) = ppc_[idx].color[cam_idx][0];
//			pp_G(cam_idx) = ppc_[idx].color[cam_idx][1];
//			pp_B(cam_idx) = ppc_[idx].color[cam_idx][2];
//		}
//		ppc_R.push_back(pp_R);
//		ppc_G.push_back(pp_G);
//		ppc_B.push_back(pp_B);
//	}
//
//
//	//norm 2
//	for (int idx = 0; idx < ppc_.size(); idx++)
//	{
//		for (int cam_idx = 0; cam_idx < total_num_cameras; cam_idx++)
//		{
//			err_R += (ppc_prime[0][idx](cam_idx) - double(ppc_R[idx](cam_idx))) * (ppc_prime[0][idx](cam_idx) - double(ppc_R[idx](cam_idx)));
//			err_G += (ppc_prime[1][idx](cam_idx) - double(ppc_G[idx](cam_idx))) * (ppc_prime[1][idx](cam_idx) - double(ppc_G[idx](cam_idx)));
//			err_B += (ppc_prime[2][idx](cam_idx) - double(ppc_B[idx](cam_idx))) * (ppc_prime[2][idx](cam_idx) - double(ppc_B[idx](cam_idx)));
//		}
//		if (err_R > DBL_MAX || err_G > DBL_MAX || err_B > DBL_MAX)
//		{
//			cout << "(get_err_norm2)Max value err!" << endl;
//			break;
//		}
//	}
//
//	err_R = sqrt(err_R / ppc_.size());
//	err_G = sqrt(err_G / ppc_.size());
//	err_B = sqrt(err_B / ppc_.size());
//
//	err_vec.push_back(err_R);
//	err_vec.push_back(err_G);
//	err_vec.push_back(err_B);
//
//	return err_vec;
//}
//
//
/////Encode
//vector<vector<vector< VectorXd>>> Encoder_(vector<PPC> input_PPC, vector<vector<MatrixXd>>& cov_mat, vector<vector<vector<bool>>>& ppc_flag_vec, vector<vector<PPC>>& class_PPC, int start_idx, int end_idx)
//{
//	cout << "===start Encode===" << endl;
//
//	///origin PPC classed
//	vector<uint> cnt_vec = get_cnt_vec(input_PPC);
//	class_PPC = classify_PPC(input_PPC, cnt_vec);
//	cout << "class_PPC.size():: " << class_PPC.size() << endl;
//
//	
//	///origin PPC classed and merged
//	vector<vector<PPC>> class_PPC_merged;
//	vector<vector<vector<bool>>> class_PPC_merged_flag;
//	for (int class_idx = 0; class_idx < total_num_cameras; class_idx++)
//	{
//		vector<PPC> merged_PPC;
//		vector<vector<bool>> class_PPC_flag;
//		merged_PPC = get_merged(class_PPC[class_idx], class_PPC_flag);
//
//		class_PPC_merged.push_back(merged_PPC);
//		ppc_flag_vec.push_back(class_PPC_flag);///ouput
//	}
//	
//
//	// 평균
//
//
//
//	///Encode
//	vector<vector<vector< VectorXd>>> PPC_prime;
//
//	vector<MatrixXd> cov_mat_temp;//3(chan)* cnt_idx*cnt_idx
//	vector<vector< VectorXd>> PPC_prime_temp;//3(chan) * PPC(size*8)
//
//	int klt_axis = 0;
//	for (int class_idx = start_idx; class_idx <= end_idx; class_idx++)
//	{
//
//				klt_axis = class_idx - 2;
//				if (start_idx < klt_axis || klt_axis < 1)
//					cerr << "!!!out of axis!!!" << endl;
//
//				PPC_prime_temp = Encode(class_PPC_merged[class_idx - 1], class_idx, klt_axis, cov_mat_temp);
//				//PPC_prime.push_back(PPC_prime_temp);
//				//cov_mat_temp 계산
//				cov_mat_temp;
//				cov_mat.push_back(cov_mat_temp);
//				cov_mat_temp.clear();
//				PPC_prime_temp.clear();
//
//	}
//
//	////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//	//for (int class_idx = med_idx; class_idx <= end_idx; class_idx++)
//	//{
//	//	klt_axis = 6;
//	//	if (start_idx < klt_axis || klt_axis < 1)
//	//		cerr << "!!!out of axis!!!" << endl;
//
//	//	PPC_prime_temp = Encode(class_PPC_merged[class_idx - 1], class_idx, klt_axis, cov_mat_temp);
//	//	PPC_prime.push_back(PPC_prime_temp);///output
//	//	cov_mat.push_back(cov_mat_temp);///output
//	//	cov_mat_temp.clear();
//	//	PPC_prime_temp.clear();
//	//}
//	cout << "===End Encode===" << endl;
//
//	return PPC_prime;
//}
//
/////Encode
//vector<vector<vector< VectorXd>>> Encoder_v2(vector<PPC> input_PPC, vector<vector<MatrixXd>>& cov_mat, vector<vector<vector<bool>>>& ppc_flag_vec, vector<vector<PPC>>& class_PPC, vector<int>& En_idx_vec, int reductoin_dim)
//{
//	cout << "===start Encode===" << endl;
//
//	///origin PPC classed
//	vector<uint> cnt_vec = get_cnt_vec(input_PPC);
//	class_PPC = classify_PPC(input_PPC, cnt_vec);
//
//	cout << "class_PPC.size():: " << class_PPC.size() << endl;
//	for (int i = 0; i < total_num_cameras; i++)
//	{
//		cout << "class_PPC size " << i+1 << ": " << class_PPC[i].size() << endl;
//	}
//
//	///origin PPC classed and merged
//	vector<vector<PPC>> class_PPC_merged;
//	vector<vector<vector<bool>>> class_PPC_merged_flag;
//	for (int class_idx = 0; class_idx < total_num_cameras; class_idx++)
//	{
//		vector<PPC> merged_PPC;
//		vector<vector<bool>> class_PPC_flag;
//		merged_PPC = get_merged(class_PPC[class_idx], class_PPC_flag);
//
//		class_PPC_merged.push_back(merged_PPC);
//		ppc_flag_vec.push_back(class_PPC_flag);///ouput
//	}
//
//
//	vector<int> exception_vec = En_idx_vec;
//	vector<int>::iterator En_it = En_idx_vec.begin();
//	int ii = 0;
//	for (int i = 0; i < total_num_cameras; i++)
//	{
//		if (i + 1 < En_idx_vec[0])
//			continue;
//		else
//		{
//			if (class_PPC[i].size() < total_num_cameras + 1)
//			{
//				exception_vec.erase(exception_vec.begin() + ii);
//			}
//			else
//				ii++;
//		}
//
//	}
//	En_idx_vec = exception_vec;
//
//	///Encode
//	vector<vector<vector< VectorXd>>> PPC_prime;
//
//	vector<MatrixXd> cov_mat_temp;//3(chan)* cnt_idx*cnt_idx
//	vector<vector< VectorXd>> PPC_prime_temp;//3(chan) * PPC(size*8)
//
//	int klt_axis = 0;
//
//	int class_idx = 0;
//	vector<int>::iterator it = En_idx_vec.begin();
//	for (it = En_idx_vec.begin(); it != En_idx_vec.end(); ++it)
//	{
//		class_idx = *it;
//		klt_axis = class_idx - reductoin_dim;
//		if (*En_idx_vec.begin() < klt_axis || klt_axis < 1)
//			cerr << "!!!out of axis!!!" << endl;
//
//		PPC_prime_temp = Encode(class_PPC_merged[class_idx - 1], class_idx, klt_axis, cov_mat_temp);
//		PPC_prime.push_back(PPC_prime_temp);
//		cov_mat.push_back(cov_mat_temp);
//		cov_mat_temp.clear();
//		PPC_prime_temp.clear();
//
//	}
//
//	//for (int class_idx = med_idx; class_idx <= end_idx; class_idx++)
//	//{
//	//	klt_axis = 6;
//	//	if (start_idx < klt_axis || klt_axis < 1)
//	//		cerr << "!!!out of axis!!!" << endl;
//
//	//	PPC_prime_temp = Encode(class_PPC_merged[class_idx - 1], class_idx, klt_axis, cov_mat_temp);
//	//	PPC_prime.push_back(PPC_prime_temp);///output
//	//	cov_mat.push_back(cov_mat_temp);///output
//	//	cov_mat_temp.clear();
//	//	PPC_prime_temp.clear();
//	//}
//	cout << "===End Encode===" << endl;
//
//	return PPC_prime;
//}
//
//
//vector<vector<VectorXd>> Encode(vector<PPC> class_PPC_merged, int class_idx, int klt_axis, vector<MatrixXd>& cov_mat)
//{
//	vector<vector<VectorXd>> Encoded_PPC;//3(chan) * PPC(size*8)
//
//
//	Encoded_PPC = KLT_trans(class_PPC_merged, class_idx, cov_mat, klt_axis);//(3 * size() * vec(klt_dim*1))		
//
//	for (int color_idx = 0; color_idx < 3; color_idx++)
//	{
//		cout << "cov_mat" << color_idx << " :: " << cov_mat[color_idx].rows() << " * " << cov_mat[color_idx].cols() << " = " << cov_mat[color_idx].size() << endl;
//		cout << cov_mat[color_idx] << endl << endl;
//
//	}
//
//
//
//	return Encoded_PPC;
//}
//
//
//vector<PPC> Decoder_(vector<vector<vector<VectorXd>>> ppc_prime_vec, vector<vector<MatrixXd>> cov_mat_vec, vector<vector<vector<bool>>> ppc_flag_vec, vector<vector<PPC>> class_ppc, vector<vector<vector< VectorXd>>>& _PPC_double, int start_idx)
//{
//	vector<PPC> _PPC;
//	vector<PPC> _PPC_uchar;
//	vector<vector<VectorXd>> _PPC_temp;
//
//
//
//	for (int class_idx = 0; class_idx < cov_mat_vec.size(); class_idx++)
//	{
//		_PPC_temp = Decode(ppc_prime_vec[class_idx], cov_mat_vec[class_idx], ppc_flag_vec[class_idx + (start_idx - 1)]);
//		_PPC_double.push_back(_PPC_temp);///output
//		_PPC_temp.clear();
//
//	}
//
//	_PPC = PPC_double2uchar(_PPC_double, class_ppc, start_idx);
//
//	for (int num_idx = 0; num_idx < start_idx - 1; num_idx++)
//	{
//		for (int idx = 0; idx < class_ppc[num_idx].size(); idx++)
//		{
//			_PPC.push_back(class_ppc[num_idx][idx]);
//		}
//	}
//
//	return _PPC;
//}
//
//vector<PPC> Decoder_v2(vector<vector<vector<VectorXd>>> ppc_prime_vec, vector<vector<MatrixXd>> cov_mat_vec, vector<vector<vector<bool>>> ppc_flag_vec, vector<vector<PPC>> class_ppc, vector<vector<vector< VectorXd>>>& _PPC_double, vector<int>& En_idx_vec)
//{
//	vector<PPC> _PPC;
//	vector<PPC> _PPC_uchar;
//	vector<vector<VectorXd>> _PPC_temp;
//
//
//	for (int class_idx = 0; class_idx < cov_mat_vec.size(); class_idx++)
//	{
//		_PPC_temp = Decode(ppc_prime_vec[class_idx], cov_mat_vec[class_idx], ppc_flag_vec[En_idx_vec[class_idx]-1]);
//		_PPC_double.push_back(_PPC_temp);///output
//		_PPC_temp.clear();
//
//	}
//
//
//	_PPC = PPC_double2uchar(_PPC_double, class_ppc, En_idx_vec);
//
//
//	for (int num_idx = 0; num_idx < total_num_cameras; num_idx++)
//	{
//		vector<int>::iterator num_it = find(En_idx_vec.begin(), En_idx_vec.end(), num_idx);
//
//		if (num_it != En_idx_vec.cend())
//			continue;
//		else
//		{ 
//			for (int idx = 0; idx < class_ppc[num_idx].size(); idx++)
//			{
//				_PPC.push_back(class_ppc[num_idx][idx]);
//			}
//		}
//
//	}
//
//	return _PPC;
//}
//
//
//vector < vector<VectorXd>> Decode(vector < vector<VectorXd>> ppc_prime, vector<MatrixXd> cov_mat, vector<vector<bool>> ppc_flag)
//{
//	vector<vector<VectorXd>> _PPC;
//	vector<vector< VectorXd>> _PPC_merged_temp;
//
//	///Decode
//	_PPC_merged_temp = decode_KLT(ppc_prime, cov_mat);
//
//	///recover Decod dim
//	_PPC = decode_PPC_prime(_PPC_merged_temp, ppc_flag);
//
//
//	return _PPC;
//}
//
//
//
//
//void print_class(vector<Vec3f> vec_, uint num_cnt)
//{
//	cout << "===========start print std_dev============" << endl;
//	vector<vector<uint>> class_(3, vector<uint>(6));
//
//	for (int idx = 0; idx < vec_.size(); idx++)
//	{
//		for (int chan_idx = 0; chan_idx < 3; chan_idx++)
//		{
//
//			if (vec_[idx][chan_idx] == 0)
//				class_[chan_idx][0]++;
//			else if (vec_[idx][chan_idx] > 0 && vec_[idx][chan_idx] < 5)
//				class_[chan_idx][1]++;
//			else if (vec_[idx][chan_idx] >= 5 && vec_[idx][chan_idx] < 10)
//				class_[chan_idx][2]++;
//			else if (vec_[idx][chan_idx] >= 10 && vec_[idx][chan_idx] < 15)
//				class_[chan_idx][3]++;
//			else if (vec_[idx][chan_idx] >= 15 && vec_[idx][chan_idx] < 20)
//				class_[chan_idx][4]++;
//			else if (vec_[idx][chan_idx] >= 20)
//				class_[chan_idx][5]++;
//		}
//	}
//
//	for (int chan_idx = 0; chan_idx < 3; chan_idx++)
//	{
//		switch (chan_idx)
//		{
//		case 0:
//			cout << "R" << endl;
//			break;
//		case 1:
//			cout << "G" << endl;
//			break;
//		case 2:
//			cout << "B" << endl;
//			break;
//		}
//		cout << "0(dummy): " << num_cnt << "개 ," << (double)num_cnt / vec_.size() * 100 << "% " << endl;
//		cout << "0: " << class_[chan_idx][0] - num_cnt << "개 ," << ((double)class_[chan_idx][0] - num_cnt) / vec_.size() * 100 << "% \n" << endl;
//		cout << "0-5: " << class_[chan_idx][1] << "개 ," << (double)class_[chan_idx][1] / vec_.size() * 100 << "% " << endl;
//		cout << "5-10: " << class_[chan_idx][2] << "개 ," << (double)class_[chan_idx][2] / vec_.size() * 100 << "% " << endl;
//		cout << "10-15: " << class_[chan_idx][3] << "개 ," << (double)class_[chan_idx][3] / vec_.size() * 100 << "% " << endl;
//		cout << "15-20: " << class_[chan_idx][4] << "개 ," << (double)class_[chan_idx][4] / vec_.size() * 100 << "% " << endl;
//		cout << "20~: " << class_[chan_idx][5] << "개 ," << (double)class_[chan_idx][5] / vec_.size() * 100 << "% \n" << endl;
//	}
//
//
//	cout << "===========End print std_dev============" << endl;
//
//}