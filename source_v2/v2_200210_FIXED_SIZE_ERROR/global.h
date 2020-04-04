#pragma once

#define BOOST_TYPEOF_EMULATION

#define MSR3DVideo_Ballet 0
#define Poznan_Fencing 1
#define Intel_Kermit 2
#define Technicolor_Painter 3
#define hotelroom_r2_front_sample 4

#include "stdio.h"
#include "stdlib.h"
#include <windows.h>
#include <iostream>
#include "opencv.hpp"
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/eigen.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <math.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <list>
#include <pcl/registration/icp.h>
#include <ctime>
#include <pcl/features/normal_3d.h>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <bitset>

using namespace pcl;
using namespace std;
using namespace io;
using namespace cv;
using namespace Eigen;
using namespace visualization;

typedef struct {
	Matrix3d m_K;
	Matrix3d m_RotMatrix;
	Matrix3Xd m_Trans;
	Matrix4d m_ProjMatrix;
} CalibStruct;

extern int mode, _width, _height, total_num_cameras, total_num_frames, color_bits, depth_bits;
extern double MinZ, MaxZ;
extern vector<Vector2d> tech_minmaxZ;
extern string path;
extern vector<CalibStruct> m_CalibParams;
extern vector<float> geo_min, geo_max;
extern int degree;

class PPC {
private:
	float* geometry;

public:

	//v2, v2_2
	virtual bool CheckOcclusion(int idx) { return false; }
	virtual Vec3b GetColor(int idx) { return { 0, 0, 0 }; }
	virtual vector<uchar> GetVU() { return { 0 }; }
	virtual vector<uchar> GetY() { return { 0 }; }
	virtual uchar GetrefV() { return 0; }
	virtual uchar GetrefU() { return 0; }
	virtual float* GetGeometry() { return NULL; }
	virtual void SetGeometry(PointXYZRGB p) {}
	virtual void SetGeometry(float* geo) {}
	virtual void SetOcclusion(int idx) {}
	virtual void SetRefColor(Vec3b color, int idx) {}
	virtual void SetRefColor(PointXYZRGB point, int idx) {}
	virtual void SetColor(Vec3b color, int idx) {}
	virtual void SetColor(uchar v_, uchar u_, uchar y_, int idx) {}
	virtual void SetColor(uchar refv, uchar refu, vector<uchar> vu, vector<uchar> y) {}

	//v2.2
	virtual int GetColorNum() { return 0; }
	virtual uchar GetV() { return 0; }
	virtual uchar GetU() { return 0; }
	virtual vector<bool> GetOcclusion() { return { false }; }
	virtual void SetColor(PointXYZRGB point, int idx) {}
	virtual void SetColor(uchar avrv, uchar avru, vector<uchar> y, vector<bool> occ) {}

	//v3.1
	virtual void SetColor(uchar refv, uchar refu, vector<uchar> vu, MatrixXf coeff) {}

	//v3.2
	virtual void SetCoef(PPC p) {}
	virtual uchar GetY(int idx) { return 0; }
	virtual MatrixXf GetCoef() { return  ArrayXf::Zero(); }
	virtual void SetColor(uchar avrv, uchar avru, MatrixXf coeff, vector<bool> occ) {}

};

class PPC_v2_1 : public PPC {
private:
	float* geometry;
	uchar refV;
	uchar refU;
	vector<uchar> VU;
	vector<uchar> Y;

public:

	PPC_v2_1()
	{
		// occlusion���� �ʱ�ȭ.
		geometry = (float*)malloc(3 * sizeof(float));
		VU = vector<uchar>(total_num_cameras, uchar(255));
		Y = vector<uchar>(total_num_cameras);
	}

	bool CheckOcclusion(int idx)
	{
		return int(VU[idx]) == 255;
	}

	virtual Vec3b GetColor(int idx)
	{
		Vec3b vuy;
		int firstCamNum = 0;
		for (int i = 0; i < total_num_cameras; i++) {
			if (!CheckOcclusion(i)) {
				firstCamNum = i;
				break;
			}
		}

		uchar v = refV;
		uchar u = refU;
		for (int i = firstCamNum + 1; i <= idx; i++) {
			if (!CheckOcclusion(i)) {
				v += (VU[i] >> 4) - 7;
				u += (uchar(VU[i] << 4) >> 4) - 7;
			}
		}
		vuy[0] = v;
		vuy[1] = u;

		vuy[2] = Y[idx];
		return vuy;
	}

	int GetColorNum() {
		int num = 0;
		for (int i = 0; i < total_num_cameras; i++)
			if (!CheckOcclusion(i)) num++;
		return num;
	}

	///save, load 함수에 쓰임
	vector<uchar> GetVU() {
		return VU;
	}

	vector<uchar> GetY() {
		return Y;
	}

	uchar GetrefV() {
		return refV;
	}

	uchar GetrefU() {
		return refU;
	}

	float* GetGeometry() {
		return geometry;
	}

	void SetGeometry(PointXYZRGB p) {

		geometry[0] = p.x;
		geometry[1] = p.y;
		geometry[2] = p.z;

	}

	void SetGeometry(float* geo) {
		geometry[0] = geo[0];
		geometry[1] = geo[1];
		geometry[2] = geo[2];
	}

	void SetOcclusion(int idx) {
		VU[idx] = uchar(255);
	}

	// color��  YUV
	void SetRefColor(Vec3b color, int idx)
	{
		refV = color[2];
		refU = color[1];
		Y[idx] = color[0];
		VU[idx] = 0;
	}

	// color�� VUY
	void SetRefColor(PointXYZRGB point, int idx)
	{

		refV = point.r;
		refU = point.g;
		Y[idx] = point.b;
		VU[idx] = 0;
	}

	// color�� yuv
	void SetColor(Vec3b color, int idx)
	{
		int firstCamNum = 0;
		for (int i = 0; i < total_num_cameras; i++) {
			if (!CheckOcclusion(i)) {
				firstCamNum = i;
				break;
			}
		}

		uchar v = refV;
		uchar u = refU;
		for (int i = firstCamNum + 1; i < idx; i++) {
			if (!CheckOcclusion(i)) {
				v += (VU[i] >> 4) - 7;
				u += (uchar(VU[i] << 4) >> 4) - 7;
			}
		}

		VU[idx] = uchar((color[2] - v + 7) << 4) + uchar(color[1] - u + 7);
		Y[idx] = color[0];
	}

	void SetColor(uchar v_, uchar u_, uchar y_, int idx)
	{
		int firstCamNum = 0;
		for (int i = 0; i < total_num_cameras; i++) {
			if (!CheckOcclusion(i)) {
				firstCamNum = i;
				break;
			}
		}

		uchar v = refV;
		uchar u = refU;
		for (int i = firstCamNum + 1; i < idx; i++) {
			if (!CheckOcclusion(i)) {
				v += (VU[i] >> 4) - 7;
				u += (uchar(VU[i] << 4) >> 4) - 7;
			}
		}

		VU[idx] = uchar(uchar((v_ - v + 7) << 4) + (u_ - u + 7));
		Y[idx] = y_;
	}

	void SetColor(uchar refv, uchar refu, vector<uchar> vu, vector<uchar> y) {
		refV = refv;
		refU = refu;
		VU = vu;
		Y = y;
	}
};

class PPC_v2_2 : public PPC {
private:
	float* geometry;
	uchar avrV;
	uchar avrU;
	vector<uchar> Y;
	vector<bool> occlusion;
public:

	PPC_v2_2()
	{
		// occlusion���� �ʱ�ȭ.
		geometry = (float*)malloc(3 * sizeof(float));
		Y = vector<uchar>(total_num_cameras);
		occlusion = vector<bool>(total_num_cameras, false);
	}

	bool CheckOcclusion(int idx)
	{
		return !occlusion[idx];
	}

	uchar GetV() {
		//uchar avrV = new uchar[1];
		return avrV;
	}

	uchar GetU() {
		//uchar* avrU = new uchar[1];
		return avrU;
	}

	vector<uchar> GetY() {
		return Y;
	}

	Vec3b GetColor(int idx)
	{
		Vec3b vuy;
		vuy[0] = avrV;
		vuy[1] = avrU;
		vuy[2] = Y[idx];

		return vuy;
	}

	float* GetGeometry() {
		return geometry;
	}

	vector<bool> GetOcclusion() {
		return occlusion;
	}

	void SetGeometry(PointXYZRGB p) {
		geometry[0] = p.x;
		geometry[1] = p.y;
		geometry[2] = p.z;

	}
	void SetGeometry(float* geo) {
		geometry[0] = geo[0];
		geometry[1] = geo[1];
		geometry[2] = geo[2];
	}

	void SetOcclusion(int idx) {
		occlusion[idx] = true;
	}

	int GetColorNum() {
		int num = 0;
		for (int i = 0; i < total_num_cameras; i++)
			if (occlusion[i]) num++;

		return num;
	}

	// color��  YUV
	void SetColor(Vec3b color, int idx)
	{
		int colorNum = GetColorNum();
		if (colorNum == 0) {
			avrV = color[2];
			avrU = color[1];
		}
		else {
			avrV = uchar(avrV * (colorNum / float(colorNum + 1)) + (color[2] / float(colorNum + 1)));
			avrU = uchar(avrU * (colorNum / float(colorNum + 1)) + (color[1] / float(colorNum + 1)));
		}
		Y[idx] = color[0];
		occlusion[idx] = true;
	}

	//VUY
	void SetColor(PointXYZRGB point, int idx) {
		int colorNum = GetColorNum();
		if (colorNum == 0) {
			avrV = point.r;
			avrU = point.g;
		}
		else {
			avrV = uchar(avrV * (colorNum / float(colorNum + 1)) + (point.r / float(colorNum + 1)));
			avrU = uchar(avrU * (colorNum / float(colorNum + 1)) + (point.g / float(colorNum + 1)));
		}
		Y[idx] = point.b;
		occlusion[idx] = true;
	}

	void SetColor(uchar avrv, uchar avru, vector<uchar> y, vector<bool> occ) {
		avrV = avrv;
		avrU = avru;
		Y = y;
		occlusion = occ;
	}
};

class PPC_v3_1 : public PPC {
private:
	float* geometry;
	uchar refV;
	uchar refU;
	vector<uchar> VU;
	MatrixXf coef;

public:

	PPC_v3_1(PPC* p, int degree)
	{
		geometry = (float*)malloc(3 * sizeof(float));
		geometry = ((PPC_v2_1*)p)->GetGeometry();
		refV = ((PPC_v2_1*)p)->GetrefV();
		refU = ((PPC_v2_1*)p)->GetrefU();
		VU = ((PPC_v2_1*)p)->GetVU();
		SetCoef(p, degree);
		//cout << "PPC_v2_1" << endl;
	}

	bool CheckOcclusion(int idx) {
		return int(VU[idx]) == 255;
	}

	float* GetGeometry() {
		return geometry;
	}

	uchar GetrefV() {
		return refV;
	}

	uchar GetrefU() {
		return refU;
	}

	vector<uchar> GetVU() {
		return VU;
	}

	MatrixXf Getcoef() {
		return coef;
	}

	void SetGeometry(float* geo) {
		geometry[0] = geo[0];
		geometry[1] = geo[1];
		geometry[2] = geo[2];
	}

	void SetCoef(PPC* p, int degree) {
		//Ax = b
		vector<uchar> Y = ((PPC_v2_1*)p)->GetY();
		int colornum = ((PPC_v2_1*)p)->GetColorNum();
		if (degree == 2) {
			coef.resize(3, 1);
			if (colornum == 1) {
				for (int i = 0; i < total_num_cameras; i++) {
					if (!((PPC_v2_1*)p)->CheckOcclusion(i)) {// occ -> true
						coef(0, 0) = Y[i];
					}
				}
				coef(1, 0) = 0.;
				coef(2, 0) = 0.;
			}
			else if (colornum == 2) {
				bool is_second = false;
				for (int i = 0; i < total_num_cameras; i++) {
					if (!((PPC_v2_1*)p)->CheckOcclusion(i)) {// occ -> true
						if (is_second) {
							coef(1, 0) = Y[i];
						}
						else {
							coef(0, 0) = Y[i];
							is_second = true;
						}
					}
				}
				coef(2, 0) = 0.;
			}
			else {
				Matrix3f A;
				Matrix3Xf b(3, 1); // coeff(3, 1);
				vector<float> A_vec(4, 0), b_vec(3, 0);

				for (int i = 0; i < total_num_cameras; i++) {
					if (!((PPC_v2_1*)p)->CheckOcclusion(i)) {// occ -> true
						A_vec[0] += i;
						A_vec[1] += i * i;
						A_vec[2] += i * i * i;
						A_vec[3] += i * i * i * i;

						b_vec[0] += Y[i];
						b_vec[1] += i * Y[i];
						b_vec[2] += i * i * Y[i];
					}
				}

				A(0, 0) = ((PPC_v2_1*)p)->GetColorNum();
				A(0, 1) = A_vec[0];
				A(0, 2) = A_vec[1];
				A(1, 0) = A_vec[0];
				A(1, 1) = A_vec[1];
				A(1, 2) = A_vec[2];
				A(2, 0) = A_vec[1];
				A(2, 1) = A_vec[2];
				A(2, 2) = A_vec[3];

				b(0, 0) = b_vec[0];
				b(1, 0) = b_vec[1];
				b(2, 0) = b_vec[2];

				coef = A.inverse() * b;
			}
		}
		else if (degree == 3) {
			coef.resize(4, 1);
			if (colornum <= 16) {
				vector<uchar> v_u;
				for (int i = 0; i < total_num_cameras; i++) {
					if (!((PPC_v2_1*)p)->CheckOcclusion(i)) {
						v_u.push_back(Y[i]);
					}
				}

				int* temp_p = new int(0);

				for (int i = 0; i < v_u.size(); i++) {

					*temp_p |= v_u[i];

					if (i % 4 == 3) {
						coef(i / 4, 0) = *reinterpret_cast<float*>(temp_p);
						*temp_p = 0;
					}
					else if (v_u.size() - 1 == i) {
						if (i % 4 == 0) *temp_p <<= 8 * 3;
						else if (i % 4 == 1) *temp_p <<= 8 * 2;
						else if (i % 4 == 2) *temp_p <<= 8 * 1;
						coef(i / 4, 0) = *reinterpret_cast<float*>(temp_p);
						break;
					}
					*temp_p <<= 8;


				}
				delete temp_p;
				//cout << "========" << endl;

			}
			else {
				Matrix4f A;
				Matrix4Xf b(4, 1); // coeff(3, 1);
				vector<float> A_vec(6, 0), b_vec(4, 0);

				for (int i = 0; i < total_num_cameras; i++) {
					if (!((PPC_v2_1*)p)->CheckOcclusion(i)) {// occ -> true
						A_vec[0] += i;
						A_vec[1] += i * i;
						A_vec[2] += i * i * i;
						A_vec[3] += i * i * i * i;
						A_vec[4] += i * i * i * i * i;
						A_vec[5] += i * i * i * i * i * i;

						b_vec[0] += Y[i];
						b_vec[1] += i * Y[i];
						b_vec[2] += i * i * Y[i];
						b_vec[3] += i * i * i * Y[i];
					}
				}

				A(0, 0) = ((PPC_v2_1*)p)->GetColorNum();
				A(0, 1) = A_vec[0];
				A(0, 2) = A_vec[1];
				A(0, 3) = A_vec[2];
				A(1, 0) = A_vec[0];
				A(1, 1) = A_vec[1];
				A(1, 2) = A_vec[2];
				A(1, 3) = A_vec[3];
				A(2, 0) = A_vec[1];
				A(2, 1) = A_vec[2];
				A(2, 2) = A_vec[3];
				A(2, 3) = A_vec[4];
				A(3, 0) = A_vec[2];
				A(3, 1) = A_vec[3];
				A(3, 2) = A_vec[4];
				A(3, 3) = A_vec[5];

				b(0, 0) = b_vec[0];
				b(1, 0) = b_vec[1];
				b(2, 0) = b_vec[2];
				b(3, 0) = b_vec[3];

				coef = A.inverse() * b;
			}
		}
	}

	int GetColorNum() {
		int num = 0;
		for (int i = 0; i < total_num_cameras; i++)
			if (!CheckOcclusion(i)) num++;
		return num;
	}

	uchar GetY(int idx) {
		uchar y;
		if (degree == 2) y = uchar(coef(0, 0) + coef(1, 0) * idx + coef(2, 0) * idx * idx);
		else if (degree == 3) y = uchar(coef(0, 0) + coef(1, 0) * idx + coef(2, 0) * idx * idx + coef(3, 0) * idx * idx * idx);
		return y;
	}

	Vec3b GetColor(int idx) {
		Vec3b vuy;
		int firstCamNum = 0;
		for (int i = 0; i < total_num_cameras; i++) {
			if (!CheckOcclusion(i)) {
				firstCamNum = i;
				break;
			}
		}

		uchar v = refV;
		uchar u = refU;

		for (int i = firstCamNum + 1; i <= idx; i++) {
			if (!CheckOcclusion(i)) {
				v += (VU[i] >> 4) - 7;
				u += (uchar(VU[i] << 4) >> 4) - 7;
			}
		}

		vuy[0] = v;
		vuy[1] = u;
		if (degree == 2) {
			if (GetColorNum() == 1) {
				vuy[2] = (uchar)coef(0, 0);
			}
			else if (GetColorNum() == 2) {
				for (int i = 0; i < total_num_cameras; i++) {
					if (!CheckOcclusion(i)) {
						if (idx == i) {
							vuy[2] = (uchar)coef(0, 0);
						}
						else {
							vuy[2] = (uchar)coef(1, 0);
						}
						break;
					}
				}
			}
			else {
				vuy[2] = GetY(idx);
			}
		}
		else if (degree == 3) {
			if (GetColorNum() <= 16) {
				int cnt = 0;
				int temp_i = 0;
				for (int i = 0; i < total_num_cameras; i++) {
					if (!CheckOcclusion(i)) {
						cnt++;
						if (idx == i) {

							//if (coef((cnt-1) / 4, 0) == 0)
							//	cout << cnt-1 << "  " << coef((cnt-1) / 4, 0) << endl ;
							float f = coef((cnt - 1) / 4, 0);
							int* temp_p = (int*)&f;
							//if (cnt == 2) cout << bitset<32>(*temp_p )<< endl;
							*temp_p <<= ((cnt - 1) % 4) * 8;
							//if (cnt == 2) cout << bitset<32>(*temp_p) << endl;
							*temp_p >>= 8 * 3;
							//if (cnt == 2) cout << bitset<32>(*temp_p) << endl;
							vuy[2] = uchar(*temp_p);

							//if(bitset<32>(*temp_p) == 0)
							//	cout << bitset<32>(*temp_p) << endl;
							//cout << " ==============" << endl;



							break;
						}
					}
				}
				//cout << "====================" << endl;
			}
			else {
				vuy[2] = GetY(idx);

			}
		}
		//cout << "V : " << (int)vuy[0] << " U : " << (int)vuy[1] << " Y : " << (int)vuy[2] << endl;
		return vuy;
	}

	void SetColor(uchar refv, uchar refu, vector<uchar> vu, MatrixXf coeff) {
		refV = refv;
		refU = refu;
		VU = vu;
		coef = coeff;
	}
};

class PPC_v3_2 : public PPC {
private:
	float* geometry;
	uchar avrV;
	uchar avrU;
	MatrixXf coef;//
	vector<bool> occlusion;

public:
	PPC_v3_2() {
		geometry = (float*)malloc(3 * sizeof(float));
		occlusion = vector<bool>(total_num_cameras, false);
	}

	PPC_v3_2(PPC* p)
	{
		geometry = (float*)malloc(3 * sizeof(float));
		occlusion = vector<bool>(total_num_cameras, false);
		geometry = ((PPC_v2_2*)p)->GetGeometry();
		avrV = ((PPC_v2_2*)p)->GetV();
		avrU = ((PPC_v2_2*)p)->GetU();
		occlusion = ((PPC_v2_2*)p)->GetOcclusion();
		SetCoef(p);
	}

	bool CheckOcclusion(int idx)
	{
		return !occlusion[idx];
	}

	void SetCoef(PPC* p) {
		//Ax = b
		vector<uchar> Y = ((PPC_v2_2*)p)->GetY();
		coef.resize(3, 1);
		int colornum = ((PPC_v2_2*)p)->GetColorNum();

		if (colornum == 1) {
			for (int i = 0; i < total_num_cameras; i++) {
				if (!((PPC_v2_2*)p)->CheckOcclusion(i)) {// occ -> true
					//if ((int)Y[i] > 253) {
					//	Y[i] = 0;
					//	avrU = 0;
					//	avrV = 0;
					//	//cout << i << " " << " Y : " << (int)Y[i] << " " << (int)avrU << " " << (int)avrV << endl;
					//}
					coef(0, 0) = Y[i];
				}
			}
			coef(1, 0) = 0.;
			coef(2, 0) = 0.;
		}
		else if (colornum == 2) {
			bool is_second = false;
			for (int i = 0; i < total_num_cameras; i++) {
				if (!((PPC_v2_2*)p)->CheckOcclusion(i)) {// occ -> true
					//if ((int)Y[i] > 253) {
					//	Y[i] = 0;
					//	avrU = 0;
					//	avrV = 0;
					//	//cout << i << " " << " Y : " << (int)Y[i] << " " << (int)avrU << " " << (int)avrV << endl;
					//}
					if (is_second) {
						coef(1, 0) = Y[i];
					}
					else {
						coef(0, 0) = Y[i];
						is_second = true;
					}
				}
			}
			coef(2, 0) = 0.;
		}
		else {
			Matrix3f A;
			Matrix3Xf b(3, 1); // coeff(3, 1);
			vector<float> A_vec(4, 0), b_vec(3, 0);

			for (int i = 0; i < total_num_cameras; i++) {
				if (!((PPC_v2_2*)p)->CheckOcclusion(i)) { // occ -> true
					//if ((int)Y[i] > 240) {
					//	Y[i] = 0;
					//	avrU = 0;
					//	avrV = 0;
					//	//cout << i << " " << " Y : " << (int)Y[i] << " " << (int)avrU << " " << (int)avrV << endl;
					//}

					A_vec[0] += i;
					A_vec[1] += i * i;
					A_vec[2] += i * i * i;
					A_vec[3] += i * i * i * i;

					b_vec[0] += Y[i];
					b_vec[1] += i * Y[i];
					b_vec[2] += i * i * Y[i];
				}
			}

			A(0, 0) = ((PPC_v2_2*)p)->GetColorNum();
			A(0, 1) = A_vec[0];
			A(0, 2) = A_vec[1];
			A(1, 0) = A_vec[0];
			A(1, 1) = A_vec[1];
			A(1, 2) = A_vec[2];
			A(2, 0) = A_vec[1];
			A(2, 1) = A_vec[2];
			A(2, 2) = A_vec[3];

			b(0, 0) = b_vec[0];
			b(1, 0) = b_vec[1];
			b(2, 0) = b_vec[2];

			coef = A.inverse() * b;

			/*if (coef(0, 0) > 255) {
				cout << coef(0, 0) << " " << coef(1, 0) << " " << coef(2, 0) << endl;

				for (int i = 0; i < total_num_cameras; i++) {
					if (!((PPC_v2_2*)p)->CheckOcclusion(i)) {
						cout << i << " " << (int)Y[i] << " " << (int)avrU << " " << (int)avrV << endl;
					}
				}

			}*/


		}

	}

	float* GetGeometry() {
		return geometry;
	}

	MatrixXf Getcoef() {
		return coef;
	}

	uchar GetY(int idx) {
		uchar y;
		float y_f = coef(0, 0) + coef(1, 0) * idx + coef(2, 0) * idx * idx;
		if (y_f > 255) y = (uchar)255;
		else  y = uchar(y_f);

		//if (y > 250)
		//	cout << "avrV : " << (int)avrV << " avrU : " << (int)avrU << " y : " << (int)y << endl;
		return y;
	}

	int GetColorNum() {
		int num = 0;
		for (int i = 0; i < total_num_cameras; i++)
			if (occlusion[i]) num++;
		return num;
	}

	Vec3b GetColor(int idx) {
		Vec3b vuy;
		vuy[0] = avrV;
		vuy[1] = avrU;
		if (GetColorNum() == 1) {
			vuy[2] = (uchar)coef(0, 0);
		}
		else if (GetColorNum() == 2) {
			for (int i = 0; i < total_num_cameras; i++) {
				if (!CheckOcclusion(i)) {
					if (idx == i) {		// 첫번째 값이라면
						vuy[2] = (uchar)coef(0, 0);
					}
					else {
						vuy[2] = (uchar)coef(1, 0);
					}
					break;
				}
			}
		}
		else {
			vuy[2] = GetY(idx);
		}

		return vuy;
	}

	void SetGeometry(float* geo) {
		geometry[0] = geo[0];
		geometry[1] = geo[1];
		geometry[2] = geo[2];
	}

	void SetColor(uchar avrv, uchar avru, MatrixXf coeff, vector<bool> occ) {
		avrV = avrv;
		avrU = avru;
		coef = coeff;
		occlusion = occ;
	}
};
