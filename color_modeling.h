#pragma once
#include "common.h"
#include "global.h"

// color polynomial modeling
typedef struct {
	float geometry[3];
	unsigned short dummy;
	float a0[3];
	float a1[3];
	float a2[3];
} PPC2;

// color polynomial modeling
typedef struct {
	float geometry[3];
	unsigned short dummy;
	float a0[3];
	float a1[3];
} PPC3;

// color modeling : diffuse (linear model), specular (2nd degree polynomial model)
typedef struct {
	float geometry[3];
	unsigned short dummy;
	float diffuse[3][2];
	float specular[3][3];
} PPC4;

// color polynomial modeling
typedef struct {
	float geometry[3];
	unsigned short dummy;
	float a0[3];
	float a1;
	float a2;
} PPC5;

// 2nd order polynomial model
vector<PPC2> encodeColorModelingPPC(vector<PPC>& Plen_PC);
vector<PPC> decodeColorModelingPPC(vector<PPC2>& color_modeling_Plen_PC);
// Linear model
vector<PPC3> encodeColorModelingPPC2(vector<PPC>& Plen_PC);
vector<PPC> decodeColorModelingPPC2(vector<PPC3>& color_modeling_Plen_PC);
// Diffuse(linear model) + specular(2nd order polynomial model)
vector<PPC4> encodeColorModelingPPC3(vector<PPC>& Plen_PC);
vector<PPC> decodeColorModelingPPC3(vector<PPC4>& color_modeling_Plen_PC);

vector<PPC5> encodeColorModelingPPC4(vector<PPC>& Plen_PC);
vector<PPC> decodeColorModelingPPC4(vector<PPC5>& color_modeling_Plen_PC);