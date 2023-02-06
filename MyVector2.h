#pragma once
#include"Matrix2.h"
//#include"InertialNavigation.h";
class Vector :public Matrix
{
public:
	Vector(int n,double value=0.0);
	Vector& operator=(const Vector&);
	Vector& operator=(const Matrix&);
	Vector& operator=(const double Array[]);
	Vector& GetFormArray(int pos,const double Array[]);//从数组中的任意位置取值.pos为数组下标。取长度为矢量长度

	double Norm()const;//向量取模
	Vector operator->*(const Vector&)const;//向量叉乘
	void ToUnit();//向量单位化
	friend Matrix AntiSymMatrix(Vector& v);//求取向量的反对称矩阵

};