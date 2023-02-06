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
	Vector& GetFormArray(int pos,const double Array[]);//�������е�����λ��ȡֵ.posΪ�����±ꡣȡ����Ϊʸ������

	double Norm()const;//����ȡģ
	Vector operator->*(const Vector&)const;//�������
	void ToUnit();//������λ��
	friend Matrix AntiSymMatrix(Vector& v);//��ȡ�����ķ��Գƾ���

};