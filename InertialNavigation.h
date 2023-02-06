#pragma once
#include"Coordinate.h"
#include"Matrix2.h"
#include"MyVector2.h"
#include<iostream>
struct ANGLE
{
	double yaw;
	double pitch;
	double roll;

	ANGLE()
	{
		yaw = 0.0;
		pitch = 0.0;
		roll = 0.0;
	}
};

class Body
{
public:
	Body(BLH B, Vector V, ANGLE A);
	~Body();

	void Updating(Vector AngInc_k_1, Vector AngInc_k, Vector VelInc_k_1, Vector VelInc_k, double delta_t);//����k-1ʱ�̽�����������kʱ�̽������������Ͳ������������̬���и���
	void Show();
	void output(FILE*fp);

	BLH getBlh();
	Vector getVel();
	ANGLE getAngle();

private:
	BLH Blh;//λ��(�Ƕ�)
	Vector Vel;//�����ٶȣ������ٶȣ��������ٶ�
	ANGLE Angle;//��̬��(�Ƕ�)
	Matrix Cn_b;//����̬�Ƕ�Ӧ�ķ������Ҿ�����b��N��

	//����ʱ��Ҫ�õ���ǰһʱ�̵�λ�ú��ٶ�
	BLH Blh_k_2;
	Vector Vel_k_2;
	
};
