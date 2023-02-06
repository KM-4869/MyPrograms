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

	void Updating(Vector AngInc_k_1, Vector AngInc_k, Vector VelInc_k_1, Vector VelInc_k, double delta_t);//输入k-1时刻角增量向量，k时刻角增量向量，和采样间隔，对姿态进行更新
	void Show();
	void output(FILE*fp);

	BLH getBlh();
	Vector getVel();
	ANGLE getAngle();

private:
	BLH Blh;//位置(角度)
	Vector Vel;//北向速度，东向速度，垂向下速度
	ANGLE Angle;//姿态角(角度)
	Matrix Cn_b;//与姿态角对应的方向余弦矩阵（由b到N）

	//更新时需要用到的前一时刻的位置和速度
	BLH Blh_k_2;
	Vector Vel_k_2;
	
};
