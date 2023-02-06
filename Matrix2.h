#pragma once
#include<stdarg.h>
class Vector;
class Matrix
{
protected:
	int MatrixRows;//矩阵行数
	int MatrixCols;//矩阵列数
	double* M;//用一维数组表示矩阵
	void Initialize();//初始化矩阵。为矩阵分配空间

public:
	Matrix(const Matrix& m);//手动重写复制构造函数
	Matrix(int, int, double = 0.0);//默认矩阵所有元素赋值为0.！！注意：若所赋值为整数，需要变成在后面加“.0”的形式！！
	Matrix(int, int, char, ...);//对整个矩阵的每个元素赋初值。其中第三个参数为为区别重载函数所设，也方便区分矩阵的行列数和所要赋的值。推荐使用‘|’。！！注意：若所赋值为整数，需要变成在后面加“.0”的形式！！
	virtual ~Matrix();//析构函数

	Matrix& operator=(const Matrix&);//矩阵对矩阵赋值
	Matrix& operator=(double Array[]);//数组对矩阵赋值(要求事先声明过矩阵的大小)
	void assign(int i, int j, double value);//对第i行第j列的元素赋值
	Matrix operator+(const Matrix& m)const;//矩阵加法
	Matrix operator-(const Matrix& m)const;//矩阵减法
	Matrix operator*(const Matrix& m)const;//矩阵乘法
	friend Matrix operator*(const double C,const Matrix&m);//矩阵数乘
	friend Matrix operator*(const Matrix& m, const double C);//矩阵数乘
	Matrix T()const;//矩阵转置
	double det()const;//求行列式
	Matrix A()const;//伴随矩阵
	Matrix inv()const;//矩阵求逆
	void ToE(int n);//将矩阵变为n维单位阵
	void Show()const;

	int getrow()const;
	int getcol()const;
	int getsize()const;
	double getelement(int row,int col)const;

	friend Matrix AntiSymMatrix(Vector &v);//向量的反对称矩阵

/*******************************************************************************
在三维空间内输入三个旋转角 和矩阵排列顺序，得到一个旋转矩阵。order由1-12分别对应
  XYZ   XZY   YXZ   YZX   ZXY   ZYX   XYX   XZX   YXY   YZY   ZXZ   ZYZ
  ！！矩阵排列顺序与轴的旋转顺序相反！！（如旋转顺序为Z轴->X轴->Y轴，则选择order=3）
********************************************************************************/
	friend Matrix RotationMatrix(double x_angle, double y_angle, double z_angle, int order);
};

Matrix RotationMatrix(double, double, double, int);