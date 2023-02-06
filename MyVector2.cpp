#include"MyVector2.h"
#include<iostream>
//���û��๹�캯����ʼ��
Vector::Vector(int n,double value):Matrix(n, 1, value)
{
	
}
/*�����Ͷ�������ֵ������˵������һά�ľ���Ӧ�ÿ���ʹ�þ��������=�ĸ�ֵ���������������Ի��ǲ���
��Ϊ�����������ԶԻ����������ó�ʼ����ֵ�����ԼӼ������˷������˶������þ���ģ�����ֵ������Ҫ��д*/
Vector& Vector::operator=(const Vector &v)
{
	if (this == &v)
		return *this;
	
	if (MatrixRows != v.MatrixRows)
	{
		delete[] M;
		MatrixRows = v.MatrixRows;
		Initialize();
	}

	for (int i = 0; i < MatrixRows; i++)
		M[i] = v.M[i];

	return *this;
}

Vector& Vector::operator=(const Matrix& m)
{
	if (m.getcol() != 1)
	{
		printf("The columns of Matrix must be 1");
		return *this;
	}

	if (MatrixRows != m.getrow())
	{
		delete[] M;
		MatrixRows = m.getrow();
		Initialize();
	}

	for (int i = 0; i < MatrixRows; i++)
		M[i] = m.getelement(i + 1, 1);

	return *this;

}

Vector& Vector::operator=(const double Array[])
{
	for (int i = 0; i < MatrixRows; i++)
		M[i] = Array[i];

	return *this;
}

Vector& Vector::GetFormArray(int pos, const double Array[])
{
	for (int i = 0; i < MatrixRows; i++)
		M[i] = Array[pos + i];

	return *this;
}

double Vector::Norm()const
{
	//ģ�������Լ�ת�ó����Լ��ٿ�����
	return(sqrt((this->T() * (*this)).getelement(1, 1)));
}

Vector Vector::operator->*(const Vector&v)const
{
	//��֧����ά�����������
	if (MatrixRows != 3 || v.MatrixRows != 3)
	{
		printf("only 3 dimension can do the cross product");
		return *this;
	}
	Vector mulv(3);

	mulv.M[0] = M[1] * v.M[2] - M[2] * v.M[1];
	mulv.M[1] = -(M[0] * v.M[2] - M[2] * v.M[0]);
	mulv.M[2] = M[0] * v.M[1] - M[1] * v.M[0];

	return mulv;
}

void Vector::ToUnit()
{
	*this=*this* (1.0 / this->Norm());
	return;
}

Matrix AntiSymMatrix(Vector &v)
{
	Matrix m(3, 3);

	if (v.MatrixRows != 3)
	{
		printf("only 3 dimension can find the Antisymmetric matrix");
		return m;
	}

	m.M[0] = m.M[4] = m.M[8] = 0.0;
	m.M[1] = -v.M[2];
	m.M[2] = v.M[1];
	m.M[3] = v.M[2];
	m.M[5] = -v.M[0];
	m.M[6] = -v.M[1];
	m.M[7] = v.M[0];
	
	return m;
}