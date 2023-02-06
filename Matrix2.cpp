#include"Matrix2.h"
#include<iostream>
void Matrix::Initialize()
{
	M = new double[MatrixRows * MatrixCols];
}


Matrix::Matrix(int rows, int cols, double value)
{
	MatrixRows = rows;
	MatrixCols = cols;
	Initialize();

	for (int i = 0; i < MatrixRows * MatrixCols; i++)
			M[i] = value;
}

Matrix::Matrix(int rows, int cols, char delimiter, ...)
{
	MatrixRows = rows;
	MatrixCols = cols;
	Initialize();

	va_list p;//指向参数的指针
	va_start(p, delimiter);//初始化指向参数的指针，第二个参数是可变参数的前一个参数。
	
	for (int i = 0; i < MatrixRows * MatrixCols; i++)
	{
		M[i] = va_arg(p, double);//获取可变参数，第二个参数为可变参数的类型。
	}

	va_end(p);
}

Matrix::~Matrix()
{
	if (M != NULL)
	{
		delete[]M;
		M = NULL;
	}
}
//重写复制构造函数。否则在函数返回值为对象时会将指向动态空间的指针直接相等，发生危险。
Matrix::Matrix(const Matrix& m)
{
	 MatrixRows = m.MatrixRows;
	 MatrixCols = m.MatrixCols;
	 M = new double[m.MatrixRows * m.MatrixCols];
	for (int i = 0; i < m.MatrixRows * m.MatrixCols; i++)
		M[i] = m.M[i];
}

Matrix& Matrix::operator=(const Matrix& m)
{
	if (this == &m)
		return *this;

	if (MatrixRows != m.MatrixRows || MatrixCols != m.MatrixCols)
	{
		delete[]M;
		MatrixRows = m.MatrixRows;
		MatrixCols = m.MatrixCols;
		Initialize();
	}

	for (int i = 0; i < MatrixRows*MatrixCols; i++)
			M[i] = m.M[i];

	return *this;

}

Matrix& Matrix::operator=(double Array[])
{
	for (int i = 0; i < MatrixRows*MatrixCols; i++)
			M[i] = Array[i];

	return *this;
}

void Matrix::assign(int i, int j, double value)
{
	if (i > MatrixRows || j > MatrixCols || i < 1 || j < 1)
	{
		printf("The index is over the Matrix size");
		return ;
	}

	M[(i - 1) * MatrixCols + (j - 1)] = value;
}

Matrix Matrix::operator+(const Matrix& m)const
{
	Matrix sumM(MatrixRows,MatrixCols);

	if (MatrixRows != m.MatrixRows || MatrixCols != m.MatrixCols)
	{
		printf("The number of rows or columns in matrix addition is inconsistent\n");
		return sumM;
	}

	for (int i = 0; i < MatrixRows * MatrixCols; i++)
		sumM.M[i] = M[i] + m.M[i];

	return sumM;
}

Matrix Matrix::operator-(const Matrix& m)const
{
	Matrix subM(MatrixRows, MatrixCols);

	if (MatrixRows != m.MatrixRows || MatrixCols != m.MatrixCols)
	{
		printf("The number of rows or columns in matrix subtraction is inconsistent\n");
		return subM;
	}

	for (int i = 0; i < MatrixRows * MatrixCols; i++)
		subM.M[i] = M[i] - m.M[i];

	return subM;
}

Matrix Matrix::operator*(const Matrix& m)const
{
	Matrix mulM(MatrixRows, m.MatrixCols);

	if (MatrixCols != m.MatrixRows)
	{
		printf("The number of rows or columns in matrix multiply is inconsistent\n");
		return mulM;
	}

	for (int i = 0; i < MatrixRows; i++)
		for (int j = 0; j < m.MatrixCols; j++)
			for (int k = 0; k < MatrixCols; k++)
				mulM.M[i * m.MatrixCols + j] += M[i * MatrixCols + k] * m.M[m.MatrixCols * k + j];

	return mulM;
}

Matrix operator*(const double C, const Matrix& m)
{
	Matrix mulM(m.MatrixRows, m.MatrixCols);

	for (int i = 0; i < m.MatrixRows * m.MatrixCols; i++)
		mulM.M[i] = C * m.M[i];

	return mulM;
}

Matrix operator*(const Matrix& m, const double C)
{
	Matrix mulM(m.MatrixRows, m.MatrixCols);

	for (int i = 0; i < m.MatrixRows * m.MatrixCols; i++)
		mulM.M[i] = C * m.M[i];

	return mulM;
}

Matrix Matrix::T()const
{
	Matrix mT(MatrixCols, MatrixRows);

	for (int i = 0; i < mT.MatrixRows; i++)
		for (int j = 0; j < mT.MatrixCols; j++)
			mT.M[i * mT.MatrixCols + j] = M[j * MatrixCols + i];

	return mT;

}

double Matrix::det()const
{
	if (MatrixRows != MatrixCols)
	{
		printf("Only square matrices can find the determinant\n");
		return -1;
	}

	double Result = 0.0;
	//二阶及一阶行列式的直接求解
	if (MatrixRows <= 2)
	{
		switch (MatrixRows)
		{
		case 1:Result = M[0];
			break;
		case 2:Result = M[0] * M[3] - M[1] * M[2];
			break;
		}
		return Result;
	}
	
	//每次都按第一行展开，且只需要展开第一行。定义任意高阶行列式首行某列元素的列序号。此元素的余子式赋值时将跳过本列
	for (int FirstRow_COL = 0; FirstRow_COL < MatrixCols; FirstRow_COL++)
	{
		//定义余子式数组
		Matrix Cofactor(MatrixRows - 1, MatrixCols - 1);
		int pos = 0;
		//赋值该元素的余子式数组。从第一行展开，故余子式用到的为二行及以后的数值。
		for (int row = 1; row < MatrixRows; row++)
		{
			for (int col = 0; col < MatrixCols; col++)
			{
				if (col == FirstRow_COL)
					continue;

				Cofactor.M[pos++] = M[row * MatrixCols + col];
			}
		}

		//解决代数余子式的符号。某元素的行数i，列数j，符号为（-1）^(i+j)次方
		int sign = 1;
		if (FirstRow_COL % 2 == 1)
			sign = -1;
		
		Result += M[FirstRow_COL] * sign * Cofactor.det();

	}

	return Result;
}

Matrix Matrix::A()const
{
	Matrix A(MatrixRows, MatrixCols);

	if (MatrixRows != MatrixCols)
	{
		printf("Only square matrices can find the adjugate matrix\n");
		return A;
	}

	//遍历矩阵中的每个元素，求出它们的余子式数组
	for (int ROW = 0; ROW < MatrixRows; ROW++)
	{
		for (int COL = 0; COL < MatrixCols; COL++)
		{
			Matrix Cofactor(MatrixRows - 1, MatrixCols - 1);
			int pos = 0;

			for (int row = 0; row < MatrixRows; row++)
			{
				if (row == ROW)
					continue;

				for (int col = 0; col < MatrixCols; col++)
				{
					if (col == COL)
						continue;
					//给余子式数组赋值
					Cofactor.M[pos++] = M[row * MatrixCols + col];
				}
			}

			//余子式符号判断
			int sign = 1;
			if ((ROW + COL) % 2 == 1)
			{
				sign = -1;
			}
			//利用求行列式函数，将每一元素代数余子式值求出（注意余子式本身就是数值，而不是矩阵）
			A.M[COL * MatrixCols + ROW] = sign * Cofactor.det();
		}
	}

	return A;
}

Matrix Matrix::inv()const
{
	double det = this->det();
	if (det == 0)
	{
		printf("You can't invert the matrix\n");
		return *this;
	}

	Matrix invM(MatrixRows, MatrixCols);

	invM = this->A()*(1.0/det);

	return invM;
}

void Matrix::ToE(int n)
{
	//将原本的矩阵删掉
	delete[]M;
	MatrixRows = n;
	MatrixCols = n;
	Initialize();

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			if (i == j)
				M[i * n + j] = 1.0;
			else
				M[i * n + j] = 0.0;
		}
	}
}

void Matrix::Show()const
{
	for (int i = 0; i < MatrixRows; i++)
	{
		for (int j = 0; j < MatrixCols; j++)
		{
			printf("%15.6f", M[i * MatrixCols + j]);
		}
		printf("\n");
	}
}

int Matrix::getrow()const
{
	return MatrixRows;
}

int Matrix::getcol()const
{
	return MatrixCols;
}

int Matrix::getsize()const
{
	return MatrixRows * MatrixCols;
}

double Matrix::getelement(int row, int col)const
{
	if (row > MatrixRows || col > MatrixCols|| row < 1 || col < 1 )
	{
		printf("The index is over the Matrix size");
		return -1;
	}

	return M[(row - 1) * MatrixCols + col - 1];
}

Matrix RotationMatrix(double x_angle, double y_angle, double z_angle, int order)
{
	Matrix X(3, 3);
	X.M[0] = 1.0; X.M[1] = 0.0; X.M[2] = 0.0;
	X.M[3] = 0.0;X.M[4] = cos(x_angle); X.M[5] = sin(x_angle);
	X.M[6] = 0.0; X.M[7] = -sin(x_angle); X.M[8] = cos(x_angle);

	Matrix Y(3, 3);
	Y.M[0] = cos(y_angle); Y.M[1] = 0.0; Y.M[2] = -sin(y_angle);
	Y.M[3] = 0.0; Y.M[4] = 1.0; Y.M[5] = 0.0;
	Y.M[6] = sin(y_angle); Y.M[7] = 0.0; Y.M[8] = cos(y_angle);

	Matrix Z(3, 3);
	Z.M[0] = cos(z_angle); Z.M[1] = sin(z_angle); Z.M[2] = 0.0;
	Z.M[3] = -sin(z_angle); Z.M[4] = cos(z_angle); Z.M[5] = 0.0;
	Z.M[6] = 0.0; Z.M[7] = 0.0; Z.M[8] = 1.0;

	switch (order)
	{
	case 1:return(X * Y * Z);
	case 2:return(X * Z * Y);
	case 3:return(Y * X * Z);
	case 4:return(Y * Z * X);
	case 5:return(Z * X * Y);
	case 6:return(Z * Y * X);
	case 7:return(X * Y * X);
	case 8:return(X * Z * X);
	case 9:return(Y * X * Y);
	case 10:return(Y * Z * Y);
	case 11:return(Z * X * Z);
	case 12:return(Z * Y * Z);
	default:
		printf("The order should be chosen from 1~12");
		return X;
		break;
	}
}