#include<string.h>


void SubString(int BeginPosition, int EndPosition, char string[],char substring[])
{
	//ÿ��ȡ���ַ���ʱ�Ȱ����ַ����е�����ȫ����ʼ����ע��ѭ����������ֱ��ʹi<strlen(substring)�������һ������ֵ���⵽ '\0'ʱ������Ϊ����Ϊ0����������ֵѭ��
	int len = strlen(substring);
	for (int i = 0; i <len; i++)
	{
		substring[i] = '\0';
	}


	for (int i = BeginPosition; i <= EndPosition; i++)
	{
		*(substring++) = string[i-1];
	}
	

}


/***********************************************************************************************************

	ȡһ���ַ����飬���䰴ָ���ָ����ָ�Ϊ�ö�ά�ַ���������ʾ�Ķ���ַ����顣�൱��һ���ַ�������

************************************************************************************************************/
void StringSplit(char str[], const char* delim, char substring[][100])
{
	char* p = strtok(str, delim);
	
	
	int k=0;
	while (p != NULL)
	{
		                   
		strcpy(substring[k], p);
		k++;
		p = strtok(NULL, delim);

	}

	
}





/***********************************************************************************************************

     �ҵ��ַ������е�һ�γ���ĳ���ַ���λ�ã��Ҳ����򷵻�0

	 �� FindCharFirstPos("16*8bbb5be3", '*')=3

************************************************************************************************************/

int FindCharFirstPos(char string[], char c)
{
	//���ҡ�c������������c��ͷ���ַ�ָ�룬��strchr("16*8bbb5be3", '*')������*8bbb5be3���Ҳ����򷵻ؿ�ֵ�� 
	char* p = strchr(string, c);

	if (p)
	{

		int plength = strlen(p);
		int strlength = strlen(string);
		return(strlength - plength + 1);

	}
	else
		return 0;
}
