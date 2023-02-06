#include<string.h>


void SubString(int BeginPosition, int EndPosition, char string[],char substring[])
{
	//每次取子字符串时先把子字符串中的内容全部初始化。注意循环条件不能直接使i<strlen(substring)，否则第一个赋空值后检测到 '\0'时它就认为长度为0，跳出赋空值循环
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

	取一行字符数组，将其按指定分隔符分割为用二维字符数组来表示的多个字符数组。相当于一个字符串数组

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

     找到字符数组中第一次出现某个字符的位置，找不到则返回0

	 如 FindCharFirstPos("16*8bbb5be3", '*')=3

************************************************************************************************************/

int FindCharFirstPos(char string[], char c)
{
	//查找‘c’，并返回以c开头的字符指针，如strchr("16*8bbb5be3", '*')，返回*8bbb5be3，找不到则返回空值。 
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
