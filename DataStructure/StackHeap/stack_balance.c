/*
使用Stack解决Balance括号问题,即"("、"]"、"}"等一系列的字符能配对 
*/

#include <stdio.h> 

char S[100];
int Top, Number_of_Items = 0; //Number_of_Items栈里面有多少个数 ，Top为栈最前面的数 

/*进栈操作*/ 
void Push(char c)
{
	//判断第一个数是否为0 
	if(Number_of_Items == 0)
	{
		Top = 0;
		S[0] = c; 
	}
	//判断第一个数不为0 
	else
	{
		Top++; 
		S[Top] = c; 
	}
	Number_of_Items++;
} 

/*出栈操作*/
void Pop()
{
	Top--;
	Number_of_Items--;
} 

/*查看栈的最顶端的数*/
char TopItem()
{	
		return S[Top];
}

int IsEmpty()
{
	if (Number_of_Items == 0)
		return 1;
	else
		return 0;
}

int main()
{
	char input;
	//Input();
	printf("请输入任意balance的符号: ");
	scanf("%c", &input);
	
	
	while(input != '\n')
	{
		if(input == '(' || input == '[' || input == '{') 
		{
			Push(input);	
		}
		else if(input == ')' )
		{			
			if(IsEmpty() || TopItem() != '(')
			{
				printf("不是一个balance！ \n" );
				//return 0;
			}
			else
				Pop();
		}
		
		else if(input == ']' )
		{
			if(IsEmpty() || TopItem() != '[')
			{
				printf("不是一个balance！ \n" );
				return 0;
			}
			else
				Pop();		
		}
		
		else if(input == '}' )
		{
			if(IsEmpty() || TopItem() != '{')
			{
				printf("不是一个balance！ \n" );
				return 0;
			}
			else
				Pop();		
		}
		scanf("%c", &input);
	}

	if (IsEmpty())  // 输入为空值
		printf("是balance! ");
	else	
	{
		printf("不是balance \n");
	}
		
}

