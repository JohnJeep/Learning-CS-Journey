/*
ʹ��Stack���Balance��������,��"("��"]"��"}"��һϵ�е��ַ������ 
*/

#include <stdio.h> 

char S[100];
int Top, Number_of_Items = 0; //Number_of_Itemsջ�����ж��ٸ��� ��TopΪջ��ǰ����� 

/*��ջ����*/ 
void Push(char c)
{
	//�жϵ�һ�����Ƿ�Ϊ0 
	if(Number_of_Items == 0)
	{
		Top = 0;
		S[0] = c; 
	}
	//�жϵ�һ������Ϊ0 
	else
	{
		Top++; 
		S[Top] = c; 
	}
	Number_of_Items++;
} 

/*��ջ����*/
void Pop()
{
	Top--;
	Number_of_Items--;
} 

/*�鿴ջ����˵���*/
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
	printf("����������balance�ķ���: ");
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
				printf("����һ��balance�� \n" );
				//return 0;
			}
			else
				Pop();
		}
		
		else if(input == ']' )
		{
			if(IsEmpty() || TopItem() != '[')
			{
				printf("����һ��balance�� \n" );
				return 0;
			}
			else
				Pop();		
		}
		
		else if(input == '}' )
		{
			if(IsEmpty() || TopItem() != '{')
			{
				printf("����һ��balance�� \n" );
				return 0;
			}
			else
				Pop();		
		}
		scanf("%c", &input);
	}

	if (IsEmpty())  // ����Ϊ��ֵ
		printf("��balance! ");
	else	
	{
		printf("����balance \n");
	}
		
}

