/*
 * @Author: JohnJeep
 * @Date: 2020-06-04 09:55:07
 * @LastEditTime: 2020-06-04 12:16:58
 * @LastEditors: Please set LastEditors
 * @Description: static：静态成员变量
 */ 
#include <iostream>
using namespace std;

class Stu
{
private:
    int score;
    static int num;
public:
    void getNum();
    void setNum(int t_num);
    void getScore();
    void setScore(int t_score);

    Stu(/* args */);
    ~Stu();
};
int Stu::num = 007;    // 类的外部初始化静态成员变量


void Stu::getNum()
{
    cout << "getNum: " << num <<endl;
}

void Stu::setNum(int t_num)
{
    num = t_num;
    cout << "setNum: " << num << endl;
}

void Stu::getScore()
{
    cout << "getScore: " << score << endl;
}

void Stu::setScore(int t_score)
{
    score = t_score;
    cout << "setScore: " << score << endl;
}

Stu::Stu(/* args */)
{
}

Stu::~Stu()
{
}


int main()
{
    Stu li, wang, zhao;

    // 不同对象之间共享 num 成员变量
    li.getNum();
    li.setNum(10010);
    wang.getNum();

    // score 成员变量对象之间不共享
    li.getScore();
    li.setScore(98);
    wang.getScore();     // 得到的值并不是98

    return 0;
}