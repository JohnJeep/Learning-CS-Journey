/*
 * @Author: JohnJeep
 * @Date: 2020-06-04 09:55:07
 * @LastEditTime: 2020-06-08 16:06:49
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
    static int id;

    void getNum();
    void setNum(int t_num);
    void getScore();
    void setScore(int t_score);

    Stu(/* args */);    // 类的外部访问构造函数，权限应该设置为 public
    ~Stu();
};
int Stu::num = 007;    // 类的外部初始化静态成员变量
int Stu::id = 999;

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

// 继承中使用static
class Girl: private Stu
{
private:
    /* data */
public:
    Girl(/* args */);
    ~Girl();
    void show()
    {
        Stu::id++;    // 子类访问父类中 public 属性的成员变量
        cout << "id: " << id << endl;
    }
};

Girl::Girl(/* args */)
{
}

Girl::~Girl()
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

    Girl kasa;
    // kasa.getNum();   // 父类中的getNum()访问属性为private
    // kasa.id = 1000;     // 由于是private继承，此时父类中的 id 变量为 private 权限
    kasa.show();

    return 0;
}