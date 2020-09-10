/*
 * @Author: JohnJeep
 * @Date: 2020-06-09 09:42:07
 * @LastEditTime: 2020-06-09 10:33:53
 * @LastEditors: Please set LastEditors
 * @Description: 实现多态的例子
 */ 
#include <iostream>
using namespace std;

class Hero
{
private:
    int blood;
public:
    Hero(/* args */);
    ~Hero();
    virtual int showBlood()
    {
        int a = 50;
        cout << "hero blood: " << a << endl;
        return a;
    }
};

Hero::Hero(/* args */)
{
}

Hero::~Hero()
{
}

class HeroShooter : public Hero
{
private:
    int blood;
public:
    HeroShooter(/* args */);
    ~HeroShooter();
    virtual int showBlood()
    {
        int a = 95;
        cout << "shooter hero blood: " << a << endl;
        return a;
    }
};

HeroShooter::HeroShooter(/* args */)
{
}

HeroShooter::~HeroShooter()
{
}


class Enemy
{
private:
    int blood;
public:
    Enemy(/* args */);
    ~Enemy();
    int showEnemyBlood()
    {
        int t = 90;
        cout << "enemy blood: " << t << endl;
        return t;
    }
};

Enemy::Enemy(/* args */)
{
}

Enemy::~Enemy()
{
}

// 采用虚函数的方式实现
// Hero *obj 父类的指针直接指向子类的对象
void duelTwoHero(Hero *obj, Enemy *eny)
{
    if (obj->showBlood() > eny->showEnemyBlood())
    {
        cout << "hero was winned." << endl;
    }
    else
    {
        cout << "hero was killed." << endl;
    }

}

int main()
{
    Hero yasuo;
    HeroShooter angle;
    Enemy gailun;

    duelTwoHero(&yasuo, &gailun);
    duelTwoHero(&angle, &gailun);

    return 0;
}

