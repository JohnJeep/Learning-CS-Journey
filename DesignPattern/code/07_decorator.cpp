/*
 * @Author: JohnJeep
 * @Date: 2020-09-15 10:08:52
 * @LastEditTime: 2020-09-15 11:30:49
 * @LastEditors: Please set LastEditors
 * @Description: 装饰模式实现
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Car
{
private:
    /* data */
public:
    Car(/* args */) {}
    virtual ~Car() {}
    virtual void apply() = 0;
};

class NormalCar : public Car
{
private:
    /* data */
public:
    NormalCar() {}
    ~NormalCar() {}
    virtual void apply()
    {
        cout << "running on the road." <<endl;
    }
};

class AmphibiousCar : public Car
{
private:
    Car* m_car;
public:
    AmphibiousCar(Car* t_car) : m_car(t_car)
    {

    }
    ~AmphibiousCar()
    {

    }

    // 新增的方法
    void swimming()
    {
        cout << "swimming in the sea" << endl;
    } 
    virtual void apply()
    {
        m_car->apply();   // 调用NormalCar类中的方法
        swimming();
    }
};


class MutateCar : public Car
{
private:
    Car* m_car;
public:
    MutateCar(Car* a_car) : m_car(a_car)
    {

    }
    ~MutateCar() 
    {

    }
    // 新增的方法，采用继承和虚函数实现了多态
    void flying()
    {
        cout << "flying on the sky" << endl;
    }
    virtual void apply()
    {
        m_car->apply();
        flying();
    }
};

int main(int argc, char *argv[])
{
    Car* Aodi = nullptr;

    Aodi = new NormalCar;
    Aodi->apply();

    AmphibiousCar* Benz = new AmphibiousCar(Aodi);
    Benz->apply();

    MutateCar* Lamborghini = new MutateCar(Benz);
    Lamborghini->apply();

    delete Lamborghini;
    delete Benz;
    delete Aodi;

    return 0;
}