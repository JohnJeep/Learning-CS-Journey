/*
 * @Author: JohnJeep
 * @Date: 2020-09-15 14:23:52
 * @LastEditTime: 2020-09-15 14:54:18
 * @LastEditors: Please set LastEditors
 * @Description: 桥接模式的实现
 *               描述：不同类型的车安装不同类型的发动机。
 *                    不同类型车的子类与发动机的抽象父类进行组合，不同车型的类继承车的抽象类。
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Engine
{
private:
    /* data */
public:
    Engine(/* args */) {}
    virtual ~Engine() {}
    virtual void machineModel() = 0; 
};

class DJModel : public Engine
{
private:
    /* data */
public:
    DJModel(/* args */) {}
    ~DJModel() {}
    virtual void machineModel()
    {
        cout << "DJ MODEL BVFG324" << endl;
    }
};

class BHAModel : public Engine
{
private:
    /* data */
public:
    BHAModel(/* args */) {}
    ~BHAModel() {}
    virtual void machineModel()
    {
        cout << "BHA MODEL ACD725" << endl;
    }    
};

class HJPModel : public Engine
{
private:
    /* data */
public:
    HJPModel(/* args */) {}
    ~HJPModel() {}
    virtual void machineModel()
    {
        cout << "HJP MODEL KYT568" << endl;
    } 
};

class Car
{
private:
    /* data */
public:
    Car(/* args */) {}
    virtual ~Car() {}
    virtual void getInfo() = 0;
};

class Benz : public Car
{
private:
    Engine* m_engine;
public:
    Benz(Engine* eng) : m_engine(eng)
    {}
    ~Benz() 
    {}
    virtual void getInfo()
    {
        cout << "I am Benz: ";
        m_engine->machineModel();
    }
};

class BMW : public Car
{
private:
    Engine* m_engine;
public:
    BMW(Engine* eng) : m_engine(eng)
    {}
    ~BMW() 
    {}
    virtual void getInfo()
    {
        cout << "I am BMW: ";
        m_engine->machineModel();
    }
};

int main(int argc, char *argv[])
{
    Engine* myEng = nullptr;
    Car* myCar = nullptr;

    myEng = new DJModel;
    myCar = new Benz(myEng);
    myCar->getInfo();
    delete myEng;

    myEng = new BHAModel;
    myCar->getInfo();
    delete myEng;

    myEng = new HJPModel;
    myCar->getInfo();
    delete myEng;
    delete myCar;

    myEng = new DJModel;
    myCar = new BMW(myEng);
    myCar->getInfo();
    delete myEng;

    return 0;
}