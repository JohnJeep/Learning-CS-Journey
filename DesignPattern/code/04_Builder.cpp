/*
 * @Author: JohnJeep
 * @Date: 2020-09-10 15:28:39
 * @LastEditTime: 2020-09-14 14:57:03
 * @LastEditors: Please set LastEditors
 * @Description: 建造者模式理解
 *                核心思想：一个对象的构建比较复杂，将一个对象的构造和对象的表示进行分离。
 *                房子类：只负责表示房子的属性
 *                工程队：负责房子各种方法的实现
 *                建筑师类：负责搭工程队构建实现房子的方法，实现了房子的下层构造与上层调用业务逻辑上良好的分离
 */
#include <iostream>
#include <cstdio>
#include <string>

using namespace std;

// 房子类的属性
class House
{
private:
    string m_floor;
    string m_wall;
    string m_window;
public:
    House() {}
    ~House() {}
    string getWall();
    string getWindow();
    string getFloor();

    void setWall(string wall)
    {
        this->m_wall = wall;
    }
    void setWindow(string window)
    {
        this->m_window = window;
    }
    void setFloor(string floor)
    {
        this->m_floor = floor;
    }
};

string House::getWall()
{
    cout << m_wall << endl; 
    return m_wall;
}

string House::getWindow()
{
    cout << m_window << endl;
    return m_window;
}

string House::getFloor()
{
    cout << m_floor << endl;
    return m_floor;
}

// 抽象的父类工程队
class Builder
{
private:
    /* data */
public:
    Builder(/* args */) {}
    virtual ~Builder() {}
    virtual void buildWall() = 0; 
    virtual void buildWindow() = 0; 
    virtual void buildfloor() = 0;
    virtual House* getHouse() = 0 ; 
};

// 子类：建造公寓的工程队
class ApartmentBuild : public Builder 
{
private:
    House* m_house;
public:
    ApartmentBuild() 
    {
        m_house = new House;
    }
    ~ApartmentBuild() 
    {

    }
    virtual void buildWall()
    {
        m_house->setWall("wall of apartment");    
    }
    virtual void buildWindow()
    {
        m_house->setWindow("window of apartment");
    }
    virtual void buildfloor()
    {
        m_house->setFloor("floor of apartment");
    }
    virtual House* getHouse()
    {
        return m_house;
    }
};

// 子类：建造平房工程队
class BungalowBuild : public Builder 
{
private:
    House* m_house;
public:
    BungalowBuild() 
    {
        m_house = new House;   // 实例化一个房子的对象
    }
    ~BungalowBuild() 
    {

    }
    virtual void buildWall()
    {
        m_house->setWall("wall of bungalow");
    }
    virtual void buildWindow()
    {
        m_house->setWindow("window of bungalow");
    }
    virtual void buildfloor()
    {
        m_house->setFloor("floor of bungalow");
    }
    virtual House* getHouse()
    {
        return m_house;
    }
};

// 子类：建造别墅工程队
class VillaBuild : public Builder 
{
private:
    House* m_house;
public:
    VillaBuild() 
    {
        m_house = new House;
    }
    ~VillaBuild() 
    {

    }
    virtual void buildWall()
    {
        m_house->setWall("wall of villa");
    }
    virtual void buildWindow()
    {
        m_house->setWindow("window of villa");
    }
    virtual void buildfloor()
    {
        m_house->setFloor("floor of villa");
    }
    virtual House* getHouse()
    {
        return m_house;
    }
};

// 建筑师类；注意：代码需要扩展时，则把建筑师类也划分为父类和子类来进行实现
class Architect
{
private:
    Builder* m_builder;
public:
    Architect(Builder* builder);
    ~Architect();
    void construct()
    {
        m_builder->buildWall();
        m_builder->buildWindow();
        m_builder->buildfloor();
    }
};

Architect::Architect(Builder* builder) 
{
    this->m_builder = builder;
}

Architect::~Architect()
{
}

int main(int argc, char *argv[])
{
    House* house = nullptr;
    Builder* builder = nullptr;
    Architect* art = nullptr;
    
    builder = new ApartmentBuild;   // 请一个公寓工程队
    art = new Architect(builder);   // 建筑师指挥工程队干活
    art->construct();
    house = builder->getHouse();    // 得到公寓工程队构建的房子对象
    house->getWall();
    house->getWindow();
    house->getFloor();
    delete house;
    delete builder;

    // 请一个别墅工程队构建房子
    builder = new VillaBuild;
    art = new Architect(builder);
    art->construct();
    house = builder->getHouse();    // 得到公寓工程队构建的房子对象
    house->getWall();
    house->getWindow();
    house->getFloor();
    delete house;
    delete builder;

    delete art;

    return 0;
}