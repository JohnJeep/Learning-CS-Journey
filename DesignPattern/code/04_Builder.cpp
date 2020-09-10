/*
 * @Author: JohnJeep
 * @Date: 2020-09-10 15:28:39
 * @LastEditTime: 2020-09-10 16:13:36
 * @LastEditors: Please set LastEditors
 * @Description: 建造者模式理解
*                核心思想：一个对象的构建比较复杂，将一个对象的构造和对象的表示进行分离。
 */
#include <iostream>
#include <cstdio>
#include <string>

using namespace std;

class House
{
private:
    /* data */
public:
    House() {}
    virtual ~House() {}
    virtual void buildWall() = 0; 
    virtual void buildWindow() = 0; 
    virtual void buildfloor() = 0; 
};

class ApartmentBuild : public House 
{
private:
    /* data */
public:
    ApartmentBuild() {}
    ~ApartmentBuild() {}
    virtual void buildWall()
    {

    }
    virtual void buildWindow()
    {

    }
    virtual void buildfloor()
    {

    }
};

// 建造平房
class BungalowBuild : public House 
{
private:
    /* data */
public:
    BungalowBuild() {}
    ~BungalowBuild() {}
    virtual void buildWall()
    {

    }
    virtual void buildWindow()
    {

    }
    virtual void buildfloor()
    {
        
    }
};

class VillaBuild : public House 
{
private:
    /* data */
public:
    VillaBuild() {}
    ~VillaBuild() {}
    virtual void buildWall()
    {

    }
    virtual void buildWindow()
    {

    }
    virtual void buildfloor()
    {
        
    }
};

class Architect
{
private:
    /* data */
public:
    Architect() {}
    ~Architect() {}
};

class Worker
{
private:
    /* data */
public:
    Worker() {}
    ~Worker() {}
};

int main(int argc, char *argv[])
{
    
    return 0;
}