/*
 * @Author: JohnJeep
 * @Date: 2020-09-18 11:44:45
 * @LastEditTime: 2020-09-18 14:34:10
 * @LastEditors: Please set LastEditors
 * @Description: 访问者模式
 * 
 */
 #include <iostream>
 #include <cstdio>
 #include <list>

 using namespace std;
 
class AbstractVisitor;
class AbstractPark
{
private:
    /* data */
public:
    AbstractPark(/* args */) {}
    virtual ~AbstractPark() {}
    virtual void accept(AbstractVisitor* vis) = 0;
};

class AbstractVisitor
{
private:
    /* data */
public:
    AbstractVisitor(/* args */) {}
    virtual ~AbstractVisitor() {}
    virtual void tour(AbstractPark *park) = 0;
};

class ParkA : public AbstractPark
{
private:
    /* data */
public:
    ParkA(/* args */) {}
    ~ParkA() {}

    // 公园接收游客的访问，让游客做自己的事
    virtual void accept(AbstractVisitor* vis)  
    {
        vis->tour(this);   // this指向当前开放的公园
    }
};

class ParkB : public AbstractPark
{
private:
    /* data */
public:
    ParkB(/* args */) {}
    ~ParkB() {}
    virtual void accept(AbstractVisitor* vis)
    {
        vis->tour(this);
    }    
};

// 全部的公园类
class TotalPark : public AbstractPark
{
private:
    list<AbstractPark*> m_lt;
public:
    TotalPark() 
    {
        m_lt.clear();
    }
    ~TotalPark() 
    {}

    void setPark(AbstractPark* p)   // 向公园的list中添加元素
    {
        m_lt.push_back(p);
    }
    virtual void accept(AbstractVisitor* vis)   // 公园去接待游客
    {
        for (list<AbstractPark*>::iterator iter = m_lt.begin(); iter != m_lt.end(); iter++)
        {
            (*iter)->accept(vis);   // 每一个公园都接待游客
        }
    }
};

class VisitorA : public AbstractVisitor
{
private:
    /* data */
public:
    VisitorA(/* args */) {}
    ~VisitorA() {}
    virtual void tour(AbstractPark *park)  // VisitorA类中组合了AbstractPark类
    {
        cout << "VisitorA tour parkA" << endl;
    }
};

class VisitorB : public AbstractVisitor
{
private:
    /* data */
public:
    VisitorB(/* args */) {}
    ~VisitorB() {}
    virtual void tour(AbstractPark *park) 
    {
        cout << "VisitorB tour parkB" << endl;
    }
};

class TourGroup : public AbstractVisitor
{
private:
    /* data */
public:
    TourGroup(/* args */) {}
    ~TourGroup() {}
    virtual void tour(AbstractPark *park) 
    {
        cout << "tour group lead visitor go sightseeing park" << endl;
    }
};

int main(int argc, char *argv[])
{
    AbstractPark* pa = new ParkA;
    AbstractPark* pb = new ParkB;
    AbstractVisitor* va = new  VisitorA;
    AbstractVisitor* vb = new  VisitorB;

    // 公园接待游客访问
    pa->accept(va);
    pb->accept(vb);

    AbstractVisitor * tg = new TourGroup;
    TotalPark* tpark = new TotalPark;
    tpark->setPark(pa);    // 开放整个公园中的pa公园
    tpark->setPark(pb);
    tpark->accept(tg);     // 整个公园接待旅游团

    delete tpark;
    delete tg;
    delete pa;
    delete pb;
    delete va;
    delete vb;

    return 0;
}