/*
 * @Author: JohnJeep
 * @Date: 2020-09-18 14:39:07
 * @LastEditTime: 2020-09-20 17:26:43
 * @LastEditors: Please set LastEditors
 * @Description: 迭代模式实现：迭代器对谁创建，就持有这个迭代器的引用。
 *               实现：通过迭代器的模式去访问聚合类中的数据
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

#define SIZE   5
typedef int Object;

// 抽象迭代器类
class AbstractIterator
{
private:
    /* data */
public:
    AbstractIterator(/* args */) {}
    virtual ~AbstractIterator() {}

    virtual void first() = 0;           // 获取第一个元素
    virtual void next() = 0;            // 访问下一个元素
    virtual bool isDone() = 0;          // 判断是否还有下一个元素
    virtual Object currentIter() = 0;   // 获取具体聚合类中当前元素的值
};

// 抽象聚合类
class AbstractAggregate
{
private:
    /* data */
public:
    AbstractAggregate(/* args */) {}
    virtual ~AbstractAggregate() {}

    virtual AbstractIterator* createIter() =0;  // 创建一个迭代器
    virtual Object getIter(int index) = 0;      // 得到聚合类中数据的值
    virtual int getSize() = 0;                  // 得到聚合类中数据的大小
};

// 具体的迭代器类，完成对聚合对象的遍历，在具体迭代器中通过游标来记录在聚合对象中所处的当前位置，
// 在具体实现时，游标通常是一个表示位置的非负整数。
class ConcreteIterator : public AbstractIterator
{
private:
    AbstractAggregate* m_ag;   // 在迭代器中持有一个聚合的引用，通过迭代器去访问聚合类
    int m_index;               // 迭代器中定义索引的位置
public:
    ConcreteIterator(AbstractAggregate* ag) 
        : m_ag(ag), m_index(0)
    {
    }
    ~ConcreteIterator() 
    {
    }

    virtual void first()
    {
        m_index = 0;
    }
    virtual void next()
    {
        if (m_index < m_ag->getSize())
        {
            m_index++;
        }
    }
    virtual bool isDone()
    {
        return (m_index == m_ag->getSize());
    }
    virtual Object currentIter()
    {
        return m_ag->getIter(m_index);
    }
};

// 具体的聚合类
class ConcreteAggregate : public AbstractAggregate
{
private:
    Object obj[SIZE];
public:
    ConcreteAggregate() 
    {
        for (int i = 0; i < SIZE; i++)
        {
            obj[i] = i + 10;
        }
    }
    ~ConcreteAggregate() 
    {
    }
    virtual Object getIter(int index)
    {
        return obj[index];
    }
    virtual int getSize()
    {
        return SIZE;
    }
    virtual AbstractIterator* createIter()
    {
        return new ConcreteIterator(this);
    }
};

int main(int argc, char *argv[])
{
    
    AbstractAggregate* myAbstractAggregate = new ConcreteAggregate;    // 创建一个聚合对象
    AbstractIterator* myIter = myAbstractAggregate->createIter();      // 创建一个遍历聚合类对象的迭代器
    
    // 通过迭代器遍历聚合类
    for (; !myIter->isDone(); myIter->next())
    {
        cout << myIter->currentIter() << " ";
    }
    delete myIter;
    delete myAbstractAggregate;

    return 0;
}
