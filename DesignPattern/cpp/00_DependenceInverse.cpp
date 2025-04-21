/*
 * @Author: JohnJeep
 * @Date: 2020-09-08 08:53:07
 * @LastEditTime: 2020-09-08 14:53:58
 * @LastEditors: Please set LastEditors
 * @Description: 设计模式的基本原则：依赖倒转
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

// 基类
class Cpu
{
private:
    /* data */
public:
    Cpu() {}
    virtual ~Cpu() {}
    virtual void work() = 0;
};

class Memory
{
private:
    /* data */
public:
    Memory() {}
    virtual ~Memory() {}
    virtual void work() = 0;
};

class Disk
{
private:
    /* data */
public:
    Disk();
    virtual ~Disk();
    virtual void work() = 0;
};

Disk::Disk()
{
}
Disk::~Disk()
{
    cout << "我是父类" << endl; 
}

// 具体的接口类
class InterCpu : public Cpu
{
private:
    /* data */
public:
    InterCpu() {}
    ~InterCpu() {}
    virtual void work() 
    {
        cout << "Inter CPU work." << endl;
    } 
};

class JSDMemory : public Memory
{
private:
    /* data */
public:
    JSDMemory() {}
    ~JSDMemory() {}
    virtual void work() 
    {
        cout << "JSD memory work." << endl;
    }     
};

class XJDisk : public Disk
{
private:
    /* data */
public:
    XJDisk();
    ~XJDisk();
    virtual void work() 
    {
        cout << "XJ disk work." << endl;
    }     
};
XJDisk::XJDisk()
{
}

XJDisk::~XJDisk()
{
    cout << "我是子类" << endl; 
}

class Computer
{
private:
    Cpu *m_cpu;
    Memory *m_memory;
    Disk *m_disk;
public:
    Computer(Cpu *pcpu, Memory *pmemory, Disk *pdisk);
    ~Computer();
    void handleWork()
    {
        m_cpu->work();
        m_memory->work();
        m_disk->work();
    }
};

Computer::Computer(Cpu *pcpu, Memory *pmemory, Disk *pdisk)
: m_cpu(pcpu), m_memory(pmemory), m_disk(pdisk)
{
}

Computer::~Computer()
{
}

int main(int argc, char *argv[])
{
    Cpu *t_cpu = new InterCpu;  // 父类指针指向子类的对象 
    Memory *t_memory = new JSDMemory;
    Disk *t_disk = new XJDisk;
    Computer *dell = new Computer(t_cpu, t_memory, t_disk);
    dell->handleWork();

    delete dell;
    delete t_disk;
    delete t_memory;
    delete t_cpu;

    return 0;
}