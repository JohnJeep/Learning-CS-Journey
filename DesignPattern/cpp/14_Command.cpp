/*
 * @Author: JohnJeep
 * @Date: 2020-09-16 14:37:03
 * @LastEditTime: 2020-09-17 12:05:32
 * @LastEditors: Please set LastEditors
 * @Description: 命令模式实现
 * 
 */
#include <iostream>
#include <cstdio>
#include <list>

using namespace std;

class Doctor
{
private:
    /* data */
public:
    Doctor(/* args */) {}
    ~Doctor() {}
    void cureHeart()
    {
        cout << "Cure heart" << endl;
    }
    void cureRenal()
    {
        cout << "Cure renal" << endl;
    }
};

class AbstractCommand
{
private:
    /* data */
public:
    AbstractCommand(/* args */) {}
    virtual ~AbstractCommand() {}
    virtual void treat() = 0;
};

class CommandHeartCase : public AbstractCommand
{
private:
    Doctor* m_doctor;
public:
    CommandHeartCase(Doctor* doc) 
        : m_doctor(doc)
    {
    }
    ~CommandHeartCase() 
    {
    }
    void treat()
    {
        m_doctor->cureHeart();
    }
};

class CommandRenalCase : public AbstractCommand
{
private:
    Doctor* m_doctor;
public:
    CommandRenalCase(Doctor* doc) 
        : m_doctor(doc)
    {
    }
    ~CommandRenalCase() 
    {
    }
    void treat()
    {
        m_doctor->cureRenal();
    }
};

// 护士处理单个病例
class Nurse
{
private:
    AbstractCommand* m_cmd;
public:
    Nurse(AbstractCommand* cmd)
        :m_cmd(cmd) 
    {}
    ~Nurse() 
    {}
    void submitCase()
    {
        m_cmd->treat();
    }
};



// 护士长处理多个病例
class HeadNurse
{
private:
    list<AbstractCommand*> m_cmd;
public:
    HeadNurse() 
    {
        m_cmd.clear();   // 清空list中原先的元素
    }
    ~HeadNurse() 
    {
    }
    void dealCase(AbstractCommand* cmd)
    {
        m_cmd.push_back(cmd);
    }
    void submitCase()  // 护士收集提交的病例
    {
        list<AbstractCommand*>::iterator iter;
        for (iter = m_cmd.begin(); iter != m_cmd.end(); iter++)
        {
            (*iter)->treat();
        }
    }
};

int main(int argc, char *argv[])
{
    // 护士处理单个病例
    Doctor* wang = new Doctor;
    AbstractCommand* cmd = new CommandHeartCase(wang); // 根据不同的病例安排不同的医生
    Nurse* li = new Nurse(cmd);   // 护士收集病例
    li->submitCase();
    
    delete li;
    delete cmd;
    delete wang;

    // 护士长处理多个病例
    cout << "-------------------------" << endl;
    Doctor* zhao = new Doctor;
    AbstractCommand* case1 = new CommandHeartCase(zhao);
    AbstractCommand* case2 = new CommandHeartCase(zhao);
    HeadNurse* lucy = new HeadNurse();
    lucy->dealCase(case1);   // 处理病例
    lucy->dealCase(case2);
    lucy->submitCase();     // 根据不同的病例，让医生去处理

    return 0;
}