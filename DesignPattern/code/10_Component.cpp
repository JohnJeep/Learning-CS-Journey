/*
 * @Author: JohnJeep
 * @Date: 2020-09-15 14:59:48
 * @LastEditTime: 2020-09-16 09:00:04
 * @LastEditors: Please set LastEditors
 * @Description: 组合模式的实现
 *               描述：文件中创建子文件。在目录下面添加不同的文件夹和文件。
 * 
 */
#include <iostream>
#include <cstdio>
#include <cstring>
#include <list>

using namespace std;

class File
{
private:
    /* data */
public:
    File(/* args */) {}
    virtual ~File() {}
    virtual void display() = 0;
    virtual int add(File* fp) = 0;
    virtual int remove(File* fp) = 0;
    virtual list<File*>* getSubfiles() = 0;
};

// 子文件
class SubFile : public File
{
private:
    string m_name;
public:
    SubFile(string name) : m_name(name)
    {}
    ~SubFile()
    {}

    virtual void display()
    {
        cout << m_name << endl;
    }
    virtual int add(File* fp)
    {
        return -1;
    }
    virtual int remove(File* fp)
    {
        return -1;
    }
    virtual list<File*>* getSubfiles()
    {
        return nullptr;
    }    
};

// 目录
class Dir : public File
{
private:
    string m_name;
    list<File*> *m_list;
public:
    Dir(string name) 
        : m_name(name)
    {
        m_list = new list<File*>;
        m_list->clear();
    }
    ~Dir()
    {}

    virtual void display()
    {
        cout << m_name << endl;
    }
    virtual int add(File* fp)
    {   
        m_list->push_back(fp);
        return 0;
    }
    virtual int remove(File* fp)
    {
        m_list->remove(fp);
        return 0;
    }
    virtual list<File*>* getSubfiles()
    {
        return m_list;
    }    
};

// 递归显示目录下的文件：1、显示根节点 
// 2、若目录下有孩子，判断孩子是文件还是目录，若孩子是文件，则显示文件，若孩子是目录，则显示子目录。
void  showFileTree(File* fp, int len)
{
    list<File*> *f = nullptr;
    int i = 0;

    for (i = 0; i < len; i++)
    {
        printf("\t");
    }
    fp->display();  // 显示根目录

    // 目录下面有文件
    f = fp->getSubfiles();
    if (f != nullptr)
    {
        for (list<File*>::iterator it = f->begin(); it != f->end(); it++)
        {
            if ((*it)->getSubfiles() == nullptr)
            {
                for (i = 0; i <= len; i++)
                {
                    printf("\t");
                }
                (*it)->display();  // 显示叶子结点
            }
            else
            {
                showFileTree((*it), len+1);
            }
        }
    }
}

int main(int argc, char *argv[])
{
    Dir* rootDir = new Dir("/C");
    Dir* dirA = new Dir("A");
    SubFile* file_a = new SubFile("a.txt");
    rootDir->add(dirA);
    dirA->add(file_a);
    
    Dir *dirB = new Dir("B");
    File* file_b = new SubFile("b.txt");
    rootDir->add(dirB);
    dirB->add(file_b);
    dirB->add(file_a);
    
    showFileTree(rootDir, 0);
    
    return 0;
}