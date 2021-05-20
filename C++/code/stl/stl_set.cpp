/*
 * @Author: JohnJeep
 * @Date: 2020-07-18 11:09:29
 * @LastEditTime: 2021-05-20 22:25:02
 * @LastEditors: Please set LastEditors
 * @Description: STL标准库set容器的用法
 */ 
#include <iostream>
#include <stdlib.h>
#include <set>

using namespace std;

void test01()
{  
    cout << "测试用例01" << endl;
    set<int> set_one;
    
    // 插入 set.insert()
    set_one.insert(11);
    set_one.insert(33);
    set_one.insert(44);
    set_one.insert(22);
    cout << "set size:" << set_one.size() << endl;

    set<int>::iterator it;
    cout << "set element: ";
    for (it = set_one.begin(); it != set_one.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;

    // 删除 set.erase()
    cout << "erase set: ";
    while (!set_one.empty())
    {
        it = set_one.begin();
        cout << *it << " ";
        set_one.erase(set_one.begin());
    }
    cout << endl;
    cout << "erase set size:" << set_one.size() << endl;
    cout << endl;
}

// 数据类型比较，测试用例
void test02()
{
    // 基础数据类型比较
    cout << "测试用例02" << endl;
    set<int> l_set;                    // 数据从小到大依次排序
    set<int, greater<int>> g_set;      // 数据从大到小依次排序
    
    // 插入 set.insert()
    g_set.insert(101);
    g_set.insert(303);
    g_set.insert(404);
    g_set.insert(202);
    g_set.insert(502);
    cout << "g_set size:" << g_set.size() << endl;    

    set<int>::iterator sit;
    cout << "g_set element: ";
    for (sit = g_set.begin(); sit != g_set.end(); sit++)
    {
        cout << *sit << " ";
    }
    cout << endl;
    cout << endl;
}

// 类对象数据类型比较
class Stu
{
private:
    int m_age;
    string m_name;

public:
    Stu(string name, int age);
    ~Stu();
    int getAge() const;
    string getName() const;
};

Stu::Stu(string name, int age)
{
    this->m_name = name;
    this->m_age = age;
}

Stu::~Stu()
{
}

int Stu::getAge() const
{
    // cout << m_age << endl;
    return m_age;
}

string Stu::getName() const
{
    // cout << m_name << "\t";
    return m_name;
}

// 定义伪函数，按照年龄排序
struct funcPseudoStu
{
    bool operator() (const Stu& left, const Stu& right) const  // 必须加 const 关键字，保证对象的成员数据在比较时不能改变
    {
        if (left.getAge() < right.getAge())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};


void test03()
{
    cout << "测试用例03" << endl;
    Stu s1("诗静依", 24);
    Stu s2("彩鳞", 23);
    Stu s3("萧薰儿", 22);
    Stu s4("绫清竹", 25);
    s1.getName();
    s1.getAge();
    s2.getName();
    s2.getAge();
    s3.getName();
    s3.getAge();
    s4.getName();
    s4.getAge();

    set<Stu, funcPseudoStu> obj_set;
    obj_set.insert(s1);
    obj_set.insert(s2);
    obj_set.insert(s3);
    obj_set.insert(s4);
    cout << "obj set size: " << obj_set.size() << endl;

    set<Stu, funcPseudoStu>::iterator obj_it;
    cout << "obj_set element: " << endl;;
    for (obj_it = obj_set.begin(); obj_it != obj_set.end(); obj_it++)
    {
        cout << "name: " << obj_it->getName() << "\t"
             << "age: "  << obj_it->getAge()  << endl;
    }
    cout << endl;
}


// 测试用例4：set容器的查找
void test04()
{
    cout << "测试用例04" << endl;
    set<int> fset;
    fset.insert(11);
    fset.insert(12);
    fset.insert(14);
    fset.insert(18);
    fset.insert(15);
    fset.insert(13);
    set<int>::iterator it;
    cout << "set origin element: ";
    for (it = fset.begin(); it != fset.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;
    it = fset.find(14);
    cout << "set find: " << *it << endl;

    int num = fset.count(18);   
    cout << "set count: " << num << endl;

    it = fset.upper_bound(15);
    cout << "set upper_bound: " << *it << endl;
    it = fset.lower_bound(15);
    cout << "set upper_bound: " << *it << endl;
    
    pair<set<int>::iterator, set<int>::iterator> eit;
    eit = fset.equal_range(14);
    set<int>::iterator ft = eit.first;
    cout << "set first: " << *ft << endl;
    set<int>::iterator st = eit.second;
    cout << "set second: " << *st << endl;

}

// 测试用例5：multiset()
void test05()
{
    cout << endl;
    cout << "测试用例05" << endl;
    multiset<int> mset;
    int num = 0;

    cout << "please input a number" << endl;
    cin >> num;
    while (num != 0)
    {
        mset.insert(num);
        cin >> num;
    }

    multiset<int>::iterator it;
    cout << "multiset element size: " << mset.size() << endl;
    cout << "multiset element: ";
    for (it = mset.begin(); it != mset.end(); it++)
    {
        cout << *it << " ";   // 输出按照从小到大的顺序排列
    }
    cout << endl;
    
    // 删除容器里面的内容
    cout << "erase element: ";
    while(!mset.empty())
    {
        it = mset.begin();      // 查看每次删除元素
        cout << *it << " ";
        mset.erase(mset.begin());
    }
    cout << endl;
    cout << "multiset element delete after size: " << mset.size() << endl;
}



int main(int argc, char *argv[])
{
    test01();
    test02();
    test03();
    test04();
    test05();

    return 0;
}