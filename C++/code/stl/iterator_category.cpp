/*
 * @Author: JohnJeep
 * @Date: 2021-05-11 20:54:30
 * @LastEditTime: 2021-05-11 23:45:53
 * @LastEditors: Please set LastEditors
 * @Description: 测试 iterator 中的 5 种类型
 */
#include <iostream>
#include <cstdlib>
#include <iterator>
#include <array>
#include <vector>
#include <list>
#include <forward_list>
#include <deque>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <typeinfo>

using namespace std;

void display_iterator_category(random_access_iterator_tag)
{
    cout << "random_access_iterator " << endl;
}

void display_iterator_category(bidirectional_iterator_tag)
{
    cout << "bidirectional_iterator_tag" << endl;
}

void display_iterator_category(forward_iterator_tag)
{
    cout << "forward_iterator_tag" << endl;
}

void display_iterator_category(input_iterator_tag)
{
    cout << "input_iterator_tag" << endl;
}

void display_iterator_category(output_iterator_tag)
{
    cout << "output_iterator_tag" << endl;
}

// 第一种测试
// template<typename I>
// void display_iterator(I iter)
// {
//     typename iterator_traits<I>::iterator_category catey;
//     display_iterator_category(catey);
// }

// 第二种测试
template<typename I>
void display_iterator(I iter)
{
    typename iterator_traits<I>::iterator_category catey;
    display_iterator_category(catey);
    cout << "typeid(iter).name() " << typeid(iter).name() << endl;
}


int main(int argc, char *argv[])
{
    cout << "test_iterator_category()..." << endl;
    
    display_iterator(array<int, 10>::iterator());
    display_iterator(vector<int>::iterator());
    display_iterator(list<int>::iterator()); 
    display_iterator(forward_list<int>::iterator());
    display_iterator(deque<int>::iterator());

    display_iterator(map<int, int>::iterator());
    display_iterator(set<int>::iterator());
    display_iterator(multimap<int, int>::iterator());
    display_iterator(multiset<int>::iterator());

    display_iterator(unordered_map<int, int>::iterator());
    display_iterator(unordered_set<int>::iterator());
    display_iterator(unordered_multimap<int, int>::iterator());
    display_iterator(unordered_multiset<int>::iterator());

    display_iterator(istream_iterator<int>());
    display_iterator(ostream_iterator<int>(cout, ""));

    return 0;
}