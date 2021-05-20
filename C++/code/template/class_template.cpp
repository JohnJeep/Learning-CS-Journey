/*
 * @Author: JohnJeep
 * @Date: 2021-04-02 11:13:06
 * @LastEditTime: 2021-04-02 16:12:02
 * @LastEditors: Please set LastEditors
 * @Description: class template 实现 
 */
#include <iostream>
#include <cstdio>
#include <vector>
#include <cassert>

using namespace std;

template<typename T>
class Stack
{
private:
   vector<T> elems;

public:
    Stack()
    {
    }
    ~Stack()
    {
    }

    Stack(Stack const&)     // copy constructor
    {
    }

    Stack& operator= (Stack const&) // assignment operator
    {
        
    }

    bool my_empty() const
    {
        return elems.empty(); // reutrn whether the stack is empty
    }

    void push(T const& e)
    {
        elems.push_back(e); // append copy of passed e
    }

    T pop()
    {
        assert(!elems.empty());
        T val = elems.back();  // save copy of last element
        elems.pop_back();       // remove last element
        return val;            // return copy of saved element 
    }

    T const& top() const
    {
        assert(!elems.empty());
        return elems.back();   // retuen copy 0f last element
    }

};

int main(int argc, char *argv[])
{
    Stack<int> intSatack;
    Stack<string> stringStack;

    intSatack.push(10);
    cout << intSatack.top() << "\n";

    stringStack.push("hello");
    cout << stringStack.top() << "\n";
    stringStack.pop();

    return 0;
}
