/*
 * @Author: JohnJeep
 * @Date: 2021-02-02 16:00:55
 * @LastEditTime: 2021-02-02 16:14:22
 * @LastEditors: Please set LastEditors
 * @Description: 模板模板参数
 */
#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>
#include <deque>
#include <stdexcept>

template <typename T,
          template <typename ELEM,
                    typename = std::allocator<ELEM>>
          class CONT = std::deque>
class Stack
{
private:
    CONT<T> elems; // elements

public:
    void push(T const &); // push element
    void pop();           // pop element
    T top() const;        // return top element
    bool empty() const
    { // return whether the stack is empty
        return elems.empty();
    }

    // assign stack of elements of type T2
    template <typename T2,
              template <typename ELEM2,
                        typename = std::allocator<ELEM2>>
              class CONT2>
    Stack<T, CONT> &operator=(Stack<T2, CONT2> const &);
};

template <typename T, template <typename, typename> class CONT>
void Stack<T, CONT>::push(T const &elem)
{
    elems.push_back(elem); // append copy of passed elem
}

template <typename T, template <typename, typename> class CONT>
void Stack<T, CONT>::pop()
{
    if (elems.empty()) {
        throw std::out_of_range("Stack<>::pop(): empty stack");
    }
    elems.pop_back(); // remove last element
}

template <typename T, template <typename, typename> class CONT>
T Stack<T, CONT>::top() const
{
    if (elems.empty()) {
        throw std::out_of_range("Stack<>::top(): empty stack");
    }
    return elems.back(); // return copy of last element
}

template <typename T, template <typename, typename> class CONT>
template <typename T2, template <typename, typename> class CONT2>
Stack<T, CONT>&
Stack<T, CONT>::operator=(Stack<T2, CONT2> const &op2)
{
    if ((void *)this == (void *)&op2) { // assignment to itself?
        return *this;
    }

    Stack<T2> tmp(op2); // create a copy of the assigned stack

    elems.clear(); // remove existing elements
    while (!tmp.empty()) { // copy all elements
        elems.push_front(tmp.top());
        tmp.pop();
    }
    return *this;
}

int main()
{
    try {
        Stack<int> intStack;     // stack of ints
        Stack<float> floatStack; // stack of floats

        // manipulate int stack
        intStack.push(42);
        intStack.push(7);

        // manipulate float stack
        floatStack.push(7.7);

        // assign stacks of different type
        floatStack = intStack;

        // print float stack
        std::cout << floatStack.top() << std::endl;
        floatStack.pop();
        std::cout << floatStack.top() << std::endl;
        floatStack.pop();
        std::cout << floatStack.top() << std::endl;
        floatStack.pop();
    }
    catch (std::exception const &ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }

    // stack for ints using a vector as an internal container
    Stack<int, std::vector> vStack;

    vStack.push(42);
    vStack.push(7);
    std::cout << vStack.top() << std::endl;
    vStack.pop();

    return 0;
}