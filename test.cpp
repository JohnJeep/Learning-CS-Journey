/*
 * @Author: your name
 * @Date: 2021-01-17 11:25:25
 * @LastEditTime: 2021-01-17 12:06:00
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Learning-Computer-Journey\test.cpp
 */
#include <iostream>
#include <stdlib.h>
#include <iterator>

using namespace std;

template <class Arg, class Result>
  struct unary_fun 
  {
    typedef Arg        arg_type;
    typedef Result    result_type;
  };
  

int main(int argc, char *argv[])
{
    // unary_fun unf;
    cout << sizeof(unary_fun<int , int >)  << endl;

    return 0;
}
