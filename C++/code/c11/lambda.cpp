/*
 * @Author: JohnJeep
 * @Date: 2021-05-15 12:26:31
 * @LastEditTime: 2021-05-15 18:21:56
 * @LastEditors: Please set LastEditors
 * @Description: 深入理解 lambda
 */
#include <iostream>
#include <cstdlib>

using namespace std;


void test01_lambda_captures()
{
    int x = 100;
    int y = 200;

    auto val = [x, &y](){
        cout << "x: " << x << endl;
        cout << "y: " << y << endl;
        ++y;
    };
    x = y = 300;
    val();
    val();
    cout << "final y: " << y << endl;
}

void test02_lambda_capture_mutable()
{
    cout << "Test lambda mutable" << endl;
    int id = 10;
    auto f = [id]() mutable {
        cout << "id: " << id << endl;
        ++id;
    };
    id = 42;
    f();
    f();
    f();
    cout << id << endl;
}

int main(int argc, char *argv[])
{
    test01_lambda_captures(); 
    test02_lambda_capture_mutable();

    return 0;
}