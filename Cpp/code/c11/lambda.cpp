/*
 * @Author: JohnJeep
 * @Date: 2021-05-15 12:26:31
 * @LastEditTime: 2025-11-19 14:01:37
 * @LastEditors: JohnJeep
 * @Description: 深入理解 lambda
 */
#include <iostream>
#include <cstdlib>

using namespace std;


void lambda_captures()
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

void lambda_capture_mutable()
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
    lambda_captures(); 
    lambda_capture_mutable();

    return 0;
}