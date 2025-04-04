#include <algorithm>
#include <iostream>

using namespace std;

int main(int argc, char* argv[])
{
    std::vector<int> vec = { 1, 3, 5, 7 };

    // for_each 与lambda表达式结合使用
    int sum = 0;
    for_each(vec.begin(), vec.end(), [&sum](int x) mutable -> int {
        sum += x;
    });
    cout << sum << endl;

    return 0;
}