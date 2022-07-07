#include <iostream>
#include <queue>

using namespace std;

int main(int argc, char *argv[])
{
    std::priority_queue<int> priq;

    priq.push(20);
    priq.push(10);
    priq.push(50);
    priq.push(30);
    priq.push(80);
    priq.push(5);

    // 遍历打印优先级队列
    while (!priq.empty()) {
        cout << priq.top() << endl;
        priq.pop();
    }
    cout << "size:" << priq.size() << endl;

    return 0;
}