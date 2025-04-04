#include <iostream>
#include <map>
#include <string>
using namespace std;

int main()
{
    /// 1. 初始化
    multimap<int, string> mapStudent;
    multimap<int, string>::iterator iter, beg, end;

    /// 2. 添加元素
    /// multimap不支持下标操作
    mapStudent.insert(pair<int, string>(0, "student_one"));
    mapStudent.insert(pair<int, string>(0, "student_one_copy")); ///一对多
    mapStudent.insert(pair<int, string>(1, "student_two"));
    mapStudent.insert(pair<int, string>(5, "Fear Kubrick"));
    mapStudent.insert(pair<int, string>(2, "Akemi Homura"));
    mapStudent.insert(pair<int, string>(-1, "Eren Jaeger"));
    mapStudent.insert(pair<int, string>(99, "lin"));
    cout << mapStudent.size() << endl;
    cout << endl;

    /// 3. 遍历
    for (iter = mapStudent.begin(); iter != mapStudent.end(); iter++)
        cout << iter->first << " " << iter->second << endl;
    cout << endl;

    /// 4. 单键查询与范围查询
    ///单键查询
    int count = mapStudent.count(0);
    iter = mapStudent.find(0);
    for (int i = 0; i < count; i++, iter++) {
        cout << iter->first << " " << iter->second << endl;
    }
    cout << endl;

    ///范围查询
    beg = mapStudent.lower_bound(1); /// >=1
    end = mapStudent.upper_bound(5); /// <=5
    for (; beg != end; beg++)
        cout << beg->first << " " << beg->second << endl;
    cout << endl;

    /// 5. 删除
    // for (iter = mapStudent.begin(); iter != mapStudent.end();) {
    for (int k = 0; k < count; k++) {
        iter = mapStudent.find(0);
        mapStudent.erase(iter++);
    }
    // }

    cout << mapStudent.size() << endl;
    for (iter = mapStudent.begin(); iter != mapStudent.end(); iter++)
        cout << iter->first << " " << iter->second << endl;
    cout << endl;

    /// 6. 判空与清空
    if (!mapStudent.empty()) {
        mapStudent.clear();
    }

    return 0;
}