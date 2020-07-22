/*
 * @Author: your name
 * @Date: 2020-07-22 09:16:43
 * @LastEditTime: 2020-07-22 16:19:28
 * @LastEditors: Please set LastEditors
 * @Description: STL标准库综合案例：24名选手参加演讲比赛，选出比赛的前三名
 * @FilePath: /42_stl_comprehensive_example.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <algorithm>
#include <string>
#include <vector>
#include <set>
#include <functional>
#include <map>
#include <deque>
#include <numeric>
#include <string.h>

using namespace std;

class Speechmaker
{
private:
    int m_score[3];  // 取得前三名的分数
    string m_name;
public:
    Speechmaker(/* args */);
    ~Speechmaker();
    int getScore(int index) const {return m_score[index];} 
    void setScore(int score, int index) { this->m_score[index] = score;} 
    string getName() const {return m_name;}
    void setName(string name) {this->m_name = name;}
};

Speechmaker::Speechmaker(/* args */)
{
}

Speechmaker::~Speechmaker()
{
}

/**
 * @description: 筛选比赛选手
 * @param {type} 参赛选手；比赛名单
 * @return: 
 */
void siftSpeaker(map<int, Speechmaker>& speaker, vector<int>& v)
{
    string str = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    int len = 24;
    string cstr = "选手";

    // 将选手姓名插入map容器中
    random_shuffle(str.begin(), str.end());
    for (int i = 0 ; i < len; i++)
    {
        Speechmaker tmp;
        string nstr = str[i] + cstr; 
        tmp.setName(nstr);
        speaker.insert(pair<int, Speechmaker>(100 + i, tmp));
    }

    // 将选手编号插入到vector容器中
    for (int i = 0; i < len; i++)
    {
        v.push_back(100 + i);
    }
}

/**
 * @description: 演讲比赛抽签
 * @param {type} 抽签名单
 * @return: 
 */
void speechContestDraw(vector<int>& entry)
{
    random_shuffle(entry.begin(), entry.end());
}

/**
 * @description:  选手进行演讲比赛             
 * @param {type}  index: 第几轮比赛
 * @param {type}  before: 晋级前的参赛选手名单
 * @param {type}  speaker: 参赛选手
 * @param {type}  after: 晋级后的参赛选手名单
 * @return: 
 */
void speechContest(int index, vector<int>& before, map<int, Speechmaker>& speaker, vector<int>& after)
{
    multimap<int, int, greater<int>> multiGroup;  // 小组成绩：记录每组比赛的得分，求出前三名和后三名，从大到小的顺序
    int tmpCount = 0;

    // 10 个评委进行打分  
    for (vector<int>::iterator it = before.begin(); it != before.end(); it++)
    {
        deque<int> gainScore;
        
        tmpCount++;
        for (int j = 0; j < 10; j++)
        {
            int score = 50 + rand() % 50;
            gainScore.push_back(score);
        }
        sort(gainScore.begin(), gainScore.end());

        //去掉最低分和最高分
        gainScore.pop_back();
        gainScore.pop_front();

        // 求平均值
        int scoreSum = accumulate(gainScore.begin(), gainScore.end(), 0);
        int scoreAvg = scoreSum / gainScore.size();
        // speaker[*it].m_score[index] = scoreAvg;              // 选手的平均分存到类对象的数组中
        speaker[*it].setScore(scoreAvg, index);              // 选手的平均分存到类对象的数组中
        multiGroup.insert(pair<int, int>(*it, scoreAvg));    // 将选手得到的分数插入到容器中

        // 处理分组：取每组的前6名存到一个容器中   
        if (tmpCount % 6 == 0)
        {
            cout << "第" << index + 1 << "轮小组比赛成绩" << endl;
            for (multimap<int, int , greater<int>>::iterator mt = multiGroup.begin(); mt != multiGroup.end(); mt++)
            {
                // 编号 姓名 得分 
                cout << mt->first << "\t" << speaker[mt->first].getName() << "\t" << mt->second << endl;
            }

            // 前三名晋级
            while (multiGroup.size() > 3)
            {
                multimap<int, int, greater<int>>::iterator jt = multiGroup.begin();
                after.push_back(jt->first);    // 将晋级的选手编号放到 res 名单中
                multiGroup.erase(jt);
            }
            multiGroup.clear();     // 清空本小组比赛的成绩
        }
    }
}

/**
 * @description: 查看比赛的结果
 * @param {type} 
 * @return: 
 */
void speechContestResult(int index, vector<int>& after, map<int, Speechmaker>& speaker)
{   
    cout << endl;
    cout << "第" << index + 1 << "轮晋级选手名单" << endl;
    for (vector<int>::iterator it = after.begin(); it != after.end(); it++)
    {
        cout << "参赛编号: " << *it << "\t" << speaker[*it].getName() << "\t" 
             << speaker[*it].getScore(index) << endl;
    }
    cout << endl;
}

int main(int argc, char *argv[])
{
    // 将参加比赛选手的信息和比赛信息的数据保存到容器中
    map<int, Speechmaker> mapSpeaker;       // 存储参赛选手的编号和名字
    vector<int> v1;                         // 第一轮比赛的名单
    vector<int> v2;                         // 第二轮比赛的名单
    vector<int> v3;                         // 第三轮比赛的名单
    vector<int> v4;                         // 最后一轮比赛名单

    siftSpeaker(mapSpeaker, v1);            // 打乱选手编号的顺序，得到参赛的名单
    // 第一轮
    speechContestDraw(v1);                  // 选手抽签
    speechContest(0, v1, mapSpeaker, v2);   // 选手进行比赛
    speechContestResult(0, v2, mapSpeaker); // 查看结果

    // 第二轮
    speechContestDraw(v2);                  
    speechContest(1, v2, mapSpeaker, v3);   
    speechContestResult(1, v3, mapSpeaker);    

    // 第三轮
    speechContestDraw(v3);                  
    speechContest(2, v3, mapSpeaker, v4);   
    speechContestResult(2, v4, mapSpeaker);    

    return 0;
}
