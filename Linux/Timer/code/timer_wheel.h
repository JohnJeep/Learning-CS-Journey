/**
 * @author your name (you@domain.com)
 * @brief  时间轮的方式实现一个高效率定时器
 * @version 0.1
 * @date 2023-03-16
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef TIMER_WHRRL_H
#define TIMER_WHRRL_H

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define BUFFER_SIZE 64
class tw_timer;

// 绑定 socket 和定时器
struct client_data {
    sockaddr_in address;
    int sockfd;
    char buf[BUFFER_SIZE];
    tw_timer* timer;
};

class tw_timer {
public:
    tw_timer(int ts, int rot)
        : time_slot(ts)
        , rotation(rot)
        , prev(nullptr)
        , next(nullptr)
    {
    }
    ~tw_timer() { }

public:
    int time_slot; // 记录定时器属于时间轮上的哪一个槽
    int rotation; // 记录定时器在时间轮上转多少圈后生效
    tw_timer* prev; // 指向前一个定时器
    tw_timer* next; // 指向下一个定时器
    client_data* user_data; // 客户数据
    void (*cb_func)(client_data*); // 定时器回调函数
};

class TimerWheel {
public:
    TimerWheel()
        : cur_slot(0)
    {
        for (int i = 0; i < N; i++) {
            slots[i] = nullptr;  // 初始化每个槽的头结点
        }
    }

    // 遍历每个槽，并销毁其中的定时器
    ~TimerWheel()
    {
        for (int i = 0; i < N; i++) {
            tw_timer* tmp = slots[i];
            while (tmp) {
                slots[i] = tmp->next;
                delete tmp;
                tmp = slots[i];
            }
        }
    }

    // 复杂度：O(1)
    // 根据定时器的值 timeout 创建一个定时器，并把它插入到合适的槽中
    tw_timer* add_timer(int timeout)
    {
        if (timeout < 0) {
            return nullptr;
        }

        // 根据超时值计算它在时间轮上转动多少个滴答后触发，并将该滴答值存在 ticks 中
        // 若定时器的超时值小于时间轮的槽间隔时间 SI，则将 ticks 向上折合为 SI，
        // 否则就将向下折合为 ticks=timeout/SI
        int ticks = 0;
        if (ticks < SI) {
            ticks = 1;
        } else {
            ticks = timeout / SI;
        }

        int rotation = ticks / N;   // 计算待插入定时器的超时值在时间轮上转动多少圈后触发
        int timerSlot = (cur_slot + (ticks / N)) % N;  // 计算待插入定时器的超时值应该被插入到哪个槽中

        // 创建一个定时器，在时间轮上转动 rotation 圈后触发，且位于第 timeSlot 个槽上
        tw_timer* timer = new tw_timer(timerSlot, rotation);

        // 若第 timeSlot 个槽中没有定时器，则将新创建的定时器插入其中，并将该定时器设置为该槽的头节点 
        if (!slots[timerSlot]) {
            printf("Add timer, rotation is %d, timeSlot is %d, curr_slot is %d\n", rotation, timerSlot, cur_slot);
            slots[timerSlot] = timer;
        } else {
            // 否则，将定时器插入第 timerSlot 个槽中
            timer->next = slots[timerSlot];
            slots[timerSlot]->prev = timer;
            slots[timerSlot] = timer;
        }

        return timer;
    }

    // 复杂度：O(1)
    // 删除指定的定时器 timer
    void del_timer(tw_timer* timer)
    {
        if (!timer) {
            return;
        }

        int ts = timer->time_slot;

        // slot[ts] 是目标定时器所在槽的头节点，若目标定时器就是该头节点，则需要重置第 ts 个槽的头节点 
        if (timer = slots[ts]) {
            slots[ts] = slots[ts]->next;
            if (slots[ts]) {
                slots[ts]->prev = nullptr;
            }
            delete timer;
        } else {
            timer->prev->next = timer->next;
            if (timer->next) {
                timer->next->prev = timer->prev;
            }
            delete timer;
        }
    }

    // 复杂度：O(n)
    // 槽间隔时间 slot interval(SI) 到了后，调用该函数，时间轮向前滚动一个槽的间隔
    void tick()
    {
        tw_timer* tmp = slots[cur_slot];  // 取得时间轮上当前槽的头节点
        printf("Current slot is %d\n", cur_slot);
        while (tmp) {
            // 若定时器的 rotation 的值大于 0，则它不在这一轮起作用
            if (tmp->rotation > 0) {
                tmp->rotation--;
                tmp = tmp->next;
            } else {
                // 定时器已到定时时间，执行定时任务，最后删除定时器
                tmp->cb_func(tmp->user_data);

                if (tmp = slots[cur_slot]) {
                    slots[cur_slot] = tmp->next;
                    delete tmp;

                    if (slots[cur_slot]) {
                        slots[cur_slot]->prev = nullptr;
                    }
                    tmp = slots[cur_slot];
                } else {
                    tmp->prev->next = tmp->next;
                    if (tmp->next) {
                        tmp->next->prev = tmp->next;
                    }

                    tw_timer* tmp_other = tmp->next;
                    delete tmp;
                    tmp = tmp_other;
                }
            }
        }
        // 更新时间轮的当前槽，以反映时间轮的转动
        cur_slot = ++cur_slot % N;
    }

private:
    static const int N = 60; // 时间轮上槽的数目
    static const int SI = 1; // 每 1s 时间轮转一次，即槽间隔为 1s
    tw_timer* slots[N]; // 时间轮的槽，每个元素指向一个定时器的链表
    int cur_slot; // 时间轮的当前槽
};

#endif