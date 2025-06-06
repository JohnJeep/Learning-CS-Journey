<!--
 * @Author: JohnJeep
 * @Date: 2021-04-05 16:08:47
 * @LastEditTime: 2025-04-04 19:30:49
 * @LastEditors: JohnJeep
 * @Description: In User Settings Edit
-->

# 1. 什么是回调函数？

- 将函数指针作为一个参数传递给另一个函数。
- 把函数的指针（地址）作为另一个函数参数的入口地址传递，当这个指针被用来调用其所指向的函数时，我们就说这是回调函数。


# 2. 分类

根据回调函数在运行时控制数据流的方式不同，可以为两种类型的回调函数。
- 阻塞回调(Blocking callbacks)，也叫同步回调(synchronous callbacks)或者直接叫回调函数(callbacks)。阻塞回调函数调用是发生在函数返回之前。阻塞回调通常不用于异步或将当前的工作委托给另外一个线程的情况。

- 延迟回调(Deferred callbacks)，也叫异步回调(asynchronous callbacks)。延时回调函数调用是发生在函数返回之后。延时回调函数常常使用在I/O操作或事件处理(event handing)中,，通过中断或多线程来调用。

Windows系统中使用回调函数，应用程序提供一个特定的自定义回调函数给操作系统调用，然后这个应用程序需要实现特定的函数功能，像鼠标点击或键盘按下。

windows系统中外部应用程序与系统库函数之间通过 `callback` 函数实现的流程。
![](./figures/Callback-notitle.svg)

在这个调用的过程中最关心的是如何进行权限(privilege)与安全(security)管理，当这个函数被操作系统调用时，它不应该带有同操作系统一样的权限。解决的方案是使用 [rings](https://en.wikipedia.org/wiki/Protection_ring) 保护。

> rings: 操作系统如何实现特权管理的一种手段。


# 3. 用途

- 错误信号处理。当Unix系统收到 `SIGTERM` 信号时，不想让Terminal立刻收到它，为了确保terminal处理的是正确的，它会注册一个 `cleanup` 函数作为回调函数。回调函数也许被使用来控制 `cleanup` 函数是否被执行。
- C++中 `functor` 普遍的使用就是 `function pointer`。
- 进程与线程之间进行通信。


# 4. 为什么要使用回调函数？

- 把调用者与被调用者分开进行实现。
- 开发者在库中封装了一套采用函数指针实现的接口，接口函数的参数就是用户实现函数的地址。
- 将回调函数（callback）传递给调用函数（calling function）的代码不需要知道将传递给调用函数的参数值有哪些。如果传递的是返回值，则需要公开暴露这些参数。


# 5. 代码实现

实例一：
```c
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* The calling function takes a single callback as a parameter. */
void PrintTwoNumbers(int (*numberSource)(void)) 
{
    int val1 = numberSource();
    int val2 = numberSource();
    printf("%d and %d\n", val1, val2);
}

/* A possible callback */
int overNineThousand(void) 
{
    return (rand()%1000) + 9001;
}

/* Another possible callback. */
int meaningOfLife(void) 
{
    return 42;
}

/* Here we call PrintTwoNumbers() with three different callbacks. */
int main(void) 
{
    time_t t;
    srand((unsigned)time(&t)); // Init seed for random function
    PrintTwoNumbers(&rand);
    PrintTwoNumbers(&overNineThousand);
    PrintTwoNumbers(&meaningOfLife);

    return 0;
}
```

实例二：
```c
#include <stdio.h>
#include <string.h>

typedef struct _MyMsg {
    int id;
    char name[32];
} MyMsg;

void myfunc(MyMsg *msg)
{
    if (strlen(msg->name) > 0 ) {
        printf("id = %d \name = %s \n",msg->id, msg->name);
    }
    else {
        printf("id = %d \name = No name\n",msg->id);
    }
}

/*
 * Prototype declaration
 */
void (*callback)(MyMsg*);

int main(void)
{
    MyMsg msg1;
    msg1.id = 100;
    strcpy(msg1.name, "This is a test\n");

    /*
     * Assign the address of the function "myfunc" to the function
     * pointer "callback" (may be also written as "callback = &myfunc;")
     */
    callback = myfunc;

    /*
     * Call the function (may be also written as "(*callback)(&msg1);")
     */
    callback(&msg1);

    return 0;
}
```



# 6. references

- [英文Wikipedia解释什么是Callback](https://en.wikipedia.org/wiki/Callback_(computer_programming))
- [What is a callback function?](https://stackoverflow.com/questions/824234/what-is-a-callback-function/7549753#7549753)
- [函数指针和指针函数用法和区别](https://blog.csdn.net/luoyayun361/article/details/80428882)
- [C语言中函数指针和回调函数的详解](https://blog.csdn.net/weixin_39939425/article/details/90298435)
- [深入浅出剖析C语言函数指针与回调函数(一)](https://blog.csdn.net/morixinguan/article/details/65494239?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task)