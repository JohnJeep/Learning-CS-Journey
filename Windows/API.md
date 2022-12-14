## Win32 API 后缀

- `A` 表示使用ANSI编码作为标准输入与输出流的文本编码
- `W` 表示使用Unicode作为编码
- `Ex` 表示拓展, 标注了Ex的winapi函数会比没有标Ex的函数多一些参数什么的, 可以说拓展了一些功能
- `ExA` 与 ExW 就是 A,W与Ex的结合了



## _beginthreadex

函数原型

```cpp
unsigned long _beginthreadex(
    void *security,    // 安全属性， 为NULL时表示默认安全性
    unsigned stack_size,    // 线程的堆栈大小， 一般默认为0
    unsigned(_stdcall *start_address)(void *),    // 所要启动的线程函数
    void *argilist, // 线程函数的参数， 是一个void*类型， 传递多个参数时用结构体
    unsigned initflag, // 新线程的初始状态，0表示立即执行，CREATE_SUSPENDED表示创建之后挂起
    unsigned *threaddr    // 用来接收线程ID
);

返回值 : // 成功返回新线程句柄， 失败返回0

```





## 线程句柄作用

1. 线程和线程句柄（Handle）不同，线程是一个程序的工作流程，线程句柄是一个内核对象。线程的生命周期就是线程函数从开始执行到线程结束，线程句柄一旦CreateThread返回，如果你不用它操作线程或者等待线程等操作比如waitforsingleobject，就可以CloseHandle。
2. CloseHandle 只是关闭了一个线程句柄对象，表示我不再使用该句柄，即不对这个句柄对应的线程做任何干预了，和结束线程没有一点关系。若在线程执行完之后，没有调用CloseHandle，在进程执行期间，将会造成内核对象的泄露，相当于句柄泄露，但不同于内存泄露，这势必会对系统的效率带来一定程度上的负面影响。但当进程结束退出后，系统会自动清理这些资源。

3. 关闭一个内核对象。其中包括文件、文件映射、进程、线程、安全和同步对象等。在CreateThread成功之后会返回一个hThread的handle，且内核对象的计数加1，CloseHandle之后，引用计数减1，当变为0时，系统删除内核对象。

## WaitForSingleObject

函数原型

```c
DWORDWINAPIWaitForSingleObject(
  HANDLEhHandle,
  DWORDdwMilliseconds);
  
 功能：等待函数 – 使线程进入等待状态，直到指定的内核对象被触发。
```

第一个参数为要等待的内核对象。

第二个参数为最长等待的时间，以毫秒为单位，如传入5000就表示5秒，传入0就立即返回，传入INFINITE表示无限等待。

因为线程的句柄在线程运行时是未触发的，线程结束运行，句柄处于触发状态。所以可以用WaitForSingleObject()来等待一个线程结束运行。

函数返回值：在指定的时间内对象被触发，函数返回WAIT_OBJECT_0。超过最长等待时间对象仍未被触发返回WAIT_TIMEOUT。传入参数有错误将返回WAIT_FAILED

## WaitForMultipleObjects

函数原型

```
WaitForMultipleObjects(
    _In_ DWORD nCount,    		// 要监测的句柄的组的句柄的个数
    _In_reads_(nCount) CONST HANDLE* lpHandles,   //要监测的句柄的组
    _In_ BOOL bWaitAll,  		// TRUE 等待所有的内核对象发出信号， FALSE 任意一个内核对象发出信号
    _In_ DWORD dwMilliseconds 	//等待时间
);

功能： 阻塞多个线程句柄，直到子线程运行完毕，主线程才会往下走
```

- 参数一：检测句柄的个数；
- 参数二：检测句柄的数组；
- 参数三：TRUE等待所有线程执行完毕，FALSE，任意一个完成就停止阻塞；
- 参数四：等待时间





## 其它

`CreateThread`是由操作系统提供的接口，而`AfxBeginThread`和`_BeginThread`则是编译器对它的封装。