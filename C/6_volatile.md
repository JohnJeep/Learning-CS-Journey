<!--
 * @Author: JohnJeep
 * @Date: 2019-08-22 11:50:29
 * @LastEditTime: 2021-03-06 18:14:44
 * @LastEditors: Please set LastEditors
 * @Description: volatile 关键字
--> 

<!-- TOC -->

- [1. Volatile关键字](#1-volatile关键字)
  - [1.1. 编译器的优化介绍](#11-编译器的优化介绍)
  - [1.2. 概念](#12-概念)
  - [1.3. 为什么使用 volatile 关键字修饰的语句会影响编译器的优化？](#13-为什么使用-volatile-关键字修饰的语句会影响编译器的优化)
  - [1.4. 什么情况下使用 volatile？](#14-什么情况下使用-volatile)
  - [1.5. 注意点](#15-注意点)
  - [1.6. Reference](#16-reference)

<!-- /TOC -->

# 1. Volatile关键字

## 1.1. 编译器的优化介绍
> 高速缓存：处理器读取程序里面的数据时，把一些访问频率比较高的数据，临时存储到寄存器(register)中，当需要取数据时，就会从 register 中取，而不是直接去从 memory(内存)中取，节约了时间，像这样的过程，叫做高速缓存。

- 硬件级别的优化：内存访问速度远不及CPU处理速度，为提高机器整体性能，在硬件上引入硬件高速缓存(Cache)，加速对内存的访问。另外在现代CPU中指令的执行并不一定严格按照顺序执行，没有相关性的指令可以乱序执行，以充分利用CPU的指令流水线(Instruction pipeline)，提高执行速度。

- 软件级别的优化：一种是在编写代码时由程序员优化，另一种是由编译器进行优化。编译器优化常用的方法有：将内存变量缓存到寄存器；调整指令顺序充分利用CPU指令流水线，常见的是重新排序读写指令。对常规内存进行优化的时候，这些优化是透明的，而且效率很好。

- 由编译器优化或者硬件重新排序引起的问题的解决办法是在从硬件（或者其他处理器）的角度看必须以特定顺序执行的操作之间设置内存屏障（memory barrier），linux 提供了一个宏解决编译器的执行顺序问题。 
  > `void Barrier(void)` 这个函数通知编译器插入一个内存屏障，但对硬件无效，编译后的代码会把当前CPU寄存器中的所有修改过的数值存入内存，需要这些数据的时候再重新从内存中读出。


## 1.2. 概念
`volatile` 关键字(keywords)是一种类型修饰符(Type Qualifiers)，volatile 的英文翻译过来是 “易变的” 。用`volatile` 声明类型变量的时候，编译器对访问该变量的代码就不再进行优化，从而可以提供对特殊地址的稳定访问；如果不使用 `volatile` 进行声明，则编译器将对所声明的语句进行优化。即 `volatile` 关键字影响编译器编译的结果，用 `volatile` 声明的变量表示该变量随时可能发生变化，与该变量有关的运算，不要进行编译优化，以免出错。


## 1.3. 为什么使用 volatile 关键字修饰的语句会影响编译器的优化？
> 当使用 `volatile` 声明变量值的时候，编译器总是重新从它所在的原内存读取数据，即使它前面的指令刚刚从该处读取过数据。
  
1> 告诉compiler不能做任何优化
```
比如要往某一地址送两指令：
int *ip =...; //设备地址
*ip = 1; //第一个指令
*ip = 2; //第二个指令

以上程序被compiler可能做优化为：
int *ip = ...;
*ip = 2;

结果第一个指令丢失。如果用volatile, compiler就不允许做任何的优化，从而保证程序的原意：
volatile int *ip = ...;
*ip = 1;
*ip = 2;
即使你要compiler做优化，它也不会把两次付值语句间化为一，它只能做其它的优化。
```

2> 用 `volatile` 定义的变量会在程序外被改变，每次都必须从内存中读取，而不能重复使用放在cache(高速缓存)或寄存器中的备份。



## 1.4. 什么情况下使用 volatile？
一般说来，volatile用在如下的几个地方：

一、 中断服务程序中修改的供其它程序检测的变量需要加 `volatile`；
```c
static int i=0;

int main(void)
{
  ...
  while (1) {
    if (i) {
      dosomething();
    }
  }
｝

/* Interrupt service routine. */
void ISR_2(void)
{
      i=1;
}
```
程序的本意是希望ISR_2中断产生时，在main当中调用do_something函数，但是，由于编译器判断在main函数里面没有修改过i，因此可能只执行一次对从i到某寄存器的读操作，然后每次if判断都只使用这个寄存器里面的“i副本”，导致do_something永远也不会被调用。如果变量加上volatile修饰，则编译器保证对此变量的读写操作都不会被优化（肯定执行）。此例中i也应该如此说明。


二、多任务环境下`各任务间共享的标志` 应该加 `volatile`；
> 在编写多线程的程序时，同一个变量可能被多个线程修改，而程序通过该变量同步各个线程。

```c
DWORD __stdcall threadFunc(LPVOID signal) 
{ 
  int* intSignal=reinterpret_cast<int*>(signal); 
  *intSignal=2; 

  while(*intSignal!=1) 
      sleep(1000); 

  return 0; 
}
```

该线程启动时将intSignal 置为2，然后循环等待直到intSignal 为1 时退出。显然intSignal的值必须在外部被改变，否则该线程不会退出。但是实际运行的时候该线程却不会退出，即使在外部将它的值改为1，看一下对应的伪汇编代码就明白了： 
```
mov ax,signal 
  label: 
  if(ax!=1) 
  goto label
```

对于C编译器来说，它并不知道这个值会被其他线程修改。自然就把它cache在寄存器里面。记住，C 编译器是没有线程概念的！这时候就需要用到volatile。volatile 的本意是指：这个值可能会在当前线程外部被改变。也就是说，我们要在threadFunc中的intSignal前面加上volatile关键字，这时候，编译器知道该变量的值会在外部改变，因此每次访问该变量时会重新读取，所作的循环变为如下面伪码所示： 
```
label: 
 mov ax,signal 
 if(ax!=1) 
 goto label 
3、Memory 
```


三、 存储器映射的硬件寄存器通常也要加volatile说明，因为每次对它的读写都可能有不同意义；
```
XBYTE[2]=0x55;
XBYTE[2]=0x56;
XBYTE[2]=0x57;
XBYTE[2]=0x58;
```
> 对外部硬件而言，上述四条语句分别表示不同的操作，会产生四种不同的动作，但是编译器却会对上述四条语句进行优化，认为只有XBYTE[2]=0x58（即忽略前三条语句，只产生一条机器代码）。如果键入volatile，则编译器会逐一地进行编译并产生相应的机器代码（产生四条代码）。
  
  
<font color=red>
注意： 以上这几种情况经常还要同时考虑数据的完整性（相互关联的几个标志读了一半被打断了重写），在1中可以通过关中断来实现，2 中可以禁止任务调度，3中则只能依靠硬件的良好设计了。
</font>


## 1.5. 注意点
- 频繁地使用 `volatile` 很可能会增加代码尺寸和降低性能，因为它频繁的访问内存，而非缓存或寄存器，因此要合理的使用 `volatile`。
- 作为指令关键字，确保本条指令不会因编译器的优化而省略，且要求每次直接读值，volatile处理结果：直接存取原始内存地址的值。


## 1.6. Reference
- [C语言中volatile关键字的作用](https://blog.csdn.net/tigerjibo/article/details/7427366)
- [volatile关键字的使用](https://blog.csdn.net/vay0721/article/details/79035854)
- [C语言的volatile关键字](https://www.cnblogs.com/OpenCoder/p/7723825.html)

