<!--
 * @Author: JohnJeep
 * @Date: 2020-05-21 19:19:20
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-25 21:40:13
 * @Description: 预处理、编译、汇编、链接过程
 * Copyright (c) 2022 by JohnJeep, All Rights Reserved. 
-->

- [1. 缩写](#1-缩写)
- [2. 编译链接](#2-编译链接)
  - [2.1. 预处理(Preprocessing)](#21-预处理preprocessing)
  - [2.2. 编译(Compilation)](#22-编译compilation)
  - [2.3. 汇编(Assembly)](#23-汇编assembly)
  - [2.4. 链接(Linking)](#24-链接linking)
  - [2.5. 常见的工具链组合](#25-常见的工具链组合)
  - [2.6. 可执行文件(Execute file)](#26-可执行文件execute-file)
- [3. 内存四区](#3-内存四区)
- [4. 函数库](#4-函数库)
  - [4.1. 静态库](#41-静态库)
  - [4.2. 动态库](#42-动态库)
    - [4.2.1. Linux平台](#421-linux平台)
    - [4.2.2. Windows平台](#422-windows平台)
- [5. ELF relocatable](#5-elf-relocatable)
- [6. Tools](#6-tools)
  - [6.1. readelf](#61-readelf)
  - [6.2. size](#62-size)
  - [6.3. nm](#63-nm)
  - [6.4. pmap](#64-pmap)
  - [6.5. patchelf](#65-patchelf)
  - [6.6. objdump](#66-objdump)
- [7. Build](#7-build)
- [8. References](#8-references)

# 1. 缩写

- EXE(Executable)：可执行文件
- PE(Portable Executable)：可移植可执行。
- ELF(Executable Linkable Format)：可执行可链接格式。
- DLL(Dynamic ALinking Library): windows下的以 `.dll` 方式命名，Linux下的以 `.so` 方式命名。
- SLL(Static ALinking Library): windows下的以 `.lib` 方式命名，Linux的以 `.a` 方式命名。
- BSS(Block Started by Symbol): 未初始化的全局变量和局部静态变量的区域。

# 2. 编译链接

程序处理的流程：源代码→预处理→编译→汇编→目标文件→链接→可执行文件

## 2.1. 预处理(Preprocessing)

处理源代码中的预处理指令，例如`#include`、`#define`、`#ifdef`等。预处理器（cpp，C Preprocessor）会展开头文件，宏替换，条件编译等，生成一个纯C++代码文件（通常以`.i`或`.ii`为扩展名）。

```bash
# 只进行预处理
g++ -E main.cpp -o main.i
```

预处理阶段处理：

- `#include` 头文件包含
- `#define` 宏展开
- 条件编译 (`#if`, `#ifdef` 等)
- 删除注释

## 2.2. 编译(Compilation)

将预处理后的C++代码转换为汇编代码。编译器（如g++）进行词法分析、语法分析、语义分析、优化等步骤，生成对应平台的汇编代码文件（通常以`.s`为扩展名）。

```bash
# 编译为汇编代码
g++ -S main.i -o main.s
```

编译阶段：

- 词法分析、语法分析、语义分析
- 生成中间代码
- 代码优化
- 生成平台相关的汇编代码

## 2.3. 汇编(Assembly)

将汇编代码转换为机器代码。汇编器（as）将汇编代码翻译成目标机器的二进制代码，生成目标文件（通常以`.o`为扩展名）。目标文件包含了二进制代码和数据，但还没有解决外部符号的地址。

目标文件(Object file)：编译器编译源码后生成的文件叫做目标文件。

```bash
# 汇编为目标文件
g++ -c main.s -o main.o
```

汇编阶段：

- 将汇编代码转换为机器代码
- 生成可重定位的目标文件 (.o 文件)

**目标文件格式**

- **ELF** (Executable and Linkable Format)：Linux/Unix
- **PE/COFF** (Portable Executable)：Windows
- **Mach-O**：macOS

## 2.4. 链接(Linking)

将一个或多个目标文件(`.o` 或 `obi`)以及库文件(`.lib` 或 `.a`)组合在一起，生成最终的可执行文件(`.out`  output file)。

```bash
# 链接为可执行文件
g++ main.o utils.o -o program
```

链接阶段：

- 符号解析
- 地址重定位
- 合并多个目标文件
- 解析库依赖

**特点**

- 可执行文件可以被加载到内存并执行。
- 链接可以执行于编译时、加载时、运行时。
- Windows中启动程序由 CRT、DLL 提供，Linux中由 glibc(libs-start.c) 提供。
- 静态连接(static linking): 将外部函数库拷贝到可执行文件。
- 动态链接(dynamic linking)：外部函数库不进入安装包，只在运行时动态引用。
- 链接器（ld: linker directive）主要完成两个任务：符号解析和重定位。
  1. 符号解析(symbol resolution)：确保所有符号（函数、变量等）都有定义。
  2. 重定位：调整符号的地址，使得它们指向正确的内存位置。

例如执行流程：hello.cpp->>hello.ii(预处理)->>hello.s(汇编)->>hello.o(目标文件)->>hello.exe(可执行)

<div align="center">
    <img width="90%" height="90%" src="./figures/gcc_build_step.png">
</div>
## 2.5. 常见的工具链组合

| 编译器    | 链接器   | 平台    | 特点          |
| :-------- | :------- | :------ | :------------ |
| GCC (g++) | GNU ld   | Linux   | 传统组合      |
| Clang     | lld      | 跨平台  | LLVM生态      |
| MSVC      | link.exe | Windows | Visual Studio |
| ICC       | XILK     | Linux   | Intel工具链   |





## 2.6. 可执行文件(Execute file)

加载器
- 运行程序时，加载器首先加载程序到内存中，<font color=red> 被加载程序称为进程 </font>，并由操作系统加载。
- 主要作用：
  - 验证
  - 从硬盘复制可执行文件到主存中
  - 配置栈
  - 配置寄存器
  - 跳转到程序入口点(_start)

加载器加载可执行文件时，操作系统使用的步骤

1. 加载器请求操作系统创建一个新进程
2. 操作系统为新进程建立页表
3. 用无效入口标记页表
4. 开始执行程序，生成即时页面错误异常

<div align="center">
    <img width="80%" height="40%" src="./figures/加载过程.png">
</div>


# 3. 内存四区

首先了解基本的几个概念。

- 全局变量（外部变量）：出现在代码块 `{}` 之外的变量就是全局变量。
- 局部变量（自动变量）：一般情况下，代码块 `{}` 内部定义的变量就是自动变量，也可使用 `auto` 显示定义。
- static 变量：是指内存位置在程序执行期间一直不改变的变量，用关键字 `static` 修饰。代码块内部的静态变量只能被这个代码块内部访问，代码块外部的静态变量只能被定义这个变量的文件访问。

源代码编译后在内存中被分成五个主要部分：代码区、数据区、BSS、栈、堆。

- 代码区(code segment/text segment)
  
  - 用来存放程序执行代码的一块内存区域，里面装的是 CPU 执行的机器指令。
  - 程序加载到内存的时候由操作系统分配，程序结束时由操作系统释放。这部分区域的大小在程序运行前就已经确定，这块内存在程序运行期间是不变的，通常属于只能读，不能写，因为防止程序被指令意外的修改。
  - 代码区是共享的，共享的目的是为了频繁执行的程序，在内存中仅有一份代码的拷贝。
  - 函数也是代码的一部分，故函数都被放在代码区，包括main函数。 

- 数据段(data segment)
  
  - 用来存放程序中已初始化的 global 变量和static 变量的一块内存区域。
  - 分配优先于main函数，生存期与程序共存亡。
  - 程序一开始就分配了，直到结束才释放。
  - 可读可写。

- 栈(stack)
  
  - 栈(stack)是一种先进后出的内存结构，存储局部变量、函数形参和返回地址。
  - 栈是从高地址向低地址方向增长。
  - 在C语言中，函数参数的入栈顺序是从右到左。
  - C语言中形参和实参之间是值传递。
  - 每个线程都有自己专属的栈。
  - 栈的最大尺寸固定，超出则引起栈溢出。
  - 变量离开作用域后，栈上的内存会由系统自动释放，释放速度很快。
  - 栈分配与释放消耗CPU资源，只能存储少量的数据。

- 堆(heap)
  
  - 堆区由操作系统分配给每个进程，动态内存从堆中获取，调用 malloc()、calloc()、realloc()函数分配动态内存。
  - 堆区分配的内存仅能通过指针访问。 
  - 调用 free()函数将内存还给堆。
  - 堆内存用完不释放，可能会导致内存泄漏。
  - `malloc` 和 `free` 函数底层函数是采用全局指针实现的。

- BSS段
  
  - BSS(Block Started by Symbol): 以符号开始的块，用来存放程序中未初始化的全局变量和局部静态变量的一块区域。
  - 包括所有未初始化的全局变量、用static关键字声明且未初始化的静态全局变量
  - 运行所需空间记录在目标文件中，不在目标文件中占用实际空间
  - 程序启动过程中需要初始化的任意变量都可以存放在BSS段中。

什么时候用堆和栈？

- 如果明确知道数据占用多少内存，那么数据量较小时用栈，较大时用堆 
- 如果不知道数据量大小(可能需要占用较大内存)，最好用堆(因为这样保险些)
- 如果需要动态创建数组，则用堆

Linux 下内存分配管理如下图所示

<div align="center">
  <img width="90%" height="90%" src="./figures/virtual_address_space.png">
</div>

--------------------------

常见的内存错误

- 内存分配未成功，却使用了它。编程新手常犯这种错误，因为他们没有意识到内存分配会不成功。常用解决办法是，在使用内存之前检查指针是否为NULL。
- 内存分配虽然成功，但是尚未初始化就引用它。犯这种错误主要有两个起因：一是没有初始化的观念；二是误以为内存的缺省初值全为零，导致引用初值错误（例如数组）。 内存的缺省初值究竟是什么并没有统一的标准，尽管有些时候为零值，我们宁可信其无不可信其有。所以无论用何种方式创建数组，都别忘了赋初值，即便是赋零值也不可省略，不要嫌麻烦。
- 内存分配成功并且已经初始化，但操作越过了内存的边界。
- 忘记了释放内存，造成内存泄露。
- 释放了内存却继续使用它。
  - 程序中的对象调用关系过于复杂，实在难以搞清楚某个对象究竟是否已经释放了内存，此时应该重新设计数据结构，从根本上解决对象管理的混乱局面。
  - 函数的return语句写错了，注意不要返回指向“栈内存”的“指针”或者“引用”，因为该内存在函数体结束时被自动销毁。
  - 使用free或delete释放了内存后，没有将指针设置为NULL。导致产生“野指针”。

# 4. 函数库

函数库是可以被调用来执行的一段功能函数。函数库分为静态库和动态库。

Linux 内核提供的库函数大多数放在 `/usr/include`、`/usr/lib`、`/usr/lib64` 里面。

## 4.1. 静态库

所有编译器都提供一种机制，将所有相关的目标模块打包成一个单独的文件，成为静态库(static library)，它可作为链接器的输入。Linux 系统下以 `.a` 后缀，而 Windows 下以 `.lib` 后缀。

命名规则

- Linux中以 `.a` 结尾。形如：`lib + 库的名字 + .a` 
- `libtest.a` 静态库为 test

制作步骤

- 由`.c` 文件生成 `.o` 文件。   例如：`gcc *.c -Wall -I ./include/`
- 将 `.o` 文件打包。使用 `ar` 命令，参数为 `rcs`。基本格式为  `ar rcs 静态库的名字(libtest.a) 所有的.o文件 `
  
  - `ar rcs libstatic_1.a *.o` 
- 另外一种写法：例子：`gcc main.c -I ./include -L lib -l mylib -o main.out`
  
  <div align="center"><img width="50%" height="50%" src="./figures/static_library.png"></div>

优缺点

- 优点：加载速度快。发布程序的时候不需要对应的库(include)文件.
- 缺点：打包的程序占用很大的空间。程序发生改变时，需要重新编译静态库。

## 4.2. 动态库

动态库也叫共享库(share library)，它是一个目标模块，在运行或加载时，能加载到任意的内存地址，并链接一个内存中的程序，这个过程就叫动态链接(dynamic linking)，它是由一个叫动态链接器(dynamic linker)的程序来执行的。Linux 系统下以 `.so` 后缀，而 Windows 下以 `.dll` 后缀。

> 只有在程序执行的过程中才会加载动态链接库。

思考？

- 既然动态库是在运行时加载，那为什么在编译时还需要指明？
  
  因为在编译的时候，编译器需要知道每一个动态库中提供了哪些符号。

### 4.2.1. Linux平台

- 命名规则
  - Linux中以 `.so` 结尾。形如：`lib + 库的名字 + .so`
  - 例如：`libtest.so`
- 制作步骤
  - 生成与位置无关的 `.o` 文件。使用的命令 `gcc -fPIC -c *.c -I ./include`  将当前目录下所有的 `.c` 文件都生成 `.o` 文件，其中包括的头文件在 `./include` 路径下。
  - 将 `.o` 文件打包。 `gcc -shared -o libmytest.so *o -I ./include`  将当前目录下所有的 `.o` 文件打包为共享库 `libmytest.so`

- 共享库的调用
  - `gcc main.c ./lib/libmytest.so -o main.out -I ./include`  源文件 ` main.c` 与`./lib/libmytest.so` 目录下的动态库文件链接生成可执行的 `main.out` 文件
  - `gcc main.c -L ./lib -l mytest -o main.out -I ./include`    这种方式实现，需要在系统中配置动态链接库的环境变量。

- `ldd` 查看可执行文件 (.out) 在动态执行过程中所依赖的所有动态库。
  
  在 `ldd` 执行的结果中，`=>` 左边的表示该程序需要连接共享库的名称，右边表示由 Linux 的共享库系统找到对应的共享库，在文件系统中的具体位置。
  
  ```sh
  [root@redis_181 lib64]# ldd libselinux.so.1
          linux-vdso.so.1 =>  (0x00007ffe5ec63000)
          libpcre.so.1 => /lib64/libpcre.so.1 (0x00007faf6fbb0000)
          libdl.so.2 => /lib64/libdl.so.2 (0x00007faf6f9ac000)
          libc.so.6 => /lib64/libc.so.6 (0x00007faf6f5de000)
          /lib64/ld-linux-x86-64.so.2 (0x00007faf70039000)
          libpthread.so.0 => /lib64/libpthread.so.0 (0x00007faf6f3c2000)
  ```

程序加载动态库是从系统的环境变量中去查找的。

① 开发过程中临时使用的一种方法，不是永久生效。每次关闭终端，都会将配置的环境变量清除。

- 系统提供的动态链接库环境变量 `LD_LIBRARY_PATH`
- 将自己制作的动态链接库路径导入到 `LD_LIBRARY_PATH` 路径中。`export  LD_LIBRARY_PATH = 制作的动态链接库路径`

② 直接在 `.bashrc`文件中去配置 `export  LD_LIBRARY_PATH = 制作的动态链接库路径`。每次打开终端都会去读取配置的文件。

③ 比较常用的方法 

- 查找动态连接器的配置文件。查找 `/etc` 目录下的 `ld.so.conf`文件
- 将自己制作的动态链接库路径写到配置文件中。要使用绝对路径，完整的动态库位置。
- 更新配置文件。`sudo ldconfig -v`

优点

- 执行程序的体积较小。
- 在程序的接口没有发生改变之前，不需要重新编译程序。 

缺点 

- 发布程序的时候，需要将动态库发布给用户。
- 加载的速度相对静态库比较慢。 

### 4.2.2. Windows平台

C++ 在调用 Dll 中的函数的时候，如果是企业内部的话，肯定是希望三件套的方式(.h .lib .dll)。这样做的话，编写方可以在头文件中写入很多信息，方便调用方的调用。但是，一旦是给其他公司的人使用，而不想让别人看到的话，那编写方肯定是不想让别人看到过多的信息的，你只管调用。
还有一点是 dll 是在调试的时候使用的，lib 是编译的时候使用的，两者是不同时期使用的工具。

# 5. ELF relocatable

<img src="./figures/ELF-relocatable.png">

查看符号表内容

```bash
readlf -s main.o
```



# 6. Tools

## 6.1. readelf

读取 ELF(Executable and Linking Format) 文件中信息

```bash
参数项
-l
  readelf -l libxxx.so

# 
-S 
  查看符号表内容
  readelf -S xxx.out/xxx.so/xxx.a

-r
  查看重定位信息
    
```

## 6.2. size

 查看object文件或者链接库文件中的object文件的各个段(section)的大小及其总的大小。

## 6.3. nm

查看静态库或可执行文件里面的内容（list symbols from object files：列出一个函数库文件中的符号表）。

例如：`nm main.out`， `nm mylib.a` 

## 6.4. pmap

打印一个进程的内存映射（report memory map of a process）。

示例：

```
pmap 进程PID
```

## 6.5. patchelf

patchelf 是一个简单的实用程序，用于修改已存在的 ELF 可执行表（executables）和库（libraries）。它会改变 可执行表的动态加载器（loader），同时也会改变可执行表和库的路径（PATH）。

```bash
--print-needed
  查看一个可执行程序或动态库，依赖于其它哪些模块。
  anna$:/usr/lib$ patchelf --print-needed libmpathcmd.so.0
  libc.so.6


```

## 6.6. objdump

是 Linux 下的反汇编目标文件或者可执行文件的命令，它以一种可阅读的格式让你更多地了解二进制文件可能带有的附加信息。

```bash
常用符号表字段

.text：   已编译程序的机器代码。

.rodata： 只读数据，比如printf语句中的格式串和开关(switch)语句的跳转表。

.data：   已初始化的全局C变量。局部C变量在运行时被保存在栈中，既不出现在.data中，也不出现在.bss节中。

.bss：    未初始化的全局C变量。在目标文件中这个节不占据实际的空间，它仅仅是一个占位符。目标文件格式区分初始化和未初始化变量是为了空间效率在：在目标文件中，未初始化变量不需要占据任何实际的磁盘空间。

.symtab： 一个符号表(symbol table)，它存放在程序中被定义和引用的函数和全局变量的信息。一些程序员错误地认为必须通过-g选项来编译一个程序，得到符号表信息。实际上，每个可重定位目标文件在 `.symtab` 中都有一张符号表。然而，和编译器中的符号表不同，.symtab符号表不包含局部变量的表目。

.rel.text：当链接噐把这个目标文件和其他文件结合时，`.text` 中的许多位置都需要修改。一般而言，任何调用外部函数或者引用全局变量的指令都需要修改。另一方面调用本地函数的指令则不需要修改。注意，可执行目标文件中并不需要重定位信息，因此通常省略，除非使用者显式地指示链接器包含这些信息。

.rel.data：被模块定义或引用的任何全局变量的信息。一般而言，任何已初始化全局变量的初始值是全局变量或者外部定义函数的地址都需要被修改。

.debug：  一个调试符号表，其有些表目是程序中定义的局部变量和类型定义，有些表目是程序中定义和引用的全局变量，有些是原始的C源文件。只有以-g选项调用编译驱动程序时，才会得到这张表。

.line：  原始C源程序中的行号和 `.text` 中机器指令之间的映射。只有以 `-g` 选项调用编译驱动程序时，才会得到这张表。

.strtab：一个字符串表，其内容包括 `.symtab` 和 `.debug` 中的符号表，以及节头部中的节名字。字符串表就是以 `null` 结尾的字符串序列。
```

```bash
参数选项

-a  --archive-headers       显示档案库的成员信息,类似ls -l将lib*.a的信息列出。 
-b bfdname --target=bfdname 指定目标码格式。这不是必须的，objdump能自动识别许多格式，比如： 

objdump -b oasys -m vax -h fu.o 
显示fu.o的头部摘要信息，明确指出该文件是Vax系统下用Oasys编译器生成的目标文件。objdump -i将给出这里可以指定的目标码格式列表。 

-C  --demangle              将底层的符号名解码成用户级名字，除了去掉所开头的下划线之外，还使得C++函数名以可理解的方式显示出来。 
-g --debugging              显示调试信息。企图解析保存在文件中的调试信息并以C语言的语法显示出来。仅仅支持某些类型的调试信息。有些其他的格式被readelf -w支持。 

-e  --debugging-tags        类似 -g 选项，但是生成的信息是和ctags工具相兼容的格式。 
-d --disassemble            从 objfile 中反汇编那些特定指令机器码的 section。 
-D  --disassemble-all       与 -d 类似，但反汇编所有section. 
--prefix-addresses          反汇编的时候，显示每一行的完整地址。这是一种比较老的反汇编格式。 

-EB 
-EL 
--endian={big|little}       指定目标文件的小端。这个项将影响反汇编出来的指令。在反汇编的文件没描述小端信息的时候用。例如S-records. 

-f --file-headers           显示 objfile 中每个文件的整体头部摘要信息。 
-h --section-headers --headers 显示目标文件各个section的头部摘要信息。 
-H --help 简短的帮助信息。 
-j name  --section=name      仅仅显示指定名称为name的section的信息 
-l --line-numbers            用文件名和行号标注相应的目标代码，仅仅和 -d、 -D 或者 -r 一起使用使用 -ld 和使用 -d 的区别不是很大，在源码级调试的时候有用，要求编译时使用了 -g 之类的调试编译选项。 

-m machine --architecture=machine 指定反汇编目标文件时使用的架构，当待反汇编文件本身没描述架构信息的时候(比如 S-records)，这个选项很有用。可以用 -i 选项列出这里能够指定的架构. 

-r --reloc                  显示文件的重定位入口。如果和-d或者-D一起使用，重定位部分以反汇编后的格式显示出来。 
-R  --dynamic-reloc         显示文件的动态重定位入口，仅仅对于动态目标文件意义，比如某些共享库。 
-s  --full-contents         显示指定 section 的完整内容。默认所有的非空section都会被显示。 
-S --source                 尽可能反汇编出源代码，尤其当编译的时候指定了 -g 这种调试参数时，效果比较明显。隐含了 -d 参数。
--show-raw-insn             反汇编的时候，显示每条汇编指令对应的机器码，如不指定 --prefix-addresses，这将是缺省选项。 
--no-show-raw-insn          反汇编时，不显示汇编指令的机器码，如不指定 --prefix-addresses，这将是缺省选项。 
--start-address=address     从指定地址开始显示数据，该选项影响 -d、 -r 和 -s 选项的输出。 
--stop-address=address      显示数据直到指定地址为止，该项影响-d、-r和-s选项的输出。 
-t --syms                   显示文件的符号表入口。类似于 nm -s 提供的信息 
-T --dynamic-syms           显示文件的动态符号表入口，仅仅对动态目标文件意义，比如某些共享库。它显示的信息类似于 nm -D|--dynamic 显示的信息。 
-V --version                版本信息 
--all-headers -x            显示所可用的头信息，包括符号表、重定位入口。 -x 等价于 -a -f -h -r -t 同时指定。 
-z --disassemble-zeroes     一般反汇编输出将省略大块的零，该选项使得这些零块也被反汇编。 
@file                       可以将选项集中到一个文件中，然后使用这个 @file 选项载入。
```


# 7. Build

两种编译构建方式。

- 原生(native)编译构建，即编译构建命令所运行(host)的系统环境和编译构建输出目标(target)的系统环境一致；
- 交叉(cross)编译构建，上述target和host不一致，即在A系统环境构建出在B系统上运行的目标，这在嵌入式开发中尤为多见。
  > 系统环境：GNU的构建工具链中使用CPU指令集架构、厂商、系统内核的三元组合来指示系统环境


# 8. References

- [C++ Dll 编写入门](https://www.cnblogs.com/daocaoren/archive/2012/05/30/2526495.html)
- [编译器的工作过程](http://www.ruanyifeng.com/blog/2014/11/compiler.html)
- [内存管理(详细版)](https://www.cnblogs.com/yif1991/p/5049638.html)：详细的解释了内存四区的相关内容。
- [编译连接出现undefined reference to 的解决方法 ](https://www.cnblogs.com/yanxin880526/p/8124764.html)
