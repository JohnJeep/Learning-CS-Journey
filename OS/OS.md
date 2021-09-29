# 1Basic elements

> memory: 一般指易失性的RAM和非易失性的磁盘。

计算机组成的整体架构入下

<img src="./figures/computer-components.png">

- Processor：控制计算机的操作、执行数据处理功能。Processor 的一种功能：与存储器（memory） 交换数据。各个部分的功能如下：
  - 存储器地址寄存器（memory address register : MAR )：用于确定下一次读/写的内存地址。
  - 存储器缓冲寄存器（memory buffer register
    : MBR ）：存放要写入内存的数据或从内存中读取的数据。
  - 输入输出地址寄存器（I/O address register : I/OAR) ）：用于确定一个特定的输入输出设备。
  - 输出输出缓冲寄存器（I/O buffer register (I/OBR) ）：用于在输入、输出模块和处理器间交换数据。
- PC（Program Counter：程序寄存器）：保存下一次要取的指令地址。
  - IR（Instruction Register：指令寄存器）：存放由处理器处取到的指令。
  
- Main memory：**Stores data and programs.   This memory is typically volatile;**  计算机关机时，内存中内容会丢失；对比磁盘内存时，计算机关机时，磁盘中保存的内容不会丢失。Main memory 也叫  **real memory or primary
  memory**  。

  >内存模块由一系列被定义为有顺序号的地址块组成。每个地址块包含一个二进制数，表示是指令或数据。

- I/O modules:   在计算机与外部环境之间移动移动数据。外部环境由很多设备组成：包括二级存储设备（secondary
  memory devices : disk）、通信设备、终端。

  > 模块中还包含内存缓冲区（buffers），用于临时保存数据，直到它们被发送出去。

- System bus:   系统总线为处理器、主存和 I/O 模块之间提供通信。



# 2 指令执行

一个程序被处理器执行由一系列存储在 memory 中的指令组成。最简单的指令包括两步：处理器从 memory 中取（fetches）一条指令，然后执行指令。

程序执行是由不断重复的**取指令**和**执行指令**的过程组成的。指令执行涉及很多操作，具体取决于指令本身。

