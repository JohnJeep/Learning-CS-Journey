```
 * @Author: your name
 * @Date: 2020-05-18 21:33:28
 * @LastEditTime: 2020-05-18 21:34:14
 * @LastEditors: Please set LastEditors
 * @Description: 常用的通用寄存器
```
### 指令(instruction)
- **DCD**  指令可分配一个或多个字的内存，在四个字节的边界上对齐，并定义内存的运行时初值
- **DCDU** 与之相同，不过内存对齐是任意的
- 分段：在实模式下通过偏移一个段寄存器的4位再加上16位偏移量，形成一个20位的物理地址


### 寄存器 
Inter 8086架构有16个处理器，可供程序员使用的有14个16位的寄存器；有16位宽数据总线和地址总线，20位宽的外部地址总线
- 8个通用寄存器：
  - 4个数据寄存器
    - `EAX(Extended Accumulator X)`	 累加寄存器
    - `EBP(Extended Base Register)`	 基址寄存器
    - `ECX(Extended Counting X	 )`  计数寄存器
    - `EDX(Extended Data Register)`  数据寄存器
    

  - 2个变址寄存器
    - `EDI(Extended Destination Indexing)`  目的变址寄存器
    - `ESI(Extended Source Indexing)     `  源变址寄存器
  - 2个指针寄存器
    - `ESP(Extended Stack Pointer)`	栈指针寄存器
    - `EBP(Extended Base pointer )`  基址指针寄存器


- `EIP(Extended Instructions Pointer)`	指令指针寄存器 
- `EFR(Extended Flag Register)       `   标志寄存器
- 4个16位段寄存器 
  > 段的起始地址称为段寄存器 
  - `CS(Code Segment) ` 代码段寄存器 
  - `DS(Data Segment) ` 数据段寄存器 
  - `SS(Stack Segment)` 堆栈段寄存器 
  - `ES(Extra Segment)` 附加段寄存器 


- `OA(Offset Address)` 偏移地址
- `EA(Effective Address)` 有效地址









