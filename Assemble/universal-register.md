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
    - `EBP(Extended Base Register X)`	 基址寄存器
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


### 基础语法
- 参考
  - [Assembly language primer](http://www.unige.ch/medecine/nouspikel/ti99/assembly.htm#JUMPs) 


- 格式 `[标号:] 操作码 [操作数] [;注释]`
  > 带有 [] 的部分为可选项。


- mov
  - 格式：`mov dest, src` 
  - 功能：将src中的内容复制到dest中。
- add
  - 格式：`add dest src`
  - 功能：`dest ← dest + src`
- sub
  - 格式：`sub dest src`
  - 功能：`dest ← dest - src`
- xchg
  - 格式：`xchg dest src`
  - 功能：将src中的内容与dest内容经行交换。
- test
  - 格式：`test dest src`
  - 功能：`dest ^ src` dest和src进行逻辑与运算
- and
  - 格式：`and dest src`
  - 功能：`dest ← dest ^ src` dest和src进行逻辑与运算
- dec
  - 格式：`add dest`
  - 功能：`dest ← dest - 1`

- push
- pop 
- call


- je 等于零
- jgt 大于则跳（Jump if Greater Than）
- jg  大于（jump greater）
- jge(或jgte) 大于等于（jump greater than or equal）
- jl  小于（jump less）
- jle 小于等于（jump less equal）
- jne 不等于（jump not equal）



标志位
- CF - carry flag
  > Set on high-order bit carry or borrow; cleared otherwise

- PF - parity flag
  > Set if low-order eight bits of result contain an even number of "1" bits; cleared otherwise

- ZF - zero flags
  > Set if result is zero; cleared otherwise

- SF - sign flag
  > Set equal to high-order bit of result (0 if positive 1 if negative)

- OF - overflow flag
  > Set if result is too large a positive number or too small a negative number (excluding sign bit) to fit in destination operand; cleared otherwise


