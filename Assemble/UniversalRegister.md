<!--
 * @Author: JohnJeep
 * @Date: 2020-05-18 21:33:28
 * @LastEditTime: 2020-09-23 11:53:28
 * @LastEditors: Please set LastEditors
 * @Description: 常用的通用寄存器
 * @FilePath: /UniversalRegister.md
-->
<!-- TOC -->

- [0.1. 指令(instruction)](#01-指令instruction)
- [0.2. 寄存器](#02-寄存器)
- [0.3. Machine-Level Representation](#03-machine-level-representation)
  - [0.3.1. Operand Specifiers(操作数指示符)](#031-operand-specifiers操作数指示符)
  - [0.3.2. Data Movement Instructions(数据移动指令)](#032-data-movement-instructions数据移动指令)
  - [0.3.3. Arithmetic and Logical Operations(算术与逻辑运算)](#033-arithmetic-and-logical-operations算术与逻辑运算)

<!-- /TOC -->

## 0.1. 指令(instruction)
- **DCD**  指令可分配一个或多个字的内存，在四个字节的边界上对齐，并定义内存的运行时初值
- **DCDU** 与之相同，不过内存对齐是任意的
- 分段：在实模式下通过偏移一个段寄存器的4位再加上16位偏移量，形成一个20位的物理地址


## 0.2. 寄存器 
Inter 8086架构有16个处理器，可供程序员使用的有14个16位的寄存器；有16位宽数据总线和地址总线，20位宽的外部地址总线。

> Intel架构的处理器，指令集命令规则演变：最初的8086中有8个 `16 bite` 的寄存器，习惯命名为 `%ax---%bp`，然后扩展到IA32架构时，命名标号变为 `%eax---%ebp`，最后扩展到X86-64架构后，原来的8个寄存器由 `32 bite`变为 `64 bite`，标号也变为 `%rax---%rbp`，还新增了 8 个新的寄存器，标号按照新的命名规则制定的 `%r8---%r15`。


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
    - `ESP(Extended Stack Pointer)`	栈指针寄存器：用来指明运行时栈的结束位置。
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



## 0.3. Machine-Level Representation
- 参考
  - [Assembly language primer](http://www.unige.ch/medecine/nouspikel/ti99/assembly.htm#JUMPs) 


- 缩写
  - CISC(Complex Instruction Set Computer): 复杂指令集计算机
  - RISC(Reduced Instruction  Set Computer) ：精简指令集计算机
  - ARM：Acorn Risc Machine


- 反汇编器(disassembler): 根据机器代码产生一种类似于汇编代码的程序。
- GCC和Objdump工具产生的汇编代码默认使用的是 `AT&T公司` 拟定的格式。
- 在GCC中使用一些参数可以产生Intel公司拟定的汇编代码格式：` gcc test.c -S  -masm=intel`






### 0.3.1. Operand Specifiers(操作数指示符)

> 大多数指令有一个或多个 Operand Specifiers，源数据(source values) 放置可执行的操作，目的位置(destination) 放置计算的结果。

- Source values 支持的操作数(operand)格式
  - constants(常数，也叫立即数)
  - read from registers or memory(从寄存器或内存中读出的数)
- Destination of Result 目的位置计算的结果支持的操作数格式
  - Register(寄存器)
  - Memory(内存)  


- 各种不同的操作数可分为下面三种类型 
  - Immediate(立即数)：表示常数值
    - 在AT&T公司格式的汇编代码中，立即数的书写是 `$` 后面跟一个用标准C表示法表示的整数。例如：`$-577` 或 `$0x1F ` 

  - Register(寄存器)：表示某个寄存器中的内容
    - 用符号 $r_a$ 表示任意寄存器 a，用引用 $R[r_a]$ 表示它的值，这是将寄存器集合看成一个数组 R，用寄存器标识符作为index(索引)。
    - 可以把16个寄存器的低8位，16位，32位或64位中的一个来作为操作数。

  - memory reference(内存引用)：根据计算出来的地址访问某个内存的位置。
    - 可以将内存看成一个很大的字节数组 (we view the memory as a large array of bytes)。 
    - 用符号 $M_b[Addr]$ 表示对存储在内存中从地址 Addr开始的 b 个字节值的reference(引用)。

    - 汇编语言中常用的内存引用表示格式：$Imm(r_{b}, r_{i}, s)$，有效地址的计算公式：$Imm + R[r_b] + R[r_i]*s$
      - Imm: immediate offset (立即数偏移)
      - $r_b$: base register(64-bit基址寄存器)
      - $r_i$: index register(64-bit变址寄存器)
      - s: scale factor(比例因子，s必须是1、2、4或8)
  <img src="./figures/操作数格式.png">



### 0.3.2. Data Movement Instructions(数据移动指令)
- 指令集的格式 `[标号:] 操作码 [操作数] [;注释]`；带有 `[]` 的部分为可选项。

<font color=red>注意：</font>
GNU汇编器使用 AT&T 样式的语法，所以其中的源和目的操作数和 Intel 文档中给出的顺序是相反的。


- mov
  - Intel架构下的格式：`mov dest, src` ；AT&T公司规定的格式：`mov source destination`
  - 功能：将source位置中的数据copy到destination位置中。注意：执行这个操作需要两条指令，第一条指令将源值(source value)加载到寄存器中，第二条指令将该寄存器值写入目的位置(destination)。
  - 几种简单的移动指令
    - movb: move byte(移动字节)
    - movw: move word(移动字)
    - movl: move double word(32 bit 数据被看成是long word，因此用后缀“l” 表示双字)
    - movq: move quad word(移动四字)
     <img src="./figures/mov指令.png">
    > 常规的 `movq` 指令只能以表示为32位补码数字的立即数作为源操作数，然后把这个值符号扩展得到64位的值，放到目的(destination)位置。`movabsq` 指令能够以任意6位立即数值作为源操作数(source operand)，并且只能以寄存器(register)作为目的(destination)。


  - 上面几个指令实现的例子
    <img src="./figures/mov指令移动例子.png">

  - source operand(源操作数)支持以下几种类型
    - Immediate(立即数)
    - Register(寄存器)
    - Memory(内存) 
  - destination operand(目的操作数)支持以下几种类型
    - Register(寄存器)
    - Memory address (内存地址)




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
  - pushq: push quad word(每次压栈按4字为单位进行压栈)


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


### 0.3.3. Arithmetic and Logical Operations(算术与逻辑运算)
- lea(load effective address)：加载有效地址
  - 是mov指令的一个变形，从内存（memory）读数据到寄存器（register）。该指令并不是从指定的位置读入数据，而是将有效地址写入到目的操作数。
  - lea指令通常用来执行简单的算术操作，能执行加法和有限形式的乘法。
  - 指令：`leaq src dst`
     - `leaq 7(%rdx, %rdx, 4)`表示: 寄存器 %rdx 的值为x,  寄存器 %rdx 将寄存器 %rax 的值设置为：`(x + 4*x) + 7`
