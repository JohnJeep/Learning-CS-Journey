---
title: C语言编程规范
data: 2025-03-30 00:04:11
tags: ['C']
category: C
---

### 1、清晰第一

清晰性是易于维护、易于重构的程序必需具备的特征。代码首先是给人读的,好的代码应当可以像文章一样发声朗诵出来。

### 2.、简洁为美

简洁就是易于理解并且易于实现。代码越长越难以看懂，也就越容易在修改时引入错误。写的代码越多，意味着出错的地方越多，也就意味着代码的可靠性越低。

因此，我们提倡大家通过编写简洁明了的代码来提升代码可靠性。废弃的代码(没有被调用的函数和全局变量)要及时清除，重复代码应该尽可能提炼成函数。

### 3、选择合适的风格，与代码原有的风格保持一致

产品所有人共同分享同一种风格所带来的好处，远远超出为了统一而付出的代价。在公司已有编码规范的指导下，审慎地编排代码以使代码尽可能清晰，是一项非常重要的技能。

如果重构/修改其他风格的代码时，比较明智的做法是根据现有代码的现有风格继续编写代码,或者使用格式转换工具进行转换成公司内部风格。

## 一、头文件

**原则1.1 头文件中适合放置接口的声明,不适合放置实现。**

说明:头文件是模块(Module)或单元(Unit)的对外接口。头文件中应放置对外部的声明,如对外提供的函数声明、宏定义、类型定义等。

**原则1.2 头文件应当职责单一。**

说明:头文件过于复杂，依赖过于复杂是导致编译时间过长的主要原因。很多现有代码中头文件过大，职责过多，再加上循环依赖的问题，可能导致为了在.c中使用一个宏，而包含十几个头文件。

**原则1.3 头文件应向稳定的方向包含。**

说明:头文件的包含关系是一种依赖，一般来说，应当让不稳定的模块依赖稳定的模块，从而当不稳定的模块发生变化时，不会影响(编译)稳定的模块。

**规则1.1 每一个.c文件应有一个同名.h文件,用于声明需要对外公开的接口。**

说明:如果一个.c文件不需要对外公布任何接口，则其就不应当存在，除非它是程序的入口，如main函数所在的文件。

**规则1.2 禁止头文件循环依赖。**

说明:头文件循环依赖，指a.h包含b.h，b.h包含c.h，c.h包含a.h之类导致任何一个头文件修改，都导致所有包含了a.h/b.h/c.h的代码全部重新编译一遍。

而如果是单向依赖，如a.h包含b.h，b.h包含c.h,而c.h不包含任何头文件，则修改a.h不会导致包含了b.h/c.h的源代码重新编译。

**规则1.3 .c/.h文件禁止包含用不到的头文件。**

说明:很多系统中头文件包含关系复杂，开发人员为了省事起见,可能不会去一一钻研，直接包含一切想到的头文件，甚至有些产品干脆发布了一个god.h,其中包含了所有头文件，然后发布给各个项目组使用，这种只图一时省事的做法，导致整个系统的编译时间进一步恶化,并对后来人的维护造成了巨大的麻烦。

**规则1.4 头文件应当自包含。**

说明:简单的说，自包含就是任意一个头文件均可独立编译。如果一个文件包含某个头文件，还要包含另外一个头文件才能工作的话，就会增加交流障碍，给这个头文件的用户增添不必要的负担。

**规则1.5 总是编写内部#include保护符(#define 保护)。**

说明:多次包含一个头文件可以通过认真的设计来避免。如果不能做到这一点，就需要采取阻止头文件内容被包含多于一次的机制。

注: 没有在宏最前面加上 _ ，即使用 FILENAME_H代替  *FILENAME_H* ，是因为一般以 _ 和  __ 开头的标识符为系统保留或者标准库使用，在有些静态检查工具中，若全局可见的标识符以 _ 开头会给出告警。

定义包含保护符时，应该遵守如下规则:

1)保护符使用唯一名称;

2)不要在受保护部分的前后放置代码或者注释。

**规则1.6 禁止在头文件中定义变量。**

说明:在头文件中定义变量，将会由于头文件被其他.c文件包含而导致变量重复定义。

**规则1.7 只能通过包含头文件的方式使用其他.c提供的接口,禁止在.c中通过extern的方式使用外部函数接口、变量。**

说明:若a.c使用了b.c定义的foo()函数，则应当在b.h中声明extern int foo(int input)；并在a.c中通过#include <b.h>来使用foo。禁止通过在a.c中直接写extern int foo(int input)；来使用foo，后面这种写法容易在foo改变时可能导致声明和定义不一致。这一点我们因为图方便经常犯的。

**规则1.8 禁止在extern "C"中包含头文件。**

说明:在extern "C"中包含头文件，会导致extern "C"嵌套，Visual Studio对extern "C"嵌套层次有限制，嵌套层次太多会编译错误。

**建议1.1 一个模块通常包含多个.c文件,建议放在同一个目录下,目录名即为模块名。为方便外部使用者,建议每一个模块提供一个.h,文件名为目录名。**

**建议1.2 如果一个模块包含多个子模块,则建议每一个子模块提供一个对外的.h,文件名为子模块名。**

**建议1.3 头文件不要使用非习惯用法的扩展名,如.inc。**

**建议1.4 同一产品统一包含头文件排列方式。**

## 二、函数

**原则2.1 一个函数仅完成一件功能。**

说明:一个函数实现多个功能给开发、使用、维护都带来很大的困难。

**原则2.2 重复代码应该尽可能提炼成函数。**

说明:重复代码提炼成函数可以带来维护成本的降低。

**规则2.1 避免函数过长,新增函数不超过50行(非空非注释行)。**

说明:本规则仅对新增函数做要求，对已有函数修改时，建议不增加代码行。

**规则2.2 避免函数的代码块嵌套过深,新增函数的代码块嵌套不超过4层。**

说明:本规则仅对新增函数做要求，对已有的代码建议不增加嵌套层次。

**规则2.3 可重入函数应避免使用共享变量;若需要使用,则应通过互斥手段(关中断、信号量)对其加以保护。**

**规则2.4 对参数的合法性检查,由调用者负责还是由接口函数负责,应在项目组/模块内应统一规定。缺省由调用者负责。**

**规则2.5 对函数的错误返回码要全面处理。**

**规则2.6 设计高扇入,合理扇出(小于7)的函数。**

说明:扇出是指一个函数直接调用(控制)其它函数的数目,而扇入是指有多少上级函数调用它。如下图：

![图片](https://mmbiz.qpic.cn/mmbiz_png/PnO7BjBKUzicXANEqlQfMdytEpr2YcicfEiczdBn8szEvZcnqnKr4c0esta68oKoYQ6JDMTf6iaHicDwxjtOIO9wMgw/640?wx_fmt=png&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)

**规则2.7 废弃代码(没有被调用的函数和变量)要及时清除。**

**建议2.1 函数不变参数使用const。**

说明:不变的值更易于理解/跟踪和分析，把const作为默认选项，在编译时会对其进行检查，使代码更牢固/更安全。

**建议2.2 函数应避免使用全局变量、静态局部变量和I/O操作,不可避免的地方应集中使用。**

**建议2.3 检查函数所有非参数输入的有效性,如数据文件、公共变量等。**

说明:函数的输入主要有两种:一种是参数输入;另一种是全局变量、数据文件的输入，即非参数输入。函数在使用输入参数之前，应进行有效性检查。

**建议2.4 函数的参数个数不超过5个。**

**建议2.5 除打印类函数外,不要使用可变长参函数。**

**建议2.6 在源文件范围内声明和定义的所有函数,除非外部可见,否则应该增加static关键字。**

## 三、 标识符命名与定义

目前比较常用的如下几种命名风格:

unix like风格：单词用小写字母,每个单词直接用下划线_分割,，例如text_mutex,kernel_text_address。

Windows风格：大小写字母混用，单词连在一起,每个单词首字母大写。不过Windows风格如果遇到大写专有用语时会有些别扭,例如命名一个读取RFC文本的函数，命令为ReadRFCText,看起来就没有unix like的read_rfc_text清晰了。

**原则3.1 标识符的命名要清晰、明了,有明确含义,同时使用完整的单词或大家基本可以理解的缩写,避免使人产生误解。**

**原则3.2 除了常见的通用缩写以外,不使用单词缩写,不得使用汉语拼音。**

**建议3.1 产品/项目组内部应保持统一的命名风格。**

**建议3.2 尽量避免名字中出现数字编号,除非逻辑上的确需要编号。**

**建议3.3 标识符前不应添加模块、项目、产品、部门的名称作为前缀。**

**建议3.4 平台/驱动等适配代码的标识符命名风格保持和平台/驱动一致。**

**建议3.5 重构/修改部分代码时,应保持和原有代码的命名风格一致。**

**建议3.6 文件命名统一采用小写字符。**

**规则3.2 全局变量应增加“g_”前缀。**

**规则3.3 静态变量应增加“s_”前缀。**

**规则3.4 禁止使用单字节命名变量,但允许定义i、j、k作为局部循环变量。**

**建议3.7 不建议使用匈牙利命名法。**

说明:变量命名需要说明的是变量的含义，而不是变量的类型。在变量命名前增加类型说明，反而降低了变量的可读性；更麻烦的问题是，如果修改了变量的类型定义，那么所有使用该变量的地方都需要修改。

**建议3.8 使用名词或者形容词+名词方式命名变量。**

**建议3.9 函数命名应以函数要执行的动作命名,一般采用动词或者动词+名词的结构。**

**建议3.10 函数指针除了前缀,其他按照函数的命名规则命名。**

**规则3.5 对于数值或者字符串等等常量的定义,建议采用全大写字母,单词之间加下划线„_‟的方式命名(枚举同样建议使用此方式定义)。**

**规则3.6 除了头文件或编译开关等特殊标识定义,宏定义不能使用下划线„_‟开头和结尾。**

## 四、变量

**原则4.1 一个变量只有一个功能,不能把一个变量用作多种用途。**

**原则4.2 结构功能单一;不要设计面面俱到的数据结构。**

**原则4.3 不用或者少用全局变量。**

**规则4.1 防止局部变量与全局变量同名。**

**规则4.2 通讯过程中使用的结构,必须注意字节序。**

**规则4.3 严禁使用未经初始化的变量作为右值。**

**建议4.1 构造仅有一个模块或函数可以修改、创建,而其余有关模块或函数只访问的全局变量,防止多个不同模块或函数都可以修改、创建同一全局变量的现象。**

**建议4.2 使用面向接口编程思想,通过API访问数据:如果本模块的数据需要对外部模块开放,应提供接口函数来设置、获取,同时注意全局数据的访问互斥。**

**建议4.3 在首次使用前初始化变量,初始化的地方离使用的地方越近越好。**

**建议4.4 明确全局变量的初始化顺序,避免跨模块的初始化依赖。**

说明:系统启动阶段，使用全局变量前，要考虑到该全局变量在什么时候初始化，使用全局变量和初始化全局变量，两者之间的时序关系，谁先谁后，一定要分析清楚，不然后果往往是低级而又灾难性的。

**建议4.5 尽量减少没有必要的数据类型默认转换与强制转换。**

说明:当进行数据类型强制转换时，其数据的意义、转换后的取值等都有可能发生变化，而这些细节若考虑不周，就很有可能留下隐患。

## 五、 宏、常量

**规则5.1 用宏定义表达式时,要使用完备的括号。**

说明:因为宏只是简单的代码替换，不会像函数一样先将参数计算后，再传递。

**规则5.2 将宏所定义的多条表达式放在大括号中。**

说明:更好的方法是多条语句写成do while(0)的方式。

**规则5.3 使用宏时，不允许参数发生变化。**

**规则5.4 不允许直接使用魔鬼数字。**

说明:使用魔鬼数字的弊端：代码难以理解；如果一个有含义的数字多处使用，一旦需要修改这个数值，代价惨重。使用明确的物理状态或物理意义的名称能增加信息，并能提供单一的维护点。

**建议5.1 除非必要,应尽可能使用函数代替宏。**

说明:宏对比函数，有一些明显的缺点：宏缺乏类型检查，不如函数调用检查严格。

**建议5.2 常量建议使用const定义代替宏。**

**建议5.3 宏定义中尽量不使用return、goto、continue、break等改变程序流程的语句。**

## 六、质量保证

**原则6.1 代码质量保证优先原则**

(1)正确性，指程序要实现设计要求的功能。

(2)简洁性，指程序易于理解并且易于实现。

(3)可维护性，指程序被修改的能力，包括纠错、改进、新需求或功能规格变化的适应能力。

(4)可靠性，指程序在给定时间间隔和环境条件下，按设计要求成功运行程序的概率。

(5)代码可测试性，指软件发现故障并隔离、定位故障的能力，以及在一定的时间和成本前提下，进行测试设计、测试执行的能力。

(6)代码性能高效，指是尽可能少地占用系统资源，包括内存和执行时间。

(7)可移植性，指为了在原来设计的特定环境之外运行，对系统进行修改的能力。

(8)个人表达方式/个人方便性，指个人编程习惯。

**原则6.2 要时刻注意易混淆的操作符。比如说一些符号特性、计算优先级。**

**原则6.3 必须了解编译系统的内存分配方式,特别是编译系统对不同类型的变量的内存分配规则,如局部变量在何处分配、静态变量在何处分配等。**

**原则6.4 不仅关注接口,同样要关注实现。**

说明:这个原则看似和“面向接口”编程思想相悖，但是实现往往会影响接口，函数所能实现的功能，除了和调用者传递的参数相关，往往还受制于其他隐含约束，如:物理内存的限制，网络状况，具体看“抽象漏洞原则”。

**规则6.1 禁止内存操作越界。**

坚持下列措施可以避免内存越界:

-   数组的大小要考虑最大情况，避免数组分配空间不够。
-   避免使用危险函数sprintf /vsprintf/strcpy/strcat/gets操作字符串，使用相对安全的函数snprintf/strncpy/strncat/fgets代替。
-   使用memcpy/memset时一定要确保长度不要越界
-   字符串考虑最后的’\0’， 确保所有字符串是以’\0’结束
-   指针加减操作时，考虑指针类型长度
-   数组下标进行检查
-   使用时sizeof或者strlen计算结构/字符串长度,，避免手工计算

坚持下列措施可以避免内存泄漏:

-   异常出口处检查内存、定时器/文件句柄/Socket/队列/信号量/GUI等资源是否全部释放
-   删除结构指针时，必须从底层向上层顺序删除
-   使用指针数组时，确保在释放数组时，数组中的每个元素指针是否已经提前被释放了
-   避免重复分配内存
-   小心使用有return、break语句的宏，确保前面资源已经释放
-   检查队列中每个成员是否释放

**规则6.3 禁止引用已经释放的内存空间。**

-   坚持下列措施可以避免引用已经释放的内存空间:
-   内存释放后，把指针置为NULL；使用内存指针前进行非空判断。
-   耦合度较强的模块互相调用时，一定要仔细考虑其调用关系，防止已经删除的对象被再次使用。
-   避免操作已发送消息的内存。
-   自动存储对象的地址不应赋值给其他的在第一个对象已经停止存在后仍然保持的对象(具有更大作用域的对象或者静态对象或者从一个函数返回的对象)

**规则6.4 编程时,要防止差1错误。**

说明:此类错误一般是由于把“<=”误写成“<”或“>=”误写成“>”等造成的,由此引起的后果，很多情况下是很严重的，所以编程时，一定要在这些地方小心。当编完程序后，应对这些操作符进行彻底检查。使用变量时要注意其边界值的情况。

**建议6.1 函数中分配的内存,在函数退出之前要释放。**

说明:有很多函数申请内存,，保存在数据结构中，要在申请处加上注释，说明在何处释放。

**建议6.2 if语句尽量加上else分支,对没有else分支的语句要小心对待。**

**建议6.3 不要滥用goto语句。**

说明:goto语句会破坏程序的结构性,所以除非确实需要,最好不使用goto语句。

**建议6.4 时刻注意表达式是否会上溢、下溢。**

## 七、 程序效率

**原则7.1 在保证软件系统的正确性、简洁、可维护性、可靠性及可测性的前提下,提高代码效率。**

**原则7.2 通过对数据结构、程序算法的优化来提高效率。**

**建议7.1 将不变条件的计算移到循环体外。**

**建议7.2 对于多维大数组,避免来回跳跃式访问数组成员。**

**建议7.3 创建资源库,以减少分配对象的开销。**

**建议7.4 将多次被调用的 “小函数”改为inline函数或者宏实现。**

## 八、 注释

**原则8.1 优秀的代码可以自我解释,不通过注释即可轻易读懂。**

说明:优秀的代码不写注释也可轻易读懂,注释无法把糟糕的代码变好,需要很多注释来解释的代码往往存在坏味道,需要重构。

**原则8.2 注释的内容要清楚、明了,含义准确,防止注释二义性。**

**原则8.3 在代码的功能、意图层次上进行注释,即注释解释代码难以直接表达的意图,而不是重复描述代码。**

**规则8.1 修改代码时,维护代码周边的所有注释,以保证注释与代码的一致性。不再有用的注释要删除。**

**规则8.2 文件头部应进行注释,注释必须列出:版权说明、版本号、生成日期、作者姓名、工号、内容、功能说明、与其它文件的关系、修改日志等,头文件的注释中还应有函数功能简要说明。**

**规则8.3 函数声明处注释描述函数功能、性能及用法,包括输入和输出参数、函数返回值、可重入的要求等;定义处详细描述函数功能和实现要点,如实现的简要步骤、实现的理由、设计约束等。**

**规则8.4 全局变量要有较详细的注释,包括对其功能、取值范围以及存取时注意事项等的说明。**

**规则8.5 注释应放在其代码上方相邻位置或右方,不可放在下面。如放于上方则需与其上面的代码用空行隔开,且与下方代码缩进相同。**

**规则8.6 对于switch语句下的case语句,如果因为特殊情况需要处理完一个case后进入下一个case处理,必须在该case语句处理完、下一个case语句前加上明确的注释。**

**规则8.7 避免在注释中使用缩写,除非是业界通用或子系统内标准化的缩写。**

**规则8.8 同一产品或项目组统一注释风格。**

**建议8.1 避免在一行代码或表达式的中间插入注释。**

**建议8.2 注释应考虑程序易读及外观排版的因素,使用的语言若是中、英兼有的,建议多使用中文,除非能用非常流利准确的英文表达。对于有外籍员工的,由产品确定注释语言。**

**建议8.3 文件头、函数头、全局常量变量、类型定义的注释格式采用工具可识别的格式。**

说明:采用工具可识别的注释格式,例如doxygen格式,方便工具导出注释形成帮助文档。

## 九、 排版与格式

**规则9.1 程序块采用缩进风格编写,每级缩进为4个空格。**

说明:当前各种编辑器/IDE都支持TAB键自动转空格输入,需要打开相关功能并设置相关功能。编辑器/IDE如果有显示TAB的功能也应该打开,方便及时纠正输入错误。

**规则9.2 相对独立的程序块之间、变量说明之后必须加空行。**

**规则9.3 一条语句不能过长,如不能拆分需要分行写。一行到底多少字符换行比较合适,产品可以自行确定。**

换行时有如下建议:

-   换行时要增加一级缩进,使代码可读性更好;
-   低优先级操作符处划分新行;换行时操作符应该也放下来,放在新行首;
-   换行时建议一个完整的语句放在一行,不要根据字符数断行

**规则9.4 多个短语句(包括赋值语句)不允许写在同一行内,即一行只写一条语句。**

**规则9.5 if、for、do、while、case、switch、default等语句独占一行。**

**规则9.6 在两个以上的关键字、变量、常量进行对等操作时,它们之间的操作符之前、之后或者前后要加空格;进行非对等操作时,如果是关系密切的立即操作符(如->),后不应加空格。**

**建议9.1 注释符(包括„/\*‟„//‟„\*/‟)与注释内容之间要用一个空格进行分隔。**

**建议9.2 源程序中关系较为紧密的代码应尽可能相邻。**

## 十、 表达式

**规则10.1 表达式的值在标准所允许的任何运算次序下都应该是相同的。**

**建议10.1 函数调用不要作为另一个函数的参数使用,否则对于代码的调试、阅读都不利。**

**建议10.2 赋值语句不要写在if等语句中,或者作为函数的参数使用。**

**建议10.3 赋值操作符不能使用在产生布尔值的表达式上。**

## 十一、 代码编辑、编译

**规则11.1 使用编译器的最高告警级别,理解所有的告警,通过修改代码而不是降低告警级别来消除所有告警。**

**规则11.2 在产品软件(项目组)中,要统一编译开关、静态检查选项以及相应告警清除策略。**

**规则11.3 本地构建工具(如PC-Lint)的配置应该和持续集成的一致。**

**规则11.4 使用版本控制(配置管理)系统,及时签入通过本地构建的代码,确保签入的代码不会影响构建成功。**

**建议11.1 要小心地使用编辑器提供的块拷贝功能编程。**

## 十二、 可测性

**原则12.1 模块划分清晰,接口明确,耦合性小,有明确输入和输出,否则单元测试实施困难。**

说明:单元测试实施依赖于:

-   模块间的接口定义清楚、完整、稳定;
-   模块功能的有明确的验收条件(包括:预置条件、输入和预期结果);
-   模块内部的关键状态和关键数据可以查询,可以修改;
-   模块原子功能的入口唯一;
-   模块原子功能的出口唯一;
-   依赖集中处理:和模块相关的全局变量尽量的少,或者采用某种封装形式。

**规则12.1 在同一项目组或产品组内,要有一套统一的为集成测试与系统联调准备的调测开关及相应打印函数,并且要有详细的说明。**

**规则12.2 在同一项目组或产品组内,调测打印的日志要有统一的规定。**

说明:统一的调测日志记录便于集成测试,具体包括:

-   统一的日志分类以及日志级别;
-   通过命令行、网管等方式可以配置和改变日志输出的内容和格式;
-   在关键分支要记录日志,日志建议不要记录在原子函数中,否则难以定位;
-   调试日志记录的内容需要包括文件名/模块名、代码行号、函数名、被调用函数名、错误码、错误发生的环境等。

**规则12.3 使用断言记录内部假设。**

**规则12.4 不能用断言来检查运行时错误。**

说明:断言是用来处理内部编程或设计是否符合假设;不能处理对于可能会发生的且必须处理的情况要写防错程序,而不是断言。

如某模块收到其它模块或链路上的消息后,要对消息的合理性进行检查,此过程为正常的错误检查,不能用断言来实现。

**建议12.1 为单元测试和系统故障注入测试准备好方法和通道。**

## 十三、 安全性

**原则13.1 对用户输入进行检查。**

说明:不能假定用户输入都是合法的,因为难以保证不存在恶意用户,即使是合法用户也可能由于误用误操作而产生非法输入。用户输入通常需要经过检验以保证安全,特别是以下场景:

-   用户输入作为循环条件
-   用户输入作为数组下标
-   用户输入作为内存分配的尺寸参数
-   用户输入作为格式化字符串
-   用户输入作为业务数据(如作为命令执行参数、拼装sql语句、以特定格式持久化)

这些情况下如果不对用户数据做合法性验证,很可能导致DOS、内存越界、格式化字符串漏洞、命令注入、SQL注入、缓冲区溢出、数据破坏等问题。

可采取以下措施对用户输入检查:

-   用户输入作为数值的,做数值范围检查
-   用户输入是字符串的,检查字符串长度
-   用户输入作为格式化字符串的,检查关键字“%”
-   用户输入作为业务数据,对关键字进行检查、转义

**规则13.1 确保所有字符串是以NULL结束。**

说明: C语言中‟\0‟作为字符串的结束符,即NULL结束符。标准字符串处理函数(如strcpy()、 strlen())

依赖NULL结束符来确定字符串的长度。没有正确使用NULL结束字符串会导致缓冲区溢出和其它未定义的行为。

为了避免缓冲区溢出,常常会用相对安全的限制字符数量的字符串操作函数代替一些危险函数。如:

-   用strncpy()代替strcpy()
-   用strncat()代替strcat()
-   用snprintf()代替sprintf()
-   用fgets()代替gets()

这些函数会截断超出指定限制的字符串,但是要注意它们并不能保证目标字符串总是以NULL结尾。如果源字符串的前n个字符中不存在NULL字符,目标字符串就不是以NULL结尾。

**规则13.2 不要将边界不明确的字符串写到固定长度的数组中。**

说明:边界不明确的字符串(如来自gets()、getenv()、scanf()的字符串),长度可能大于目标数组长度,直接拷贝到固定长度的数组中容易导致缓冲区溢出。

**规则13.3 避免整数溢出。**

说明:当一个整数被增加超过其最大值时会发生整数上溢,被减小小于其最小值时会发生整数下溢。带符号和无符号的数都有可能发生溢出。

**规则13.4 避免符号错误。**

说明:有时从带符号整型转换到无符号整型会发生符号错误,符号错误并不丢失数据,但数据失去了原来的含义。

带符号整型转换到无符号整型,最高位(high-order bit)会丧失其作为符号位的功能。如果该带符号整数的值非负,那么转换后值不变;如果该带符号整数的值为负,那么转换后的结果通常是一个非常大的正数。

**规则13.5:避免截断错误。**

说明:将一个较大整型转换为较小整型,并且该数的原值超出较小类型的表示范围,就会发生截断错误,原值的低位被保留而高位被丢弃。截断错误会引起数据丢失。使用截断后的变量进行内存操作,很可能会引发问题。

**规则13.6:确保格式字符和参数匹配。**

说明:使用格式化字符串应该小心,确保格式字符和参数之间的匹配,保留数量和数据类型。格式字符和参数之间的不匹配会导致未定义的行为。大多数情况下,不正确的格式化字符串会导致程序异常终止。

**规则13.7 避免将用户输入作为格式化字符串的一部分或者全部。**

说明:调用格式化I/O函数时,不要直接或者间接将用户输入作为格式化字符串的一部分或者全部。攻击者对一个格式化字符串拥有部分或完全控制,存在以下风险:进程崩溃、查看栈的内容、改写内存、甚至执行任意代码。

**规则13.8 避免使用strlen()计算二进制数据的长度。**

说明:strlen()函数用于计算字符串的长度,它返回字符串中第一个NULL结束符之前的字符的数量。因此用strlen()处理文件I/O函数读取的内容时要小心,因为这些内容可能是二进制也可能是文本。

**规则13.9 使用int类型变量来接受字符I/O函数的返回值。**

**规则13.10 防止命令注入。**

说明:C99函数system()通过调用一个系统定义的命令解析器(如UNIX的shell,Windows的CMD.exe)来执行一个指定的程序/命令。类似的还有POSIX的函数popen()。

## 十四、 单元测试

**规则14.1 在编写代码的同时,或者编写代码前,编写单元测试用例验证软件设计/编码的正确。**

**建议14.1 单元测试关注单元的行为而不是实现,避免针对函数的测试。**

说明:应该将被测单元看做一个被测的整体,根据实际资源、进度和质量风险,权衡代码覆盖、打桩工作量、补充测试用例的难度、被测对象的稳定程度等,一般情况下建议关注模块/组件的测试,尽量避免针对函数的测试。

尽管有时候单个用例只能专注于对某个具体函数的测试,但我们关注的应该是函数的行为而不是其具体实现细节。

## 十五、 可移植性

**规则15.1 不能定义、重定义或取消定义标准库/平台中保留的标识符、宏和函数。**

**建议15.1 不使用与硬件或操作系统关系很大的语句,而使用建议的标准语句,以提高软件的可移植性和可重用性。**

**建议15.2 除非为了满足特殊需求,避免使用嵌入式汇编。**

参考：[编码规范——华为篇](https://blog.csdn.net/qq_33499229/article/details/88677757)

