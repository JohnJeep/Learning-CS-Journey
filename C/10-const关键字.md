<!--
 * @Author: your name
 * @Date: 2019-10-21 19:00:34
 * @LastEditTime: 2020-06-02 20:53:40
 * @LastEditors: Please set LastEditors
 * @Description: C语言const关键字
--> 

- C90新增两个限定符：`const` ` volatile`
- C99新增一个限定符：`restrict` 
-  C11新增一个限定符：`_Atmoic`
- C语言中， 用const类型限定符声明的是变量， 不是常量
- 用于限定一个变量为只读,数值不能通过赋值或递增、 递减来修改


## 例子
- `const float* pf;`  pf 指向一个float类型的const值 
  - pf指向的值不能改变，指针本身可以改变
- `float* const pt;`   pt 是一个const指针
  - pt本身的值不能改变，但它所指向的值可以改变
- `const float* const ptr;` 表明ptr既不能指向别处， 它所指向的值也不能改变。
   >  const放在`*`左侧任意位置，限定了指针指向的数据不能改变；const放在`*`的右侧， 限定了指针本身不能改变 


## 什么时候用？
- 如果一个指针仅用于给函数访问值， 应将其声明为一个指向const限定类型的指针。 
- 如果要用指针更改主调函数中的数据， 就不使用const关键字。
- 用const声明全局变量，可以创建const变量、 const数组和const结构
- 多个文件间共享const数据时遵循两个规则：
  - 在一个文件中使用定义式声明，
  - 在其他文件中使用引用式声明（用extern关键字），即只声明不能赋值


<font color="red"> 注意：</font> 在C语言中，const定义的值可以采用指针间接赋值的方法，改变指针本身const定义的值

