## Python

## Introduction

Python 是一门动态、解释型语言，运行依赖解释器，运行的平台只要有解释器，就能运行。

解释型语言的特点：

1. 跨平台性好，无需编译、开发调试灵活高效。
2. 执行效率低：每次运行都需要重新解释，做类型检查和动态查找。

## 设计哲学

在Python中，变量、函数参数和返回值都没有显式的类型声明，因为Python的解释器在运行时才会确定变量的类型。这称为“动态绑定”或“鸭子类型”。

Python 的设计理念核心在于 **动态类型（Dynamic Typing）** 。

原理：

1. Python的实现方式：Python的变量实际上是一个指向内存中**对象的引用**。每个Python对象都有一个类型信息（存储在对象的 `__class__` 属性中），但变量本身没有类型，只是指向这些对象的指针。
   当我们给变量赋值时，变量可以指向任意类型的对象。因此，同一个变量可以在程序运行的不同时刻指向不同类型的对象。

   ```python
   >>> x = 42
   >>> type(x)  # 查询对象类型
   <class 'int'>
   >>> x.__class__
   <class 'int'>
   ```

2. 函数定义：在Python中，函数定义时不需要指定参数类型和返回值类型。函数可以接受任意类型的参数，只要在函数体内对参数的操作是有效的，否则会在运行时抛出异常。
   这种设计使得Python函数非常灵活，可以处理多种类型的数据。

   ```python
   # 运行时确定类型
   x = 10          # 现在是 int
   x = "hello"     # ✅ 现在是 str，完全合法
   
   def add(a, b):  # 参数可以是任何类型
       return a + b  # 只要支持 + 操作
   
   add(1, 2)       # ✅ 返回 3
   add("a", "b")   # ✅ 返回 "ab"
   add([1], [2])   # ✅ 返回 [1, 2]
   ```

3. 类型检查：Python在运行时进行类型检查。例如，当你调用一个对象的方法时，Python会检查该对象是否具有该方法，如果没有则抛出异常。

4. 对比C++：C++是静态类型语言，变量类型在编译时就必须确定，并且不能更改。函数参数和返回值类型必须明确指定，编译器会检查类型是否匹配，从而在编译时捕获类型错误。

   ```Python
   # Python：这个函数可以用于整数、浮点数、字符串等，只要这些类型支持“+”操作
   def add(a, b):
   return a + b
   
   
   # C++：这个函数只能用于整数，如果传入浮点数，会被截断为整数（除非重载）
   int add(int a, int b) {
   return a + b;
   }
   ```

5. 优缺点：
   Python的动态类型使得代码编写灵活、简洁，但可能会在运行时出现类型错误，且执行效率相对较低（因为需要在运行时进行类型判断）。
   C++的静态类型使得编译器可以进行更多的优化，执行效率高，且编译时就能发现类型错误，但代码编写不够灵活，类型系统复杂。

**为什么要这样设计**？

1. **开发效率优先**：减少样板代码
2. **灵活性**：快速原型开发，代码简洁



## 代码风格约定

常量：约定用全大写变量名表示，多个单词之间用下划线分隔。



## DataType

- str
- int
- float
- bool
- NoneType: `None`





## Function

**位置参数**：调用函数时，根据参数在函数定义中出现的顺序，把实参的值一次传递给对应的形参。

```python
# Positional arguments
def introduce(name, age):
  print(f"My name is {name} and I am {age} years old.")

introduce("Alice", 30)
```

**关键字参数**：函数调用时，通过 `形参名=value` 的形式传递参数。

```python
# keyword-only arguments
def display_info(name, age):
  print(f"Name: {name}, Age : {age}")

display_info(age=25, name="John")
```

**限制传参方式**

规则：

1. `/` 前面只能用位置参数，`*`后面只能用关键参数。
2. `/` 和 `*`，同时使用时，`/` 必须在 `*` 前面。

```python
# Positional-only and keyword-only arguments
def student(name, /, age, *, grade):
  ''' This function displays student information.''' # 函数说明文档
  print(f"Name: {name}, Age: {age}, Grade: {grade}")

student("Alice", 20, grade=100)
```



**默认参数**：必须要放在必选参数的后面。即某个形参，一旦设置了默认值，那么它后面的所有形参都必须要给默认值。

```python
# Default parameter values
def greet(date, greeting="Hello", name="World"):
  print(f"{date}, {greeting}, {name}!")

greet("Mon", name="Bob")
```

原理：print 函数底层给 end 函数设置了默认值 `\n` 。

```python
print("hello word", end="!!!")
print("+++++++++++++++++++++++++++++++")

# 输出为：hello word!!!+++++++++++++++++++++++++++++++
```



函数可以动态添加类型

```python
def greet(date, greeting="Hello", name="World"):
  print(f"{date}, {greeting}, {name}!")

greet.desc = "this is a greeting description"
print(greet.desc)
```







## Buildin-function

1. type



## Data Structures

1. list(列表)
2. tuple(元组)
3. str(字符串)
4. set(集合)
5. dict(字典)



## Class





## References

1. offical: https://www.python.org/

