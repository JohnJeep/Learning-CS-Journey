<!--
 * @Author: JohnJeep
 * @Date: 2025-04-19 17:31:46
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-08-29 23:00:53
 * @Description: Python Usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
-->

# 1. Introduction

Python 是一门动态、解释型语言，运行依赖解释器，运行的平台只要有解释器，就能运行。

解释型语言的特点：

1. 跨平台性好，无需编译、开发调试灵活高效。
2. 执行效率低：每次运行都需要重新解释，做类型检查和动态查找。

# 2. 设计哲学

在 Python 中，变量、函数参数和返回值都没有显式的类型声明，因为 Python
的解释器在运行时才会确定变量的类型。这称为“动态绑定”或“鸭子类型”。

Python 的设计理念核心在于 **动态类型（Dynamic Typing）** 。

原理：

1. Python 的实现方式：Python 的变量实际上是一个指向内存中**对象的引用**。每个 Python
   对象都有一个类型信息（存储在对象的 `__class__` 属性中），但变量本身没有类型，只是指向这些对象的指针。
   当我们给变量赋值时，变量可以指向任意类型的对象。因此，同一个变量可以在程序运行的不同时刻指向不同类型的对象。

   ```python
   >>> x = 42
   >>> type(x)  # 查询对象类型
   <class 'int'>
   >>> x.__class__
   <class 'int'>
   ```

2. 函数定义：在 Python 中，函数定义时不需要指定参数类型和返回值类型。函数可以接受任意类型的参数，只要在函数体内对参数
   的操作是有效的，否则会在运行时抛出异常。
   这种设计使得 Python 函数非常灵活，可以处理多种类型的数据。

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

3. 类型检查：Python 在运行时进行类型检查。例如，当你调用一个对象的方法时，Python
   会检查该对象是否具有该方法，如果没有则抛出异常。

4. 对比 C++：C++是静态类型语言，变量类型在编译时就必须确定，并且不能更改。函数参数和返回值类型必须明确指定，编译器会检
   查类型是否匹配，从而在编译时捕获类型错误。

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
   Python 的动态类型使得代码编写灵活、简洁，但可能会在运行时出现类型错误，且执行效率相对较低（因为需要在运行时进行类型
   判断）。
   C++的静态类型使得编译器可以进行更多的优化，执行效率高，且编译时就能发现类型错误，但代码编写不够灵活，类型系统复杂。

**为什么要这样设计**？

1. **开发效率优先**：减少样板代码
2. **灵活性**：快速原型开发，代码简洁



# 3. 代码风格约定

Python 项目大多都遵循 [**PEP 8**](https://peps.python.org/pep-0008/) 的风格指南；它推行的编码风格易于阅读、赏心悦目。Python 开发者均应抽时间悉心研读；以下是该提案中的核心要点：

- 缩进，用 4 个空格，不要用制表符。

  4 个空格是小缩进（更深嵌套）和大缩进（更易阅读）之间的折中方案。制表符会引起混乱，最好别用。

- 换行，一行不超过 79 个字符。

  这样换行的小屏阅读体验更好，还便于在大屏显示器上并排阅读多个代码文件。

- 用空行分隔函数和类，及函数内较大的代码块。

- 最好把注释放到单独一行。

- 使用文档字符串。

- 常量：约定用**全大写**变量名表示，多个单词之间用下划线分隔。

- 运算符前后、逗号后要用空格，但不要直接在括号内使用： `a = f(1, 2) + g(3, 4)`。

- 类和函数的命名要一致；按惯例，命名类用 `UpperCamelCase`，命名函数与方法用 `lowercase_with_underscores`。命名方法中第一个参数总是用 `self` (类和方法详见 [初探类](https://docs.python.org/zh-cn/3/tutorial/classes.html#tut-firstclasses))。

- 编写用于国际多语环境的代码时，不要用生僻的编码。Python 默认的 UTF-8 或纯 ASCII 可以胜任各种情况。

- 同理，就算多语阅读、维护代码的可能再小，也不要在标识符中使用非 ASCII 字符。

# 4. DataType

## 4.1. str

## 4.2. int

## 4.3. float

## 4.4. bool

## 4.5. NoneType: `None`

## 4.6. Function

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







## 4.7. Built-in Functions

| Built-in Functions                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **A**[`abs()`](https://docs.python.org/3.14/library/functions.html#abs)[`aiter()`](https://docs.python.org/3.14/library/functions.html#aiter)[`all()`](https://docs.python.org/3.14/library/functions.html#all)[`anext()`](https://docs.python.org/3.14/library/functions.html#anext)[`any()`](https://docs.python.org/3.14/library/functions.html#any)[`ascii()`](https://docs.python.org/3.14/library/functions.html#ascii) **B**[`bin()`](https://docs.python.org/3.14/library/functions.html#bin)[`bool()`](https://docs.python.org/3.14/library/functions.html#bool)[`breakpoint()`](https://docs.python.org/3.14/library/functions.html#breakpoint)[`bytearray()`](https://docs.python.org/3.14/library/functions.html#func-bytearray)[`bytes()`](https://docs.python.org/3.14/library/functions.html#func-bytes) **C**[`callable()`](https://docs.python.org/3.14/library/functions.html#callable)[`chr()`](https://docs.python.org/3.14/library/functions.html#chr)[`classmethod()`](https://docs.python.org/3.14/library/functions.html#classmethod)[`compile()`](https://docs.python.org/3.14/library/functions.html#compile)[`complex()`](https://docs.python.org/3.14/library/functions.html#complex) **D**[`delattr()`](https://docs.python.org/3.14/library/functions.html#delattr)[`dict()`](https://docs.python.org/3.14/library/functions.html#func-dict)[`dir()`](https://docs.python.org/3.14/library/functions.html#dir)[`divmod()`](https://docs.python.org/3.14/library/functions.html#divmod) | **E**[`enumerate()`](https://docs.python.org/3.14/library/functions.html#enumerate)[`eval()`](https://docs.python.org/3.14/library/functions.html#eval)[`exec()`](https://docs.python.org/3.14/library/functions.html#exec) **F**[`filter()`](https://docs.python.org/3.14/library/functions.html#filter)[`float()`](https://docs.python.org/3.14/library/functions.html#float)[`format()`](https://docs.python.org/3.14/library/functions.html#format)[`frozenset()`](https://docs.python.org/3.14/library/functions.html#func-frozenset) **G**[`getattr()`](https://docs.python.org/3.14/library/functions.html#getattr)[`globals()`](https://docs.python.org/3.14/library/functions.html#globals) **H**[`hasattr()`](https://docs.python.org/3.14/library/functions.html#hasattr)[`hash()`](https://docs.python.org/3.14/library/functions.html#hash)[`help()`](https://docs.python.org/3.14/library/functions.html#help)[`hex()`](https://docs.python.org/3.14/library/functions.html#hex) **I**[`id()`](https://docs.python.org/3.14/library/functions.html#id)[`input()`](https://docs.python.org/3.14/library/functions.html#input)[`int()`](https://docs.python.org/3.14/library/functions.html#int)[`isinstance()`](https://docs.python.org/3.14/library/functions.html#isinstance)[`issubclass()`](https://docs.python.org/3.14/library/functions.html#issubclass)[`iter()`](https://docs.python.org/3.14/library/functions.html#iter) | **L**[`len()`](https://docs.python.org/3.14/library/functions.html#len)[`list()`](https://docs.python.org/3.14/library/functions.html#func-list)[`locals()`](https://docs.python.org/3.14/library/functions.html#locals) **M**[`map()`](https://docs.python.org/3.14/library/functions.html#map)[`max()`](https://docs.python.org/3.14/library/functions.html#max)[`memoryview()`](https://docs.python.org/3.14/library/functions.html#func-memoryview)[`min()`](https://docs.python.org/3.14/library/functions.html#min) **N**[`next()`](https://docs.python.org/3.14/library/functions.html#next) **O**[`object()`](https://docs.python.org/3.14/library/functions.html#object)[`oct()`](https://docs.python.org/3.14/library/functions.html#oct)[`open()`](https://docs.python.org/3.14/library/functions.html#open)[`ord()`](https://docs.python.org/3.14/library/functions.html#ord) **P**[`pow()`](https://docs.python.org/3.14/library/functions.html#pow)[`print()`](https://docs.python.org/3.14/library/functions.html#print)[`property()`](https://docs.python.org/3.14/library/functions.html#property) | **R**[`range()`](https://docs.python.org/3.14/library/functions.html#func-range)[`repr()`](https://docs.python.org/3.14/library/functions.html#repr)[`reversed()`](https://docs.python.org/3.14/library/functions.html#reversed)[`round()`](https://docs.python.org/3.14/library/functions.html#round) **S**[`set()`](https://docs.python.org/3.14/library/functions.html#func-set)[`setattr()`](https://docs.python.org/3.14/library/functions.html#setattr)[`slice()`](https://docs.python.org/3.14/library/functions.html#slice)[`sorted()`](https://docs.python.org/3.14/library/functions.html#sorted)[`staticmethod()`](https://docs.python.org/3.14/library/functions.html#staticmethod)[`str()`](https://docs.python.org/3.14/library/functions.html#func-str)[`sum()`](https://docs.python.org/3.14/library/functions.html#sum)[`super()`](https://docs.python.org/3.14/library/functions.html#super) **T**[`tuple()`](https://docs.python.org/3.14/library/functions.html#func-tuple)[`type()`](https://docs.python.org/3.14/library/functions.html#type) **V**[`vars()`](https://docs.python.org/3.14/library/functions.html#vars) **Z**[`zip()`](https://docs.python.org/3.14/library/functions.html#zip) **_**[`__import__()`](https://docs.python.org/3.14/library/functions.html#import__) |

Built-in Functions: https://docs.python.org/3.14/library/functions.html

# 5. Data Structures

## 5.1. list

## 5.2. tuple

## 5.3. str

## 5.4. set

## 5.5. dict

在 Python 中，`dict`（字典）是**最常用、最重要的内置数据结构之一**。它是一种**可变的、无序的（Python 3.7+
保持插入顺序）、键值对（key-value）映射**的数据类型。

在 Python 中，`dict`（字典）是一种非常常用且强大的内置数据结构，用于存储**键值对**（key-value
pairs）。字典是**可变的**（mutable）、**无序的**（在 Python 3.7+
中插入顺序被保留，但逻辑上仍视为无序集合），并且**键必须是不可变类型**（如字符串、数字、元组等）。

------

### 5.5.1. 创建字典

```python
# 空字典
d = {}

# 使用花括号创建
d = {'name': 'Alice', 'age': 25, 'city': 'Beijing'}

# 使用 dict() 构造函数
d = dict(name='Alice', age=25, city='Beijing')

# 从键值对列表创建
d = dict()
```

------

### 5.5.2. 访问值

通过键来访问对应的值：

```python
print(d['name'])  # 输出: Alice
```

如果键不存在，会抛出 `KeyError`。可以使用 `.get()` 方法安全访问：

```python
print(d.get('name'))        # Alice
print(d.get('gender'))      # None
print(d.get('gender', 'N/A'))  # N/A（指定默认值）
```

------

### 5.5.3. 修改和添加元素

```python
d['age'] = 26          # 修改已有键的值
d['job'] = 'Engineer'  # 添加新键值对
```

------

### 5.5.4. 删除元素

```python
del d['city']          # 删除键 'city' 及其值
value = d.pop('age')   # 删除并返回该键的值
d.clear()              # 清空整个字典
```

------

### 5.5.5. 常用方法

| 方法                       | 说明                                 |
| -------------------------- | ------------------------------------ |
| `keys()`                   | 返回所有键的视图（类似列表）         |
| `values()`                 | 返回所有值的视图                     |
| `items()`                  | 返回所有 (键, 值) 对的视图           |
| `update(other_dict)`       | 用另一个字典更新当前字典             |
| `setdefault(key, default)` | 如果 key 不存在，设为 default 并返回 |

示例：

```python
for key in d.keys():
  print(key)

for value in d.values():
  print(value)

for key, value in d.items():
  print(f"{key}: {value}")
```

------

### 5.5.6. 字典推导式（Dict Comprehension）

类似列表推导式，可以快速构建字典：

```python
squares = {x: x**2 for x in range(5)}
# 结果: {0: 0, 1: 1, 2: 4, 3: 9, 4: 16}
```

------

### 5.5.7. 注意事项

- **键必须是可哈希的**（hashable）：不能是 list、dict、set 等可变类型。

  ```python
  d = {[1,2]: 'invalid'}  # ❌ 报错：list 不可哈希
  d = {(1,2): 'valid'}    # ✅ 元组可以作为键
  ```

- 从 Python 3.7 起，字典**保持插入顺序**（这是语言规范，不只是实现细节）。

------

### 5.5.8. 示例：综合使用

```python
student = {
  'name': 'Bob',
  'grades': [85, 90, 78],
  'active': True
}

# 安全获取平均分（如果 grades 存在）
grades = student.get('grades', [])
avg = sum(grades) / len(grades) if grades else 0
print(f"Average grade: {avg:.2f}")
```

------

如果你有具体使用场景（比如 JSON 解析、计数、缓存等），也可以告诉我，我可以给出更针对性的例子！



# 6. Compound statements

## 6.1. with

在 Python 中，`with` 语句用于**上下文管理（Context Management）**，它提供了一种简洁、安全的方式来处理需要**设置和清理*
*的资源操作（比如文件、网络连接、锁等）。其核心优势是：**无论代码块中是否发
生异常，都能确保资源被正确释放**。

------

### 6.1.1. 基本语法

```python
with context_manager as variable:
  # 在此代码块中使用 variable
```

其中 `context_manager` 是一个**上下文管理器对象**，它必须实现两个特殊方法：

- `__enter__(self)`：进入 `with` 代码块时调用，通常用于获取资源。
- `__exit__(self, exc_type, exc_val, exc_tb)`：退出 `with` 代码块时调用，用于释放资源。即使发生异常也会被调用。

------

### 6.1.2. 最常见的例子：文件操作

#### 6.1.2.1. 不使用 `with`（不推荐）

```python
f = open('file.txt', 'r')
data = f.read()
f.close()  # 如果中间出错，可能不会执行到这行！
```

#### 6.1.2.2. 使用 `with`（推荐）

```python
with open('file.txt', 'r') as f:
  data = f.read()
# 文件会自动关闭，即使读取过程中发生异常
```

这里 `open()` 返回的是一个**文件对象**，它本身就是一个上下文管理器，实现了 `__enter__` 和 `__exit__` 方法。

------

### 6.1.3. 自定义上下文管理器

你可以通过类或装饰器来创建自己的上下文管理器。

**方法一：使用类**

```python
class MyContext:
  def __enter__(self):
    print("进入上下文")
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    print("退出上下文")
    # 返回 True 可以抑制异常（一般不建议）
    return False

with MyContext() as mc:
  print("在 with 块中")
```

输出：

```txt
进入上下文
在 with 块中
退出上下文
```

**方法二：使用 `contextlib.contextmanager` 装饰器**

```python
from contextlib import contextmanager

@contextmanager
def my_context():
  print("进入")
  try:
    yield "some resource"
  finally:
    print("退出")

with my_context() as res:
  print(f"使用 {res}")
```

输出：

```
进入
使用 some resource
退出
```

------

### 6.1.4. 实际应用场景

- 文件读写（最常见）
- 数据库连接（自动提交/回滚/关闭）
- 线程锁（`with lock:` 自动加锁/解锁）
- 临时修改环境变量或配置
- 测试中模拟（mock）对象

✅ **优点**：

- 代码更简洁
- 避免资源泄漏
- 异常安全

📌 **记住**：只要一个对象支持上下文管理协议（即有 `__enter__` 和 `__exit__`），就可以用在 `with` 语句中。

# 7. Exception

Exception 是 Python 里处理"错误/异常"的核心语法。

## 7.1. 为什么需要 try except

写代码时，有些错误是**运行时才会发生**的，比如：

```python
num = int(input("请输入一个数字："))   # 如果用户输入的是"abc"，这里就会报错崩溃
```

如果不处理，程序会直接**崩溃退出**，报错信息类似：

```
ValueError: invalid literal for int() with base 10: 'abc'
```

`try except` 的作用就是：**"先试着跑一下这段代码，如果出错了，不要崩溃，按我说的方式处理"**

## 7.2. 基本语法结构

```python
try:
    # 可能会出错的代码，放这里"试"一下
    可能出错的代码
except:
    # 如果上面出错了，就跑这里的代码
    出错时执行的代码
```

- `try`：把可能出错的代码"圈起来"试跑
- `except`：出错了怎么办（可以针对不同错误类型分别处理）
- `else`：**没出错**时才执行（可选）
- `finally`：**不管有没有出错都会执行**，通常用来做收尾/清理工作（比如关闭文件、串口、网络连接）

**最简单的例子：**

```python
try:
    num = int(input("请输入一个数字："))
    print(f"你输入的数字是: {num}")
except:
    print("输入错误，这不是一个有效的数字！")
```

- 如果用户输入 `5`：正常走 `try` 里的代码，打印"你输入的数字是: 5"
- 如果用户输入 `abc`：`try` 里的代码执行到 `int("abc")` 时**炸了**，Python立刻跳到 `except`，打印"输入错误..."，**程序不会崩溃，会继续往下走**

## 7.3. 捕获具体的错误类型（推荐做法）

上面写的 `except:`（不带任何类型）会捕获**所有**类型的错误，这其实是个坏习惯，因为你**分不清到底是哪里错了**。更好的写法是**指定具体的异常类型**：

```python
try:
    num = int(input("请输入一个数字："))
    result = 10 / num
except ValueError:
    print("你输入的不是数字！")
except ZeroDivisionError:
    print("不能除以0！")
```

- 如果输入 `abc` → 触发 `ValueError`，走第一个except
- 如果输入 `0` → 触发 `ZeroDivisionError`，走第二个except
- 每种错误对应各自的处理方式，**更精确、更好排查问题**

### 7.3.1. 常见的异常类型

| 异常类型            | 什么时候触发                                |
| ------------------- | ------------------------------------------- |
| `ValueError`        | 值的类型对，但内容不合法，比如 `int("abc")` |
| `TypeError`         | 类型不匹配，比如字符串和数字相加 `"a" + 1`  |
| `ZeroDivisionError` | 除以0                                       |
| `IndexError`        | 列表下标越界，比如 `[1,2,3][10]`            |
| `KeyError`          | 字典里没有这个key，比如 `{"a":1}["b"]`      |
| `FileNotFoundError` | 打开一个不存在的文件                        |
| `AttributeError`    | 调用了对象不存在的属性/方法                 |

## 7.4. 拿到错误的具体信息：`as e`

```python
try:
    num = int("abc")
except ValueError as e:
    print(f"出错了，原因是: {e}")
```

输出类似：

```
出错了，原因是: invalid literal for int() with base 10: 'abc'
```

`e` 就是Python给你的这次错误的"详细说明"，方便你打印日志、排查问题。`e`这个名字可以随便起，但约定俗成写`e`（error的缩写）。

## 7.5. `else`：没出错的时候才执行

```python
try:
    num = int(input("请输入一个数字："))
except ValueError:
    print("输入错误！")
else:
    print(f"输入成功，数字是: {num}")   # 只有try里完全没出错，才会走这里
```

`else` 不是必须的，加它的意义是：**把"正常情况下要做的事"和"try里为了防止出错而放进去的代码"分开**，可读性更好。

## 7.6. `finally`：不管有没有出错，最后都会执行

这是你问的重点，我详细讲：

```python
try:
    print("尝试执行")
    num = int("abc")     # 这里会报错
except ValueError:
    print("捕获到错误")
finally:
    print("不管有没有出错，我都会执行")
```

输出：

```
尝试执行
捕获到错误
不管有没有出错，我都会执行
```

`finally` 里的代码**无论如何都会跑一遍**，不管：

- try里的代码顺利执行完了（没出错）
- try里的代码出错了，并且被except成功捕获处理了
- 甚至try里出错了，但是**没有对应的except能处理这个错误**（程序即将崩溃退出前），`finally`依然会执行，执行完之后程序才真正崩溃

**`finally` 最典型的用途：释放资源**，比如关闭文件、断开网络连接、释放硬件占用——这些"收尾工作"不管程序是正常结束还是出错，都必须做，不然会造成资源泄漏。

## 7.7. 完整结构总览（顺序固定，不能乱）

```python
try:
    可能出错的代码
except 异常类型1 as e1:
    处理异常1
except 异常类型2 as e2:
    处理异常2
else:
    没有出错时，额外执行的代码
finally:
    不管有没有出错，最后都会执行的代码
```

顺序必须是：`try` → 若干个`except`（可以0个、1个或多个）→ `else`（可选）→ `finally`（可选）

## 7.8.

## 7.9. 常见的实用技巧

**1. 一个except同时捕获多种类型：**

```python
except (ValueError, TypeError) as e:
    print(f"值或类型错误: {e}")
```

**2. 主动抛出自己的错误（raise）：**

```python
def set_speed(speed):
    if speed > 100:
        raise ValueError("速度不能超过100")   # 主动报错，让调用者用try捕获
```

**3. 捕获所有异常但仍打印详细信息（调试时常用）：**

```python
import traceback
try:
    do_something()
except Exception as e:
    print(f"发生错误: {e}")
    traceback.print_exc()   # 打印完整的错误堆栈，方便排查是哪一行出的问题
```



# 8. Class

💡 装饰器常用于：日志记录、权限检查、缓存、计时、重试机制等。



# 9. Packages

package：一个包就是一个文件夹，里面装了很多模块（`.py`文件），文件夹里通常有个 `__init__.py` 文件表明"这是个包"。默认情况下，只有 `__init__.py` 里写了的东西，才会在 `import 包名` 之后能直接用 `包名.xxx` 点出来。

module：一个模块就是一个单独的 `.py` 文件；

**用文件结构类比一下**

```
rclpy/                  <- 这是一个包（文件夹）
├── __init__.py          <- import rclpy 时，只执行这个文件
├── node.py               <- 定义了 class Node，但不会被自动加载！
├── time.py               <- 定义了 class Time，也不会被自动加载！
└── ...
```

当你写 `import rclpy` 时：

- Python 只执行 `rclpy/__init__.py` 这一个文件
- 如果 `__init__.py` 里**没有**写 `from . import node`，那么 `node.py` 这个文件根本**不会被读取、不会被执行**
- 所以你此时写 `rclpy.node.Node` 会报错：`AttributeError: module 'rclpy' has no attribute 'node'`

而你写 `from rclpy.node import Node` 的时候，Python会：

1. 专门去打开 `rclpy/node.py` 这个文件，把它完整执行一遍（这一步之前没做过）
2. 从执行完的结果里，把 `Node` 这个类抠出来给你用；

**注意点：**

- `import rclpy` 只加载了这个包的"总入口文件"(`__init__.py`)，并不会自动把所有子模块（`node.py`、`time.py`……）都加载并挂到 `rclpy` 上。想用某个子模块里的东西，必须**专门再写一行**去导入那个子模块（或其中的具体类），这不是重复劳动，而是"顶层功能"和"子模块具体功能"本来就是两码事，要分别导入。

- `import xxx` 和 `from xxx import yyy` 在"底层加载多少东西"上没有任何区别，两者都会完整加载整个模块。唯一的区别是：前者要求你用的时候写前缀（如 `xxx.功能`），后者让你直接用抠出来的那个名字，不用写前缀。这纯粹是**代码可读性/书写便利性**的选择，跟性能、体积无关。
- 包可以嵌套：包里面还可以有包；



# 10. References

1. offical: https://www.python.org/
1. Python Package Index: https://pypi.org/
1. exception: https://docs.python.org/3/library/exceptions.html

