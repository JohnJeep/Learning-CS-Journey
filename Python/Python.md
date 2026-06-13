<!--
 * @Author: JohnJeep
 * @Date: 2025-04-19 17:31:46
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 18:51:10
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

| Built-in Functions                                           |                                                              |                                                              |                                                              |
| :----------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
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



# 7. Class



💡 装饰器常用于：日志记录、权限检查、缓存、计时、重试机制等。

# 8. References

1. offical: https://www.python.org/
1. Python Package Index: https://pypi.org/

