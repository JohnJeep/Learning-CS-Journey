<!--
 * @Author: JohnJeep
 * @Date: 2021-01-10 18:25:09
 * @LastEditTime: 2021-02-07 15:58:54
 * @LastEditors: Please set LastEditors
 * @Description: 剖析C++标准库
-->

<!-- TOC -->

- [1. Standard Template Library](#1-standard-template-library)
  - [1.1. Thinking(思考)](#11-thinking思考)
  - [1.2. History(历史)](#12-history历史)
  - [1.3. STL（Standard Template Library）标准模板库](#13-stlstandard-template-library标准模板库)
  - [1.4. Container(容器)](#14-container容器)
    - [1.4.1. Sequence containers(有序容器)](#141-sequence-containers有序容器)
      - [1.4.1.1. Array](#1411-array)
      - [1.4.1.2. vector（单端的动态数组）](#1412-vector单端的动态数组)
      - [1.4.1.3. deque（双端数组）](#1413-deque双端数组)
      - [1.4.1.4. list（双向链表）](#1414-list双向链表)
      - [1.4.1.5. forword list(单向链表)](#1415-forword-list单向链表)
    - [1.4.2. Associative containers(关联性容器)](#142-associative-containers关联性容器)
      - [1.4.2.1. set](#1421-set)
      - [1.4.2.2. map](#1422-map)
      - [1.4.2.3. multiset](#1423-multiset)
      - [1.4.2.4. mutimap](#1424-mutimap)
    - [1.4.3. Unordered associative containers(无序关联容器)](#143-unordered-associative-containers无序关联容器)
      - [1.4.3.1. unordered_set](#1431-unordered_set)
      - [1.4.3.2. unordered_map](#1432-unordered_map)
      - [1.4.3.3. unordered_multisert](#1433-unordered_multisert)
      - [1.4.3.4. unordered_multimap](#1434-unordered_multimap)
    - [1.4.4. Container adaptors(容器适配器)](#144-container-adaptors容器适配器)
      - [1.4.4.1. stack](#1441-stack)
      - [1.4.4.2. queue](#1442-queue)
      - [1.4.4.3. priority_queue(优先级队列)](#1443-priority_queue优先级队列)
  - [1.5. Algorithm](#15-algorithm)
    - [1.5.1. heap](#151-heap)
    - [1.5.2. 其它函数](#152-其它函数)
  - [1.6. Adaptor(适配器)](#16-adaptor适配器)
  - [1.7. Functor(仿函数)](#17-functor仿函数)
  - [1.8. Iterator(迭代器)](#18-iterator迭代器)
  - [1.9. Allocator(分配器)](#19-allocator分配器)

<!-- /TOC -->

# 1. Standard Template Library
## 1.1. Thinking(思考)

---------------------------
使用它是一件很愉快的事。

使用一个东西，却不明白它的道理，不高明！---林语堂

源码之前了无秘密。

天下大事，必作于细。

高屋建瓴，细致入微。

所谓剖析源码，其目的在于明理、解惑，提高自身水平，并不是要穷经皓首，倒背如流。

STL学习境界：会用，明理，能扩展。



---------------------------

目标

Level 0: 使用C++标准库

Level 1: 深入认识C++标准库(胸中自有丘壑)

Level 2: 良好使用C++标准库

Level 3: 扩充C++标准库

---------------------------

## 1.2. History(历史)
STL创始人：Alexander Stepanov



## 1.3. STL（Standard Template Library）标准模板库
- Generic Programming(泛型编程): 就是利用模板为主要工具来进行编写程序。STL是泛型编程(GP)最成功的一个作品。

- STL所实现的是依据泛型思维架设起来的概念结构。STL的核心思想：算法和数据结构的实现是分离的。


- STL六大部件
  - 算法（Algorithm）
  - 容器（Container）
  - 迭代器（Iterator）
  - 仿函数（Functor）
  - 适配器（Adaptor）
  - 分配器（Alloctor） 

- 六大部件之间的关系
  > Container 通过 Allocator 取得数据存储空间，Algorithm 通过 Iterator 存取 Container 内容，Functor 可以协助 Algorithm 完成不同的策略变化，Adaptor 可以修饰或套接 Functor。
  <img src="./figures/stl-component.png">



- 使用六大部件的例子
  <img src="./figures/stl-component-example.png">
  

- 常用的容器
  - Vector(向量)
  - Deque(双队列)
  - List(链表)
  - Map/Multimap(映射/多重映射)
  - Set/Multiset(集合/多重集合)
<img src="./figures/container-library.png">



## 1.4. Container(容器)
- 容器是一个一个的模板类，里面放的是元素。

- STL标准库中 `容器` 内存储的元素都必须能够拷贝，而C++编译器默认提供的是浅拷贝，程序在执行时，则会出现问题。因此需要 `重写构造函数` 和重载 `=` 操作运算符，执行深拷贝。



### 1.4.1. Sequence containers(有序容器)

#### 1.4.1.1. Array
- fixed number of elements(固定数量的元素)

- 内部结构图
  
  <img src="./figures/container-arrays.png">


- 不能扩容。


#### 1.4.1.2. vector（单端的动态数组）
- 动态数组实现机制：
  > 先为数组开辟较小的空间，然后往数组里面添加数据，当数据的数目超过数组的容量时，再重新分配一块更大的空间（STL中 `vector` 每次扩容时，新的容量都是前一次的两倍），再把之前的数据复制到新的数组中，再把之前的内存释放。
  - 注意：使用动态数组时，尽量减少改变数组容量大小的次数，可以减少时间性能的消耗。
- 动态数组，在运行阶段执行。`vector` 支持随机迭代访问器。
- 向容器中插入元素时，内部的元素必须能够执行 `拷贝（必须提供拷贝构造）` 操作。

- 内部结构图

  <img src="./figures/container-vectors.png">

- 一般每次扩容为原来的  2 倍。


- 模板：`template <typename T> void Show(T arrNum[], int len);`
- 成员函数
  - `size()`: 返回容器中元素的个数
  - `get(r)`: 获取秩（索引）为r的元素
  - `put(r, e)`: 用e替换秩为r元素的数值
  - `insert(r, e)`: 向秩为r的元素处插入数值e，后面元素依次后移
  - `remove(r)`: 初除秩为 `r` 的元素，返回全元素中原存放的对象
  - `disordered()`: 判断所有元素是否已按升序序排列
  - `sort()`: 调整各元素癿位置，使按照升序序排列
  - `deduplicate()`: 删除重复元素   ---向量
  - `uniquify()`: 删除重复元素 ---有序向量
  - `traverse()`: 遍历向量幵统一处理所有元素，处理斱法由函数对象指定
  - `empty()`: 判断容器是否为空
  - `at(index)`: 返回索引为index的元素
  - `erase(p)`: 删除p位置处的元素
  - `erase(beg, end)`:删除区间`[beg, end)`的数据
  - `pop_back()`: 删除最后一个元素
  - `push_back()`: 在容器末尾插入一个元素
  - `back()`: 获取尾部元素
  - `front()`: 获取首部元素
  - `begin(), end()`: 返回容器首尾元素的迭代器
  - `clear()`: 移除容器中所有的元素
  - `swap()`: 交换两个容器的内容，交换两个 vector 的内容后，两者的容量也交换了，这是一个间接缩短vector的小窍门。
  - `shrink_to_fit()`: 缩短vector的大小到合适的空间，为实现特定的优化保留了回旋的余地。


#### 1.4.1.3. deque（双端数组）
- 与 `vector` 容器类似，但是可以在 `Deque` 的两端进行操作。
- `push_back()`: 在容器末尾插入一个元素
- `push_front()` 容器头部插入一个元素
- `pop_front()`: 容器头部删除一个元素
- `pop_back()`: 删除最后一个元素

- 内部结构图
  
  <img src="./figures/container-deques.png">
  <img src="./figures/container-deques-internal-structure.png">
  <img src="./figures/deque.png">

- 每次扩容的大小为一个 buffer。


#### 1.4.1.4. list（双向链表）
- 是一个双向链表的容器，可以高效的进行插入和删除元素。
- 不支持随机存储元素，即不支持 `at.(pos)` 函数和 `[]` 操作符。
- 链表的插入操作：在 pos 位置插入新的节点，新插入的数据存放在 pos 位置之前。
- list 的删除
  - `clear()` 移除容器中所有的数据
  - `erase(begin, end)` 删除区间 `[begin, end)` 的数据，返回下一个元素的位置。
  - `erase(pos)` 删除指定 pos 位置的数据，返回下一个元素的位置。
  - `remove(element)` 删除容器中所有与 element 值匹配的数据。 

- 内部结构图

  <img src="./figures/container-lists.png">
  <img src= "./figures/container-lists-internal-structure.png">

- 每次只能扩充一个结点，效率低，但空间浪费是最小的。

#### 1.4.1.5. forword list(单向链表)
- forword list链表是C++11新加的功能，比list的效率要快，是单向的链表。

- 内部结构图  
  <img src="./figures/container-forward-lists.png">

- 只能扩充一个结点。

### 1.4.2. Associative containers(关联性容器)
关联式容器并不提供元素的直接访问，需要依靠迭代器访问。map 是个例外，提供了subscript(下标)操作符，支持元素的直接访问。


#### 1.4.2.1. set
- set 是一个 `集合` 容器，包含的元素是唯一的，集合中的元素按照一定的顺序排列，不能随意指定位置插入元素。
- set 底层采用红黑树的数据结构实现的。
- set 支持唯一的键值，容器里面的元素值只能出现一次，而 `multiset` 集合容器中同一个元素值可以出现多次。
- 不可以直接修改 set和multiset集合容器中元素的值，因为集合容器是自动排序的。修改集合容器中元素的值，必须先删除原先元素的值，再插入新元素的值。

- 内部结构图  
  <img src="./figures/set-multiset.png">
  <img src="./figures/set-multiset-internal-structure.png">


- `insert()` 函数的返回值类型为 `pair<iterator, bool>`，结果是一对数据类型。
  ```
  pair<T1, T2> 存放两个不同类型的数值
  ```

- set 容器中的查找
  - `find()` 返回查找元素的迭代器，查找的元素默认是区分大小写的。
  - `count()` 返回容器中查找元素的个数
  - `upper_bound` 返回容器中大于查找元素的迭代器位置
  - `lower_bound` 返回容器中小于查找元素的迭代器位置
  - `equal_range(ele)`返回容器中等于查找元素的两个上下限的迭代器位置（第一个：大于等于ele元素的位置，第二个：大于 ele元素的位置）



#### 1.4.2.2. map
- map 是关联式容器，一个 map 就是一个键值对。map 中的 `key` 值唯一，容器中的元素按照一定的顺序排列，不能在任意指定的位置插入元素。
- map 的底层原理是按照平衡二叉树的数据结构来实现的，在插入和删除的操作上比 `vector` 容器快。
- map 支持唯一的键值，每个 `key` 只能出现一次，支持 `[]` 操作，形如：`map[key] = value`。 `multimap` 不支持唯一的键值，容器中的每个 `key` 可以出现相同的多次，不支持 `[]` 操作。

- 内部结构图  
  <img src="./figures/map-multimap.png">
  <img src="./figures/map-multimap-internal-structure.png">


```C++
  // 四种map容器的插入方法
  map<int, string> mp;
  mp.insert(pair<int, string>(101, "赵云"));                   // 法一
  mp.insert(make_pair<int, string>(102, "关羽"));              // 法二
  mp.insert(map<int, string>::value_type(103, "曹操"));        // 发三
  mp[104] = "张飞";                                            // 法四

  // 方法一到方法三向容器中插入相同的键值时，不会插入成功。
  // 采用法四向容器中插入相同的键值时，会覆盖原先相同键值的数据。

```
- map的查找操作需要做异常判断处理

- `at(key)`
  -  `at()` 函数会根据它收到的 `key` 得到元素的 `value`，如果不存在这样的元素，则抛出 `out_of_range` 异常。

- `operator []`
  - `operator []` 的索引就是 `key`，其类型可能属于任何的类型，不一定是整数。
  - 如果你选择某 `key` 作为索引，容器内没有相应的元素，那么 map 会自动安插一个新元素，其 value 将被其类型的 default 构造函数初始化。因此你不可以指定一个 `不具 default 构造函数的 value 类型`。一般基础类型都有一个 `default 构造函数` ,设初值为 0。

#### 1.4.2.3. multiset
collection of keys, sorted by keys.容器中的 key 可以重复。


#### 1.4.2.4. mutimap
collection of key-value pairs, sorted by keys


### 1.4.3. Unordered associative containers(无序关联容器)
#### 1.4.3.1. unordered_set
- `unordered_set` 是一种无序的容器集合。底层采用哈希表实现的。
- STL无序容器存储状态，hash表存储结构图
<img src="./figures/unordered-containers.png">
<img src="./figures/unordered-sets-multisets-internal-structure.png">
  

- `unordered_set` 模板类中的定义
  ```C++
  template<typename _Value,                        // 容器中存储元素的类型
          typename _Hash = hash<_Value>,           // 确定元素存储位置的哈希函数
          typename _Pred = std::equal_to<_Value>,  // 判断各个元素是否相等
          typename _Alloc = std::allocator<_Value>, // 指定分配器对象的类型
          typename _Tr = __uset_traits<__cache_default<_Value, _Hash>::value>>
  ```

- 注意：此容器模板类中没有重载 `[ ]` 运算符，也没有提供 `at()` 成员方法，`unordered_set` 容器内部存储的元素值不能被修改，可以使用迭代器遍历容器中的数，但不能修改容器中元素的值。


- hash table 为了解决冲突采用 `separate chaining` 的方式。



#### 1.4.3.2. unordered_map
- 内部结构图
  
  <img src="./figures/unordered-maps-multimsps-internal-structure.png">

#### 1.4.3.3. unordered_multisert


#### 1.4.3.4. unordered_multimap


### 1.4.4. Container adaptors(容器适配器)
容器适配器为有序的容器提供了不同的接口。queue和stack底层完全借助 deque实现的。


#### 1.4.4.1. stack
<img src="./figures/stack.png">


- `push()` 入栈
- `pop()` 出栈
- `top()` 获取栈顶元素
- `size()` 获取栈大小
- `empty()` 栈为空


#### 1.4.4.2. queue
<img src="./figures/queue.png">

- `push()` 入队列
- `pop()` 出队列
- `empty()` 对列为空
- `front()` 队列头部元素
  


#### 1.4.4.3. priority_queue(优先级队列)
```C++
// 最大或最小优先级队列变量的声明 
priority_queue<int> g_priq;                            // 默认为最大值优先队列
priority_queue<int, vector<int>, greater<int>> l_priq; // 最小值优先队列
```


## 1.5. Algorithm
- 从实现的角度来看，STL算法是一种 `function template`。

- STL算法的核心思想
  - STL通过类模板技术，实现了数据类型与容器模型的分离。
  - 通过函数对象实现了自定义数据类型与底层算法的分离。
  - 通过迭代器的方式统一的去遍历容器，向容器中读数据和写数据。


### 1.5.1. heap
堆（heap）的STL库中函数

- `make_heap(first, last, comp);` 建立一个空堆；
- `push_heap(first, last, comp);` 向堆中插入一个新元素；
- `top_heap(first, last, comp); ` 获取当前堆顶元素的值；
- `sort_heap(first, last, comp);` 对当前堆进行排序；



### 1.5.2. 其它函数
- `for_each()` 遍历容器中的所有元素。
- `transform()` 将容器中的数据进行某种转换的运算。
  - 两个算法的区别
    - `for_each()` 使用的函数对象可以没有 `返回值`，参数一般传 `reference`，因此速度较快，不是很灵活。
    - `transform()` 使用的函数对象必须要有 `返回值`，参数一般传 `value`，因此速度较慢，但是很灵活。
- `adjacent()` 查找一对相邻位置重复的元素，找到则返回指向这对元素的第一个元素的迭代器值。
- `distance()` 迭代器下标的位置。
- `binary_search()` 采用二分法在有序序列中查找 value，找到则返回 true。在无序的序列中不能使用。
- `count()` 计数容器中指定元素的个数。
- `count_if()` 使用 `谓词` 计数容器中指定条件元素的个数。
- `find()` 
- `find_if()` 
- `merge()`  合并两个有序的序列，并存放到另一个序列中。
- `sort()` 默认按照升序的方式重新排列指定范围内元素的元素。
- `random_shuffle()` 对指定范围内的元素随机进行排序。
- `reverse()` 对指定范围内的元素进行倒叙排序。
- `copy()` 将一个容器中的元素值拷贝到另一个容器中
- `replace()` 将指定范围内的 `oldValue` 替换为 `newValue`
- `replace_if()` 将指定范围内的 `oldValue` 替换为 `newValue`，需要指定 `函数对象`（是自定义的函数对象或STL预定义的函数对象）。
- `swap()`  交换两个容器
- `accumulate()` 累加遍历容器中指定范围内的元素，并在结果上加一个指定的值。
- `stable_partition()`函数
- `upper_bound()` 函数
- `lower_bound()` 函数
- `std::floor` 和 `std::ceil`都是对变量进行四舍五入，只不过四舍五入的方向不同。 
  - `std::floor` -->向下取整数。`5.88   std::floor(5.88) = 5;`
  - `std::ceil ` -->向上取整数。`std::ceil(5.88)   = 6;`


     

## 1.6. Adaptor(适配器)
- 什么是适配器？
  > 一种用来修饰容器(containers)或仿函数(functor)或迭代器(iterators)接口的东西。改变 `functor` 接口者，称为 `function adaptor`；改变 `container` 接口者，称为 `container adaptor`；改变 `iterator` 接口者，称为 `iterator adaptor`。


- 函数适配器
  - bind adaptor(绑定适配器)
  - composite adaptor(组合适配器)
  - pointer adaptor(指针适配器)
  - member function adaptor(成员函数适配器) 

predicate: 判断这个条件是真还是假


## 1.7. Functor(仿函数)
- 什么是仿函数？
  > 仿函数(Functor)也叫函数对象(Function object)或者叫伪函数。它是在 `struct` 结构体中定义一种新的函数。从实现的角度看，仿函数是一种重载了 `operator()` 的 `class` 或 `class template`。一般函数指针可视为狭义的仿函数。 

- 分类
  - 预定义函数对象：标准STL模板库中提前预定义了很多的函数对象。
  - 用户自定义的函数对象 

- Function object(函数对象): 需要重载 `()` 操作运算符。函数对象的调用与 `回调函数` 的调用类似。

- 函数对象调用
  - 函数对象可以做函数参数。 
  - 函数对象可以做返回值。 
  ```C++
  class Stu
  {
    private:
    public:
      void operator() (Stu& T) {}
  }
  ``` 



## 1.8. Iterator(迭代器)
- 迭代器就是一种泛化的指针。从实现的角度看，迭代器是一种将 `operator*`, `operator->`, `operator++`, `operator--` 等指针操作给予重载的 `class template`。

- 前闭后开区间
  - begin: 指向容器中的第一个元素的位置。
  - end: 指向容器中最后一个元素的下一个位置。
<img src="./figures/begin-end.png">




## 1.9. Allocator(分配器)
- 什么是Allocator？
  > 负责内存空间的分配与管理。分配器是一个实现了动态空间配置、空间管理、空间释放的 `class template`。
  

