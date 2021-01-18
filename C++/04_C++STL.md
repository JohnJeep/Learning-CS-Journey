<!--
 * @Author: JohnJeep
 * @Date: 2021-01-10 18:25:09
 * @LastEditTime: 2021-01-10 18:25:51
 * @LastEditors: Please set LastEditors
 * @Description: 剖析C++标准库
-->

<!-- TOC -->

- [1. Standard Template Library](#1-standard-template-library)
  - [1.1. 参考](#11-参考)
  - [1.2. STL（Standard Template Library）标准模板库](#12-stlstandard-template-library标准模板库)
    - [1.2.1. Sequence containers(有序容器)](#121-sequence-containers有序容器)
      - [1.2.1.1. vector（单端的动态数组）](#1211-vector单端的动态数组)
      - [1.2.1.2. deque（双端数组）](#1212-deque双端数组)
      - [1.2.1.3. list（双向链表）](#1213-list双向链表)
    - [1.2.2. Associative containers(关联性容器)](#122-associative-containers关联性容器)
      - [1.2.2.1. set容器](#1221-set容器)
      - [1.2.2.2. map容器](#1222-map容器)
      - [1.2.2.3. multiset](#1223-multiset)
      - [1.2.2.4. mutimap](#1224-mutimap)
    - [1.2.3. Unordered associative containers(无序关联容器)](#123-unordered-associative-containers无序关联容器)
      - [1.2.3.1. unordered_set](#1231-unordered_set)
      - [1.2.3.2. unordered_map](#1232-unordered_map)
      - [1.2.3.3. unordered_multisert](#1233-unordered_multisert)
      - [1.2.3.4. unordered_multimap](#1234-unordered_multimap)
    - [1.2.4. Container adaptors(容器适配器)](#124-container-adaptors容器适配器)
      - [1.2.4.1. stack容器](#1241-stack容器)
      - [1.2.4.2. queue容器](#1242-queue容器)
      - [1.2.4.3. 优先级队列 priority_queue](#1243-优先级队列-priority_queue)
  - [1.3. STL常用算法](#13-stl常用算法)

<!-- /TOC -->

# 1. Standard Template Library
使用它是一件很愉快的事。


## 1.1. 参考
- [cpprocks](https://cpprocks.com/c11-compiler-support-shootout-visual-studio-gcc-clang-intel/)：查看C++11支持哪些编译，里面还有许多优质的东西，值得挖掘。



## 1.2. STL（Standard Template Library）标准模板库
- STL的核心思想：算法和数据结构的实现是分离的。


- STL广义分类
  - 算法（Algorithm）
  - 容器（Container）
  - 迭代器（Iterator）：相当于一个指针
  - 仿函数（Function object）
  - 适配器（Adaptor）
  - 空间适配器（Alloctor） 



- 常用的容器
  - Vector(向量)
  - Deque(双队列)
  - List(链表)
  - Map/Multimap(映射/多重映射)
  - Set/Multiset(集合/多重集合)
  <p align="center">
    <img src="./figures/容器库成员函数表.png">
  </p>

- STL标准库中的 `容器`内 存储的元素都必须能够拷贝，C++编译器默认提供的是浅拷贝，程序在执行时，会出现问题。因此需要重写构造函数和重载 `=` 操作运算符，执行深拷贝。


### 1.2.1. Sequence containers(有序容器)

#### 1.2.1.1. vector（单端的动态数组）
- 动态数组实现机制：
  > 先为数组开辟较小的空间，然后往数组里面添加数据，当数据的数目超过数组的容量时，再重新分配一块更大的空间（STL中 `vector` 每次扩容时，新的容量都是前一次的两倍），再把之前的数据复制到新的数组中，再把之前的内存释放。
  - 注意：使用动态数组时，尽量减少改变数组容量大小的次数，可以减少时间性能的消耗。
- 动态数组，在运行阶段执行。
- 向容器中插入元素时，内部的元素必须能够执行 `拷贝（必须提供拷贝构造）` 操作。
- `vector` 支持随机迭代访问器。

- 模板：`template <typename T> void Show(T arrNum[], int len);`
- 函数接口
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
  - `swap()`: 交换两个容器的内容


#### 1.2.1.2. deque（双端数组）
- 与 `vector` 容器类似，但是可以在 `Deque` 的两端进行操作。
- `push_back()`: 在容器末尾插入一个元素
- `push_front()` 容器头部插入一个元素
- `pop_front()`: 容器头部删除一个元素
- `pop_back()`: 删除最后一个元素


#### 1.2.1.3. list（双向链表）
- 是一个双向链表的容器，可以高效的进行插入和删除元素。
- 不支持随机存储元素，即不支持 `at.(pos)` 函数和 `[]` 操作符。
- 链表的插入操作：在 pos 位置插入新的节点，新插入的数据存放在 pos 位置之前。
- list 的删除
  - `clear()` 移除容器中所有的数据
  - `erase(begin, end)` 删除区间 `[begin, end)` 的数据，返回下一个元素的位置。
  - `erase(pos)` 删除指定 pos 位置的数据，返回下一个元素的位置。
  - `remove(element)` 删除容器中所有与 element 值匹配的数据。 


### 1.2.2. Associative containers(关联性容器)

#### 1.2.2.1. set容器
- set 是一个 `集合` 容器，包含的元素是唯一的，集合中的元素按照一定的顺序排列，不能随意指定位置插入元素。
- set 底层采用红黑树的数据结构实现的。
- set 支持唯一的键值，容器里面的元素值只能出现一次，而 `multiset` 集合容器中同一个元素值可以出现多次。
- 不可以直接修改 set和multiset集合容器中元素的值，因为集合容器是自动排序的。修改集合容器中元素的值，必须先删除原先元素的值，再插入新元素的值。


- 仿函数（伪函数）
  - 在 `struct` 结构体中定义新的函数。 

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


#### 1.2.2.2. map容器
- map 是关联式容器，一个 map 就是一个键值对。map 中的 `key` 值唯一，容器中的元素按照一定的顺序排列，不能在任意指定的位置插入元素。
- map 的底层原理是按照平衡二叉树的数据结构来实现的，在插入和删除的操作上比 `vector` 容器快。
- map 支持唯一的键值，每个 `key` 只能出现一次，支持 `[]` 操作，形如：`map[key] = value`。 `multimap` 不支持唯一的键值，容器中的每个 `key` 可以出现相同的多次，不支持 `[]` 操作。

```
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


#### 1.2.2.3. multiset
collection of keys, sorted by keys


#### 1.2.2.4. mutimap
collection of key-value pairs, sorted by keys


### 1.2.3. Unordered associative containers(无序关联容器)


#### 1.2.3.1. unordered_set
- `unordered_set` 是一种无序的容器集合。底层采用哈希表实现的。
- STL无序容器存储状态，hash表存储结构图
  <p align="center">
    <img src="./figures/STL无序容器存储状态.png">
  </p>
  
-  `unordered_set` 模板类中的定义
  ```
    template<typename _Value,                        // 容器中存储元素的类型
            typename _Hash = hash<_Value>,           // 确定元素存储位置的哈希函数
            typename _Pred = std::equal_to<_Value>,  // 判断各个元素是否相等
            typename _Alloc = std::allocator<_Value>, // 指定分配器对象的类型
            typename _Tr = __uset_traits<__cache_default<_Value, _Hash>::value>>
  ```

- 注意：此容器模板类中没有重载 `[ ]` 运算符，也没有提供 `at()` 成员方法，`unordered_set` 容器内部存储的元素值不能被修改，可以使用迭代器遍历容器中的数，但不能修改容器中元素的值。


#### 1.2.3.2. unordered_map


#### 1.2.3.3. unordered_multisert


#### 1.2.3.4. unordered_multimap


### 1.2.4. Container adaptors(容器适配器)
容器适配器为有序的容器提供了不同的接口。

#### 1.2.4.1. stack容器
- `push()` 入栈
- `pop()` 出栈
- `top()` 获取栈顶元素
- `size()` 获取栈大小
- `empty()` 栈为空


#### 1.2.4.2. queue容器
- `push()` 入队列
- `pop()` 出队列
- `empty()` 对列为空
- `front()` 队列头部元素


#### 1.2.4.3. 优先级队列 priority_queue
```
// 最大或最小优先级队列变量的声明 

priority_queue<int> g_priq;                            // 默认为最大值优先队列
priority_queue<int, vector<int>, greater<int>> l_priq; // 最小值优先队列
```


## 1.3. STL常用算法
- 函数对象: 需要重载 `()` 操作运算符。函数对象调用与 `回调函数` 的调用类似。
- 分类
  - 预定义函数对象：标准STL模板库中提前预定义了很多的函数对象。
  - 用户自定义的函数对象 

- 函数对象调用
  - 函数对象可以做函数参数。 
  - 函数对象可以做返回值。 
  ```
  clss Stu
  {
    private:
    public:
      void operator() (Stu& T) {}
  }
  ``` 

- 函数适配器
  - 绑定适配器（bind adaptor）
  - 组合适配器（composite adaptor）
  - 指针适配器（pointer adaptor）
  - 成员函数适配器（member function adaptor） 

- STL算法的核心思想
  - STL通过类模板技术，实现了数据类型与容器模型的分离。
  - 通过函数对象实现了自定义数据类型与底层算法的分离。
  - 通过迭代器的方式统一的去遍历容器，向容器中读数据和写数据。


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


- 堆（heap）的STL库中函数
  - `make_heap(first, last, comp);` 建立一个空堆；
  - `push_heap(first, last, comp);` 向堆中插入一个新元素；
  - `top_heap(first, last, comp); ` 获取当前堆顶元素的值；
  - `sort_heap(first, last, comp);` 对当前堆进行排序；


- `stable_partition()`函数
- `upper_bound()` 函数
- `lower_bound()` 函数
