# STL Queue 中 最大能 push 多少元素？

阅读标准库中的源码可知：Queue 底层是基于 deque 实现的。源码中 `push` 函数调用的是 deque 容器中的 `push_back` 函数，而 deque 容器中队列能分配最大的值由 `max_size` 函数决定，`max_size` 函数的返回的类型为 `size_type`。一步一步追踪朔源，发现 `size_type` 是 `size_t` 的别名，而 `size_t` 又是 `__SIZE_TYPE__` 的别名，`__SIZE_TYPE__` 最终是 `long unsigned int` 的别名。终于找到源头了，STL 标准库的 Queue 容器最大能 push 多少个值由 `long unsigned int` 值决定的，而数据类型是跟操作系统的位数相关的，32 位与 64 位操作系统中  `long unsigned int` 的值是不一样。32 位系统中数值为 $2^{32}$，64 位系统中数值为 $2^{64}$。

下面列出 STL 中的用到的源码部分

```cpp
// queue 容器中的 push 操作
void
push(const value_type& __x)
{ c.push_back(__x); }
```

```cpp
// queue 容器类模板部分代码
template<typename _Tp, typename _Sequence = deque<_Tp> >
    class queue
    {
      // concept requirements
      typedef typename _Sequence::value_type _Sequence_value_type;
        
      template<typename _Tp1, typename _Seq1>
        friend bool
        operator==(const queue<_Tp1, _Seq1>&, const queue<_Tp1, _Seq1>&);

      template<typename _Tp1, typename _Seq1>
        friend bool
        operator<(const queue<_Tp1, _Seq1>&, const queue<_Tp1, _Seq1>&);

    public:
      typedef typename _Sequence::value_type                value_type;
      typedef typename _Sequence::reference                 reference;
      typedef typename _Sequence::const_reference           const_reference;
      typedef typename _Sequence::size_type                 size_type;
      typedef          _Sequence                            container_type;

    protected:
      /**
       *  'c' is the underlying container.  Maintainers wondering why
       *  this isn't uglified as per style guidelines should note that
       *  this name is specified in the standard, [23.2.3.1].  (Why?
       *  Presumably for the same reason that it's protected instead
       *  of private: to allow derivation.  But none of the other
       *  containers allow for derivation.  Odd.)
       */
      _Sequence c;
        ......
    };
```

```cpp
// deque 容器中的 push_back 操作
void
push_back(const value_type& __x)
{
if (this->_M_impl._M_finish._M_cur
!= this->_M_impl._M_finish._M_last - 1)
{
this->_M_impl.construct(this->_M_impl._M_finish._M_cur, __x);
++this->_M_impl._M_finish._M_cur;
}
else
_M_push_back_aux(__x);
}
```

```cpp
// deque 容器中能最大分配的值
/**  Returns the size() of the largest possible %deque.  */
size_type
max_size() const _GLIBCXX_NOEXCEPT
{ return _M_get_Tp_allocator().max_size(); }
```

```cpp
// 类型的别名
typedef size_t  size_type;
typedef __SIZE_TYPE__ size_t;
#define __SIZE_TYPE__ long unsigned int
```

