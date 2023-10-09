### How do I know whether a variable is allocated on the heap or the stack?

一、C语言中返回函数中局部变量值和指针
(1) 在C语言中，一个函数可以直接返回函数中定义的局部变量，其实在函数返回后，局部变量是被系统自动回收的，因为局部变量是分配在栈空间，那为什么还可以返回局部变量，其实这里返回的是局部变量的副本（拷贝）。

(2) 函数返回局部变量地址：局部变量内存分配在栈空间，因为函数返回后，系统自动回收了函数里定义的局部变量，所以运行时去访问一个被系统回收后的地址空间，一定就会发生段错误，这是C/C++语言的特点。内存空间分配在堆中即可。

二、GO函数中返回变量，指针
示例代码：

```go
package main

import "fmt"

func fun() *int {    //int类型指针函数
    var tmp := 1
    return &tmp      //返回局部变量tmp的地址
}

func main() {
    var p *int
    p = fun()
    fmt.Printf("%d\n", *p) //这里不会像C，报错段错误提示，而是成功返回变量V的值1
}
```

参考go FAQ里面的一段话：

```sh
How do I know whether a variable is allocated on the heap or the stack?

From a correctness standpoint, you don't need to know. Each variable in Go exists as long as there are references to it. The storage location chosen by the implementation is irrelevant to the semantics of the language.

The storage location does have an effect on writing efficient programs. When possible, the Go compilers will allocate variables that are local to a function in that function's stack frame. However, if the compiler cannot prove that the variable is not referenced after the function returns, then the compiler must allocate the variable on the garbage-collected heap to avoid dangling pointer errors. Also, if a local variable is very large, it might make more sense to store it on the heap rather than the stack.

In the current compilers, if a variable has its address taken, that variable is a candidate for allocation on the heap. However, a basic escape analysis recognizes some cases when such variables will not live past the return from the function and can reside on the stack.
```

意思是说go语言编译器会自动决定把一个变量放在栈还是放在堆，编译器会做逃逸分析(escape analysis)，当发现变量的作用域没有跑出函数范围，就可以在栈上，反之则必须分配在堆。所以不用担心会不会导致memory leak，因为GO语言有强大的垃圾回收机制。go语言声称这样可以释放程序员关于内存的使用限制，更多的让程序员关注于程序功能逻辑本身。

对于动态new出来的局部变量，go语言编译器也会根据是否有逃逸行为来决定是分配在堆还是栈，而不是直接分配在堆中。

结论：
函数内部局部变量，无论是动态new出来的变量还是创建的局部变量，它被分配在堆还是栈，是由编译器做逃逸分析之后做出的决定。

### References

- Go FAQ: https://go.dev/doc/faq#stack_or_heap
- CSDN Go语言---函数返回局部变量地址：https://blog.csdn.net/li_101357/article/details/80209413

