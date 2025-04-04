<!--
 * @Author: JohnJeep
 * @Date: 2022-01-27 17:21:36
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-05-27 17:03:46
 * @Description: 动态库学习
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->

# 动态库

## Windows

### GetProcAddress()

函数原型

```cpp
FARPROC GetProcAddress(
    HMODULE hModule, // DLL模块句柄
    LPCSTR lpProcName // 函数名
);
```



### FreeLibrary()



## Linux

### dlopen()

`dlopen` 是一个Linux下的动态链接库操作函数，用于在运行时加载共享对象（动态链接库）并返回一个句柄（handle），以便在程序中使用。这个句柄可以用于后续的符号查找和符号解析，以及动态链接库的卸载。 `dlopen` 可以将一个共享对象加载到进程的地址空间中，从而允许程序在运行时动态地加载库。

`dlopen` 的函数原型如下：

```
cCopy code
void *dlopen(const char *filename, int flag);
```

其中，`filename` 参数是一个字符串，表示要加载的共享对象的文件名。`flag` 参数用于指定加载库的行为，可以是以下值之一：

- `RTLD_LAZY`：在程序首次访问动态链接库的符号时进行符号解析；
- `RTLD_NOW`：在 `dlopen` 调用时立即解析动态链接库中的所有符号；
- `RTLD_GLOBAL`：使动态链接库中定义的符号对其他动态链接库和程序的符号表可见；
- `RTLD_LOCAL`：使动态链接库中定义的符号仅对该动态链接库的符号表可见。

`dlopen` 函数将返回一个指向句柄的指针。该句柄可用于调用 `dlsym` 函数来查找动态链接库中的符号，并且在程序结束时可以使用 `dlclose` 函数卸载动态链接库。

使用 `dlopen` 可以实现一些有趣的应用，如插件机制、动态库加载和更新、代码加密和混淆等。

### dlclose()

在 Linux 系统中，`dlclose` 是一个函数，它用于关闭动态链接库（也称为共享对象），释放该库在进程中占用的资源。

当程序使用 `dlopen` 函数打开一个动态链接库时，系统会将该库加载到进程的虚拟地址空间中，并在进程中创建一个动态链接库句柄（handle）。这个句柄可以被用于在程序中访问该库中定义的函数和变量。

当程序使用完这个动态链接库后，可以使用 `dlclose` 函数来释放该库占用的资源，并关闭该库。这可以释放进程中已分配的虚拟地址空间，同时释放库中定义的全局变量等资源。

`dlclose` 的语法如下：

```c
int dlclose(void *handle);
```

其中，`handle` 是一个指向动态链接库句柄的指针。函数返回值为0表示成功，否则表示失败。注意，如果在使用该库的过程中，还存在指向该库中某个函数或变量的指针，那么在关闭该库之前，必须将这些指针置为NULL，否则可能导致程序崩溃。

总之，`dlclose` 是一个用于关闭动态链接库并释放占用资源的函数，它可以帮助程序更好地管理系统资源，从而提高程序的效率和稳定性。

### dlsym()

`dlsym()` 是 Linux 操作系统中的一个动态链接库函数，用于在运行时动态加载和使用共享库中定义的符号。它的作用是在共享库中查找一个符号，并返回该符号的地址。

`dlsym()` 函数的原型如下：

```c
#include <dlfcn.h>
void* dlsym(void*handle, const char*symbol);
```

其中，`handle` 参数是一个指向已打开共享库的句柄的指针，`symbol` 参数是要查找的符号的名称。

通过调用 `dlsym()` 函数，我们可以在程序运行时动态地加载共享库，并使用其中定义的符号。这使得我们可以编写模块化的程序，将不同的功能模块放在不同的共享库中，并在运行时根据需要加载这些库。

另外，`dlsym()` 还常用于编写插件系统。插件系统允许在程序运行时动态地加载和卸载插件，从而扩展程序的功能。在插件系统中，`dlsym()` 可以用来查找插件中定义的符号，并将其与主程序进行连接，从而实现插件的功能扩展。

总之，`dlsym()` 是 Linux 系统中一种非常有用的动态链接库函数，它允许程序在运行时动态地加载和使用共享库中定义的符号，从而使程序更加灵活和可扩展。

### dlerror()


在Linux操作系统中，`dlerror` 是一个用于动态链接库错误处理的函数。它被定义在 `` 头文件中。

`dlerror` 函数可以返回最近的动态链接库错误信息。在程序中使用动态链接库时，如果出现错误，可以使用 `dlerror` 函数获取错误信息并进行处理。

通常，当使用 `dlopen` 函数打开一个动态链接库时，如果出现错误，该函数会返回一个空指针，并且可以使用 `dlerror` 函数来获取错误信息。同样，当使用 `dlsym` 函数获取一个动态链接库中的符号时，如果出现错误，该函数也会返回一个空指针，并且可以使用 `dlerror` 函数获取错误信息。

`dlerror` 函数返回的错误信息是一个字符串，表示最近的错误信息。如果该函数返回空指针，则表示没有错误信息。

示例程序：

```c
#include <stdio.h>
#include <dlfcn.h>

int main() {
    void* handle = dlopen("libtest.so", RTLD_NOW);
    if (!handle) {
        printf("dlopen error: %s\n", dlerror());
        return -1;
    }
    
    void (*test_func)() = (void (*)())dlsym(handle, "test");
    if (!test_func) {
        printf("dlsym error: %s\n", dlerror());
        return -1;
    }
    
    test_func();
    
    dlclose(handle);
    return 0;
}
```

在上面的代码中，如果 `dlopen` 函数或者 `dlsym` 函数出现错误，就可以使用 `dlerror` 函数获取错误信息并输出。这样可以方便地调试程序，并解决动态链接库相关的问题。

# References

- [百度百科 dlsym](https://baike.baidu.com/item/dlsym/6603915)
- [百度百科 GetProcAddress](https://baike.baidu.com/item/GetProcAddress/1633633)