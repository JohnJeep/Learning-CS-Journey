<!--
 * @Author: JohnJeep
 * @Date: 2021-01-25 21:25:38
 * @LastEditTime: 2021-01-25 21:36:34
 * @LastEditors: Please set LastEditors
 * @Description: snprintf()函数的使用
-->
- 函数原型
    ```C
    int snprintf(char *str, size_t size, const char *format, ...)
    ```
- 功能：将可变参数 `…` 按照 `format` 的格式格式化为字符串，然后再将其拷贝至 `str`中。
  - (1) 如果格式化后的 `字符串长度 < size`，则将此字符串全部复制到 `str` 中，并给其后添加一个字符串结束符`('\0')`；
  - (2) 如果格式化后的 `字符串长度 >= size`，则只将其中的 `(size-1)` 个字符复制到 `str` 中，并给其后添加一个字符串结束符`('\0')`，返回值为将要写入的字符串长度。

- 返回值：
  - 若成功则返回 `预写入的字符串长度`。
  - 若出错则返回 `负值`。

```C
#include <stdio.h> 
  
int main() 
{ 
    char buffer[50]; 
    char* s = "geeksforgeeks"; 
  
    // Counting the character and storing  
    // in buffer using snprintf 
    int j = snprintf(buffer, 6, "%s\n", s); 
  
    // Print the string stored in buffer and 
    // character count 
    printf("string:\n%s\ncharacter count = %d\n", buffer, j); 
  
    return 0; 
} 
```

```C++
#include <cstdio>
#include <iostream>

using namespace std;

int main()
{
    char buffer[100];
    int retVal, buf_size = 100;
    char name[] = "Max";
    int age = 23;

    retVal = snprintf(buffer, buf_size, "Hi, I am %s and I am %d years old", name, age);
    if (retVal > 0 && retVal < buf_size) {
        cout << buffer << endl;
        cout << "Number of characters written = " << retVal << endl;
    }
    else {
        cout << "Error writing to buffer" << endl;
    }
    
    return 0;
}
```