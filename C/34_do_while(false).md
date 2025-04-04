<!--
 * @Author: JohnJeep
 * @Date: 2021-01-18 19:18:53
 * @LastEditTime: 2022-07-08 18:05:38
 * @LastEditors: DESKTOP-0S33AUT
 * @Description: do {
 *                                        ...
 *                                        } while(false)  使用
-->

## 为什么要使用 do {.. } while (false)?

简化某些场景下 `if else` 的多级嵌套。 使用 `do... while (false)` 的用意就在于在 `do {}` 的过程中可以 `break`，使得函数唯一的出口就是最后一行的return，可以避免过多的嵌套括号，使结果更清晰，更容易理解。

没有使用  `do... while (false)`，if 语句之间多级嵌套。
```c
    if (condition1) {
        // some code that should execute if condition is not true
        
        if (condition2) {
            //further code that should not execute if condition or condition2 are true

            if (condition3) {
                //do some more stuff
                break;
            }
        }
    }
```

采用  `do... while (false)` 后，代码逻辑结构更简洁、清晰。
```c
do {
    // some code that should always execute...
    if (condition1)
    {
        //do some stuff
        break;
    }

    // some code that should execute if condition is not true
    if (condition2)
    {
        //do some more stuff
        break;
    }

    //further code that should not execute if condition or condition2 are true
    if (condition3)
    {
        //do some more stuff
        break;
    }

}
while(false);
```

## 应用场景
- 在C/C++ 的 library 中有很多的宏都使用了这种操作。