<!--
 * @Author: JohnJeep
 * @Date: 2021-01-18 19:18:53
 * @LastEditTime: 2021-01-18 19:28:47
 * @LastEditors: Please set LastEditors
 * @Description: do {
 *                                        ...
 *                                        } while(false)  使用
-->

- 为什么要使用do {.. }while(false)?
  > 简化某些场景下多级判断的代码嵌套。 使用 `do...while(false)` 的用意就在于在 `do{}` 的过程中可以 `break`，使得函数唯一的出口就是最后一行的return，可以避免过多的嵌套括号，使结果更清晰，更容易理解。

    ```
    do
    {
    //some code that should always execute...

    if ( condition )
    {
        //do some stuff
        break;
    }

    //some code that should execute if condition is not true

    if ( condition2 )
    {
        //do some more stuff
        break;
    }

    //further code that should not execute if condition or condition2 are true

    }
    while(false);
    ```

- 在C/C++ 的library中有很多的宏都使用了这种操作。