<!--
 * @Author: JOhnJeep
 * @Date: 2020-09-07 09:18:32
 * @LastEditTime: 2025-04-04 20:06:32
 * @LastEditors: JohnJeep
 * @Description: QT基础知识
 * 
-->

# 1. 信号与槽函数

- QT中使用 `connect()`来操作信号与槽
  > `connect()` 中的第五个参数作用：只有在多线程处理时才有意义。如果是多线程，默认使用队列，如果是单线程，默认使用直接的方式。队列：槽函数所在线程和接收者一样；直接连接：槽函数所在线程与发送者一样。

- 信号
  > signals: 就是信号代码段的标志，这个标志不带 public 、protected、private 等前缀，那是因为信号默认强制规定为公有类型，这样才能保证其他对象能接收到信号。
  - 自定义信号
    - 返回值是void，只需要声明，不需要实现
    - 函数中可以有参数，可以重载
    - 触发信号使用 `emit` 关键字
  - 标准的信号

- 槽函数
  - 自定义槽函数
    - 早期Qt版本必须写到public slots下，现在高级版本可以直接写在public下
    - 返回值为void，需要声明，也需要实现
    - 可以有参数，可以重载
  - 标准的槽函数 

- <font color=red> 注意点 </font>
  - 信号可以连接信号
  - 一个信号可以连接多个槽函数。
  - 多个信号可以连接同一个槽函数。
  - 信号与槽函数的参数类型必须一一对应。
  - 信号的参数个数可以多于槽函数的参数个数，反之则不行。
  - 信号与槽函数重载时，需要将将信号与槽函数分别定义为函数指针，再传入函数的地址，否则编译器会出错。函数指针：`void (*p)(int) = fun`
  - 在Qt4中，槽函数必须使用 `slots` 关键字来修饰，使用下面的方式来调用信号与槽函数。
    > `connect(&sender, SIGNAL(mySignal), &ricever, SOLT(mySlot));`
    这样做的缺点：SIGNAL和 SOLT这两个宏将mySignal和mySlot函数的名字各自转化为字符串，而不做错误检查，而是在运行时才给你报错。因此，不建议使用Qt4的标准，采用Qt5的标准比较好。

# 2. 对象树

Qt 内部维护了一个对象树，通过父类对象来回收子类对象的资源。

父类：按照继承关系来说的。

父对象：按照从属关系来说的。


# 3. lambda表达式

- lambda表达式也叫匿名函数表达式，其中函数没有名称。
- 格式：`[](){}`，需要在项目配置文件中加入 `CONFIG += c++11`，才有效，现在高版本Qt已经默认将该项添加到配置文件中了。
- []标识符，匿名函数
  - `=`: 将类中所有的成员变量、局部变量，按值方式传递。
  - `&`: 将类中所有的局部变量按照引用传递。
  - `this`: 将类中所有的成员变量按值方式进行传递。
- ()参数
  - 标识重载的()操作符参数，若没有参数时，这部分可以省略，参数通过按值`(a, b)` 或按引用`(&a, &b)`两种方式进行传递。
  - `mutable` 关键字，修改值传递进来的拷贝值，而并不是原来的值。
- {} 函数实现体
  - 函数中具体实现的内容
- 返回值
  - 一般使用 `->`来表示返回值：`[]()->int{}`


# 4. 对话框

- 模态对话框（阻塞对话框）：创建对话框后不可以对其它的对话框进行操作。
- 非模态对话框：创建对话框后可以对其它的对话框进行操作。


# 5. Qt内存回收机制

- 指定父对象
  - 如果不指定父对象，子控件与父控件之间没有任何的关系。若指定父对象后，对象上的子对象自动显示。
  - 两种方式指定父对象：
    - `setParent();`  函数
    - 通过构造函数传参数来指定父类对象：`QPushButton* btn3 = new QPushButton(this);`
- 子类对象指定父类对象后，若子类对象是动态分配的，不需要手动 delete 释放，父类对象会自动释放内存。
- 注意：
  - 必须要指定父类对象，否则就需要手动释放。
  - 父对象直接或间接继承于 `QObject` 类。



# 6. QString

Qt 库中对字符串类型进行了封装，QString 类提供了所有字符串操作方法，给开发带来了便利。 由于第三方库的类型基本上都是标准的类型，即使用std::string或char *来表示字符 (串) 类型，因此在Qt框架下需要将QString转换成标准字符 (串) 类型。


# 7. std::string和char *的相互转换

- 将char *或char[]转换为std::string
  ```
  std::string ss,str;
  const char *y="hello";
  const char z[]="hello world";
  ss=y;
  str=z;
  ```

- 将std::string转换为char *或char[]，有3种方法，推荐第二种方法
  - 1)尾部不会附加结束符'\0'
    ```
    std::string str="abc";
    char *p=str.data();
    ```

  - 2)尾部附加结束符'\0'
    ```
    std::string str="Pigman";
    char ch[10];
    strcpy(ch,str.c_str());
    ```
  - 3)尾部不会附加结束符'\0'，第二个参数为复制字符个数，第三个为复制位置
    ```
    std::string str("pig can fly");
    char *p;
    str.copy(p,3,0);
    *(p+3)='\0';　　//　手动添加结束符
    ```


# 8. QString和std::string相互转换，以及避免出现乱码

```
QString qstr;
std::string str;
//　　QString转std::string
str=qstr.toStdString();
str=(const char*)qstr.toLocal8bit();　　　　　　//　中文字符串避免出现乱码
//　　std::string转QString
qstr=QString::fromStdString(str);
qstr=QString::fromLocal8bit(str.c_str());　　//　中文字符串避免出现乱码
```


# 9. QString和char *相互转换

- QString转为char *
  ```cpp  两种方法
    1. 先转为std::string，再转为char *，如上所示
    2. 先转为QByteArray，再转为char *
    
  QString ss("Flying without wings");
  QByteArray sr=ss.toLocal8Bit();
  char ch[10];
  strcpy(ch,sr.data());
  ```


- char *转为QString
  ```cpp
  char *ch="westlife";
  QString str(ch);   // Qt5     
  QString str = QString::fromUtf8(ch);    //  Qt4
  ```


# 10. Qt常用类

## 10.1. QFrame 

- QFrame与QWidget的区别：
  - QWidget类是所有用户界面对象的基类。QFrame是基本控件的基类，QWidget是QFrame基类。其关系如下 `QPushButton,QLabel… -> QFrame ->QWidget`
  
  - Widget是用户界面的基本单元：它从窗口系统接收鼠标，键盘和其他事件，并在屏幕上绘制自己。每个Widget都是矩形的，它们按照Z-order进行排序。


## 10.2. QBoxLayout

- QBoxLayout 可以在水平方向或垂直方向上排列控件，由QHBoxLayout、QVBoxLayout所继承。
  - QHBoxLayout：水平布局，在水平方向上排列控件，即：左右排列。
  - QVBoxLayout：垂直布局，在垂直方向上排列控件，即：上下排列。



## 10.3. QComboBox

QComboBox 是下拉列表框组件类，它提供一个下拉列表供用户选择，也可以直接当作一个 QLineEdit 用作输入。QComboBox 除了显示可见下拉列表外，每个项（item，或称列表项）还可以关联一个 QVariant 类型的变量，用于存储一些不可见数据。


## 10.4. Spacer

QSpacerItem类为布局提供了一个空白区。


## 10.5. QStackedWidget

QStackedWidget继承自 QFrame。QStackedWidget类提供了多页面切换的布局，一次只能看到一个界面。QStackedWidget可用于创建类似 QTabWidget提供的用户界面。


## 10.6. QPushButton

setCheckable(true)为属性，表示可以选中

setChecked（true）为属性的值，表示已经选中


## 10.7. QString

`QString QString::trimmed() const`   返回值为去除了字符串中的开头和结尾的空字符，不能去除掉字符串内部的空字符。

`QString QString::simplified() const`  返回值为去除了字符串开头和结尾空的字符，并且字符串内部的空字符也去掉。

# 11. 多线程

多线程编程也使用回调函数作为新线程里的任务函数。

TCP Client 接口

connectToHost(IP, Port)函数连接服务端

readyRead()信号是否被触发来判断是否有数据传入，如果该信号被触发，则调用自定义函数（如：ClientRecvData()）来保存接收到的数据。通过connect()函数，将信号readyRead()与槽函数ClientRecvData()建立映射关系。在槽函数ClientRecvData()中通过read()函数接收数据




关闭 TCP 连接

void QAbstractSocket::disconnectFromHost()
void QAbstractSocket::close()
void QAbstractSocket::abort()


TCP Server 接口 



QHostAddress::Null表示一个空地址；

QHostAddress::LocalHost表示IPv4的本机地址127.0.0.1；

QHostAddress::LocalHostIPv6表示IPv6的本机地址；

QHostAddress::Broadcast表示广播地址255.255.255.255；

QHostAddress::Any表示IPv4的任意地址；

QHostAddress::AnyIPv6表示IPv6的任意地址。

ewConnection() 来判断是否接收到了新的连接，当服务端接收到一个客户端的连接时，就会触发信 newConnection()

nextPendingConnection() 函数获得连接客户端的SOCKET套接字：

readyRead()来判断是否有数据传入，当客户端向服务端成功发送数据之后，就会在服务端触发readyRead()信号


# 12. Qt Creator 快捷键

- `F1`  查看帮助
- `F2`  跳转到函数定义（和Ctrl+鼠标左键一样的效果）
- `Shift+F2`    声明和定义之间切换
- `F4`: 同名.h与.cpp文件之间快速的切换 
- `ctrl + shift + ↑或↓`: 整行向上或向下移动
- `ctrl + alt + ↑或↓`: 将当前行整行向上或向下复制
- `shift + F5`:   停止调试
- `F5`:           启动调试
- `Ctrl+1`        欢迎模式
- `Ctrl+2`        编辑模式
- `Ctrl+3`        调试模式
- `Ctrl+4`        项目设置模式
- `Ctrl+5`        帮助模式    
- `Ctrl+6`        输出模式
- `ESc`           切换到编辑模式
- `Ctrl+B`        编译工程
- `Ctrl+R`        运行工程
- `Ctrl+I`        自动对齐
- `Ctrl+/`        注释行，取消注释行
- `Ctrl+F`        查找替换当前选中的内容
- `Ctrl+I`        自动对齐
- `Ctrl+Shift+F`  查找内容
- `F5`            开始调试
- `Shift+F5`      停止调试
- `F9`            设置和取消断点
- `F10`           单步前进
- `F11`           单步进入函数
- `Shift + F11`   单步跳出函数
- `Ctrl+ Tab`     快速切换已打开的文件
- 快速在(.cpp)文件中添加方法的实体：将光标移动到 .h 文件中的方法声明，先按 `Alt(按住)+ Enter`，再按 `回车键` 将在 .cpp 中添加该函数的声明。
- `Ctrl + Shift + R` 一次性修改多个变量


# 13. References

- [关于QT的系统总结（非常全面，非常好）](https://www.cnblogs.com/findumars/p/5529526.html)
- [Qt 学习之路 2](https://www.bookstack.cn/read/qt-study-road-2/da83aa582cd34d4d.md)：简单些了Qt一些主要的学习内容。
- [Qt 资料大全](https://waleon.blog.csdn.net/article/details/51752029)：一去丶二三里 CSDN博主总结的Qt学习的资料。
- [QT读取和写入文件](https://blog.csdn.net/qq_40732350/article/details/86774306)：讲解了QFile、QFileInfo、QTemporaryFile、QDir的相关用法。
- [QT学习笔记8：QDir类及其用法总结](https://www.cnblogs.com/aiguona/p/10298226.html)：QDir类提供了访问系统目录结构及其内容的与平台无关的方式。
- [Qt拷贝文件、文件夹、创建文件夹、删除文件夹操作](https://blog.csdn.net/ljt350740378/article/details/71381705)
- [QT创建TCP Socket通信](https://www.cnblogs.com/stones-dream/p/9700660.html)
- https://cryfeifei.top/2020/05/30/qt-ji-chu-jiao-cheng/
-[QString, Std::string, char *相互转换](https://www.cnblogs.com/zxbilly/p/9195411.html)



