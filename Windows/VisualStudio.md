## VisualStudio 版本对比

因为微软的版本比较乱，所以要理清版本，首先需要区分 VS 和 VC 的含义：

- VS（Visual Studio）: 针对多语言（C++、C#、F#、J#、Asp、Web 等）的 IDE 集成开发环境
- VC（Visual C++）: 针对 C++ 语言的 IDE 集成开发环境，也称为 MSVC

换言之，VS 包含 VC， VC 只是 VS 的其中一个工具集。

微软历年发布的 IDE 版本如下：

| IDE 名称           | 发布时间 | IDE 版本 | 工具集版本 | MSC_VER | MSVC++      | 系统支持    |
| :----------------- | :------- | :------- | :--------- | :------ | :---------- | :---------- |
| Visual C++6.0      | 1998     | 6        | V60        | 1200    | MSVC++ 6.0  | WinXP、Win7 |
| Visual Studio 2002 | 2002     | 7        | V70        | 1300    | MSVC++ 7.0  | WinXP、Win7 |
| Visual Studio 2003 | 2003     | 8        | V71        | 1310    | MSVC++ 7.1  | WinXP、Win7 |
| Visual Studio 2005 | 2005     | 9        | V80        | 1400    | MSVC++ 8.0  | WinXP、Win7 |
| Visual Studio 2008 | 2008     | 10       | V90        | 1500    | MSVC++ 9.0  | WinXP、Win7 |
| Visual Studio 2010 | 2010     | 11       | V100       | 1600    | MSVC++ 10.0 | WinXP、Win7 |
| Visual Studio 2012 | 2012     | 12       | V110       | 1700    | MSVC++ 11.0 | WinXP、Win7 |
| Visual Studio 2013 | 2013     | 13       | V120       | 1800    | MSVC++ 12.0 | Win7、Win10 |
| Visual Studio 2015 | 2015     | 14       | V140       | 1900    | MSVC++ 14.0 | Win7、Win10 |
| Visual Studio 2017 | 2017     | 15       | V141       | 1910    | MSVC++ 14.1 | Win7、Win10 |
| Visual Studio 2019 | 2019     | 16       | V142       | 1920    | MSVC++ 14.2 | Win7、Win10 |
| Visual Studio 2022 | 2022     | 17       | V143       | 1930    | MSVC++ 14.3 | Win7、Win10 |

## 查看 VS 工程版本

对于一个 VS 工程，要想知道它是用哪个 VC 编译的，只需要用文本打开根目录下的 `*.sln` 文件，即可在文件开头找到一段版本说明，例如：

```
Microsoft Visual Studio Solution File, Format Version 12.00
# Visual Studio Version 17
VisualStudioVersion = 17.1.32328.378
MinimumVisualStudioVersion = 12.0.40629.0
```

查上表可知，Visual Studio Version 17 即 Visual Studio 2022。

如果以后工程更换了 VS 版本，可以用同样的方法查找版本。





