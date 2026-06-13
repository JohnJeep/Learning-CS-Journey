<!--
 * @Author: JohnJeep
 * @Date: 2025-04-09 10:34:40
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-24 16:58:03
 * @Description: windows basic concept
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->


# Windows 编程基本概念

- 宽字符：Unicode 字符，双字节

- 窄字符：ASCII 字符，单字节


- Windows 中所有的底层函数都是 Unicode 编码。

  `_UNICODE` 的例子你可以在 `TCHAR.H` 中找到，它用来解析`TCHAR`等类型是宽字符还是单字节字符，以及一些字符串宏的处理结
  果是宽字符还是单字节。

- COM 组件必须使用 Unicode 编码（COM 组件可以理解为 DLL，主要是用于代码重用）。

  比如在 Windows API 中：`FindWindowW`和`FindWindowA`。W 的意思为 wide（宽），A 的意思为 ASCII

- 在 `Windows.h`中有一个`UNICODE`宏，底层调用宽字节版本，窄字节版本仅作编码转换。

- Unicode：一个中文字符长度为 1；多字节字符集：一个中文字符长度为 2

# 字符

- L 表示 long
- P 表示 pointer
- C 表示 constant
- W 表示 wide
- T：win32 环境中有一个_T 宏，用来标识字符是否采用 Unicode 编码（两字节表示一个字符），若程序中定义了
  Unicode，该字符/字符串被作为 Unicode 字符串，否则就是标准的
  ANSI（单字节表示一个字符）字符串。
- STR 表示这个变量是一个字符串。

LPCSTR: long pointer const string，可看成 const char*，与 PCSTR 相似

LPSTR：可看成 char*，与 PSTR 相似

**关于 wchar_t\*的：**

LPCWSTR，PCWSTR，LPWSTR，PWSTR

## Ascii 与 unicode 互转

```cpp
//unicode转为ascii
std::string UnicodeToAscii( const std::wstring& in_str )
{
	int nNeedChars = WideCharToMultiByte( CP_ACP, 0, in_str.c_str(), -1, 0, 0, 0, 0 );
	if (nNeedChars > 0)//再次判断一下
	{	
		std::string temp;
		temp.resize(nNeedChars);
		::WideCharToMultiByte( CP_ACP, 0, in_str.c_str(), -1, &temp[0], nNeedChars, 0, 0 );
		return temp;
	}
 
	return std::string();
}
 
//ascii转为unicode
std::wstring AsciiToUnicode( const std::string& in_str )
{
	int nNeedWchars = MultiByteToWideChar( CP_ACP, 0, in_str.c_str(), -1, NULL, 0 );
	if (nNeedWchars > 0)
	{
		std::wstring temp;
		temp.resize(nNeedWchars);
		::MultiByteToWideChar( CP_ACP, 0, in_str.c_str(), -1, &temp[0], nNeedWchars );
		return temp;
	}
 
	return std::wstring();
}
```

# Utf8 和 Unicode 的互转

```cpp
//utf8转为unicode
std::wstring UTF8ToUnicode( const std::string& in_utf8Str )
{
	int nNeedWchars = MultiByteToWideChar( CP_UTF8, 0, in_utf8Str.c_str(), -1, NULL, 0 );
	if (nNeedWchars > 0)
	{
		std::wstring temp;
		temp.resize(nNeedWchars);
		::MultiByteToWideChar( CP_UTF8, 0, in_utf8Str.c_str(), -1, &temp[0], nNeedWchars );
		return temp;
	}
 
	return std::wstring();
}
 
//unicode转为utf8
std::string UnicodeToUTF8( const std::wstring& in_wStr )
{
	int nNeedChars = WideCharToMultiByte( CP_UTF8, 0, in_wStr.c_str(), -1, 0, 0, 0, 0 );
	if (nNeedChars > 0)//再次判断一下
	{	
		std::string temp;
		temp.resize(nNeedChars);
		::WideCharToMultiByte( CP_UTF8, 0, in_wStr.c_str(), -1, &temp[0], nNeedChars, 0, 0 );
		return temp;
	}
 
	return std::string();
}
```

# Ascii 和 Utf8 的互转

```cpp
 //ascii转为utf8
std::string AsciiToUTF8(const std::string& in_asciiStr)
{
	return UnicodeToUTF8(AsciiToUnicode(in_asciiStr));
}
 
//utf8转为ascii
std::string UTF8ToAscii(const std::string& in_utf8Str)
{
	return UnicodeToAscii(UTF8ToUnicode(in_utf8Str));
}
```

# GB2312 和 Unicode 的互转

```cpp
 
//BIG5 转换成 Unicode：
std::wstring BIG5ToUnicode(const std::string& strBIG5String)
{
	UINT nCodePage = 950; //BIG5
	int nNeedWchars = MultiByteToWideChar(nCodePage, 0, strBIG5String.c_str(), -1, NULL, 0);
	if (nNeedWchars > 0)
	{
		std::wstring temp;
		temp.resize(nNeedWchars);
		::MultiByteToWideChar( nCodePage, 0, strBIG5String.c_str(), -1, &temp[0], nNeedWchars );
		return temp;
	}
 
	return std::wstring();
}
 
//Unicode 转换成 BIG5：
std::string UnicodeToBIG5(const std::wstring& strUnicodeString)
{
	UINT nCodePage = 950; //BIG5
	int nNeedChars = WideCharToMultiByte(nCodePage, 0, strUnicodeString.c_str(), -1, NULL, 0, NULL, NULL);
	//再次判断一下
	if (nNeedChars > 0)
	{
		std::string temp;
		temp.resize(nNeedChars);
		::WideCharToMultiByte( nCodePage, 0, strUnicodeString.c_str(), -1, &temp[0], nNeedChars, 0, 0 );
		return temp;
	}
 
	return std::string();
}
```





# References

-  [Windows 下的字符集转换](https://blog.csdn.net/r5014/article/details/112220672)
-  https://blog.csdn.net/luoyeaijiao/article/details/7266490
-  [多字节字符集与 Unicode 字符集](https://www.cnblogs.com/gaohongchen01/p/4006700.html)
-  [Unicode 和多字节字符集 (MBCS) 支持](https://www.cnblogs.com/ccjt/p/4320244.html)

