
<!-- TOC -->

- [1. rapidjson](#1-rapidjson)
- [2. 基础用法](#2-基础用法)
  - [2.1. 是什么](#21-是什么)
  - [2.2. 特征](#22-特征)
  - [2.3. 怎么用](#23-怎么用)
    - [2.3.1. 解析](#231-解析)
    - [2.3.2. 创建](#232-创建)
  - [2.4. 注意事项](#24-注意事项)
  - [2.5. 查询 Value](#25-查询-value)
    - [2.5.1. 查询 Array](#251-查询-array)
    - [2.5.2. 查询 Object](#252-查询-object)
    - [2.5.3. 查询 String](#253-查询-string)
- [3. 内部原理](#3-内部原理)
  - [3.1. Reader](#31-reader)
  - [3.2. Writer](#32-writer)
  - [3.3. Document](#33-document)
  - [3.4. Value](#34-value)
  - [3.5. Allocator](#35-allocator)
- [4. Reference](#4-reference)

<!-- /TOC -->


# 1. rapidjson 

# 2. 基础用法

## 2.1. 是什么

rapidjson 是腾讯的开源 Json解析框架，用 C++ 代码实现，用于解析和生成 JSON 由于全部代码仅用头文件实现，因此很容易集成在项目中。根据其作者 Milo Yipz 所做的比较，可以看出 rapidjson 的性能非常可观。通过使用 DOM（Document Object Model）可以很方便的将 Json 转化成 DOM，然后查询修改，再转化为一个 Json。通过 rapidjson 库，可以方便我们进行参数的传递，解析的工作。Json 非常便捷的支持键值对、数组、以及深入的嵌套，在编写程序时，可以帮助我们聚焦于业务，而对于参数的传递，则可以较少的投入精力。

## 2.2. 特征

优点

- RapidJSON 库仅由头文件组成，只需把头文件复制至你的项目中。

- 独立、最小依赖。不需依赖 STL、Boost 库等。

- 没使用 C++ 异常、RTTI。

- 支持跨平台。

  编译器：Visual Studio、gcc、clang 等
  架构：x86、x64、ARM 等
  操作系统：Windows、Mac OS X、Linux、iOS、Android 等

- RapidJSON 解析和生成速度快。使用模版及内联函数去降低函数调用开销。内部经优化的 Grisu2 及浮点数解析实现。

- RapidJSON 是一个C++ 的 JSON 解析器及生成器，它的灵感来自 RapidXml

- RapidJSON 对 Unicode 友好。它支持 UTF-8、UTF-16、UTF-32 (大端序／小端序)，并内部支持这些编码的检测、校验及转码。

  例如，RapidJSON 可以在分析一个 UTF-8 文件至 DOM 时，把当中的 JSON 字符串转码至 UTF-16。它也支持代理对（surrogate pair）及 ”\u0000”（空字符），这些特征使得我们，可以很好的把 rapidjson 集成到项目代码之中，提高开发的效率。

缺点

- RapidJSON 库较难使用，源码使用了大量的模板类和模板函数。
- RapidJSON 库的 SAX 风格接口较难使用，因此使用 DOM 接口即可。

## 2.3. 怎么用

Rapidjson [官网](https://rapidjson.org/zh-cn/md_doc_tutorial_8zh-cn.html) 非常详细的介绍了如何去使用，自己就不再把官方的文档般到这里来了。只总结一些使用的心得。

### 2.3.1. 解析

json 字符串

```cpp
static constexpr auto json = "{\"Info\":[{\"lots\":10,\"order_algorithm\":\"01\",\"buy_close\":9000,\"spread_shift\":0,\"position_b_sell\":0,\
\"position_a_buy_today\":0,\"position_a_buy_yesterday\":0,\"sell_open\":-9000,\"list_instrument_id\":[\"rb1705\",\"rb1701\"],\
\"position_b_buy_today\":0,\"buy_open\":-9000,\"position_a_sell_yesterday\":0,\"strategy_id\":\"02\",\"position_b_buy\":0,\
\"a_wait_price_tick\":1,\"trade_model\":\"boll_reversion\",\"b_wait_price_tick\":0,\"sell_close\":9000,\"only_close\":0,\
\"order_action_limit\":400,\"is_active\":1,\"lots_batch\":1,\"position_a_sell\":0,\"position_b_buy_yesterday\":0,\
\"user_id\":\"063802\",\"position_a_buy\":0,\"trader_id\":\"1601\",\"position_a_sell_today\":0,\"stop_loss\":0,\
\"position_b_sell_today\":0,\"position_b_sell_yesterday\":0,\"on_off\":0},{\"lots\":20,\"order_algorithm\":\"02\",\
\"buy_close\":9000,\"spread_shift\":0,\"position_b_sell\":0,\"position_a_buy_today\":0,\"position_a_buy_yesterday\":0,\
\"sell_open\":-9000,\"list_instrument_id\":[\"ni1705\",\"ni1701\"],\"position_b_buy_today\":0,\"buy_open\":-9000,\
\"position_a_sell_yesterday\":0,\"strategy_id\":\"01\",\"position_b_buy\":0,\"a_wait_price_tick\":1,\"trade_model\":\"boll_reversion\",\
\"b_wait_price_tick\":0,\"sell_close\":9000,\"only_close\":0,\"order_action_limit\":400,\"is_active\":1,\"lots_batch\":1,\
\"position_a_sell\":0,\"position_b_buy_yesterday\":0,\"user_id\":\"063802\",\"position_a_buy\":0,\"trader_id\":\"1601\",\
\"position_a_sell_today\":0,\"stop_loss\":0,\"position_b_sell_today\":0,\"position_b_sell_yesterday\":0,\"on_off\":0}],\
\"MsgSendFlag\":0,\"MsgErrorReason\":\"IDorpassworderror\",\"MsgRef\":1,\"MsgType\":3,\"MsgResult\":0}";
```

代码解析实现

```cpp
rapidjson::Document doc;
doc.Parse(json);

// 2. Modify it by DOM.
rapidjson::Value& s = doc["MsgSendFlag"];
s.SetInt(s.GetInt() + 1);

rapidjson::Value& infoArray = doc["Info"];
if (infoArray.IsArray()) {
    for (int i = 0; i < infoArray.Size(); i++) {
        const rapidjson::Value& object = infoArray[i];
        int lots = object["lots"].GetInt();
        std::string order_algorithm = object["order_algorithm"].GetString();
        std::cout << "int lots = " << lots << std::endl;
        std::cout << "string order_algorithm = " << order_algorithm << std::endl;
        const rapidjson::Value& info_object = object["list_instrument_id"];
        if (info_object.IsArray()) {
            for (int j = 0; j < info_object.Size(); j++) {
                std::string instrument = info_object[j].GetString();
                std::cout << "instrument[" << j << "] = " << instrument << std::endl;
            }
        }
    }
}

// 3. Stringify the DOM
rapidjson::StringBuffer buffer;
rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
doc.Accept(writer);
std::cout << buffer.GetString() << std::endl;
```

### 2.3.2. 创建

组建一个json字符串

```cpp
std::cout << json << std::endl;

std::cout << "|==================|" << std::endl;
std::cout << "||rapidjson output||" << std::endl;
std::cout << "|==================|" << std::endl;

rapidjson::Document doc; // doc 对象默认类型为 null
doc.SetObject(); // 改变类型
rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();

doc.AddMember("MsgSendFlag", 1, allocator);
doc.AddMember("MsgErrorReason", "IDorpassworderror", allocator);
doc.AddMember("MsgRef", 1, allocator);

rapidjson::Value info_array(rapidjson::kArrayType);

for (int i = 0; i < 2; i++) {
    rapidjson::Value info_object(rapidjson::kObjectType); // 调用构造函数，参数类型为：json vale type
    info_object.SetObject();
    info_object.AddMember("lots", 10 + i, allocator);
    info_object.AddMember("order_algorithm", "01", allocator);

    rapidjson::Value instrument_array(rapidjson::kArrayType);
    for (int j = 0; j < 2; j++) {
        rapidjson::Value instrument_object(rapidjson::kObjectType);
        instrument_object.SetObject();
        instrument_object.SetString("cu1701");
        instrument_array.PushBack(instrument_object, allocator);
    }

    info_object.AddMember("list_instrument_id", instrument_array, allocator);
    info_array.PushBack(info_object, allocator);
}

doc.AddMember("Info", info_array, allocator);

// 3. Stringify the DOM
rapidjson::StringBuffer buffer; // 输出流，它分配一个内存缓冲区，供写入整个 JSON
rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
doc.Accept(writer);
std::cout << buffer.GetString() << std::endl; // GetString() 获取缓冲区的内容
```

## 2.4. 注意事项

- RapidJSON 在类型转换时会检查数值的范围。

- 字符串字面量的优化
  
  - 只储存指针，不作复制
  
- 优化“短”字符串
  - 在 `Value` 内储存短字符串，无需额外分配。
  - 对 UTF-8 字符串来说，32 位架构下可存储最多 11 字符，64 位下 21 字符（x86-64 下 13 字符）。
  
- 可选地支持 `std::string`（定义 `RAPIDJSON_HAS_STDSTRING=1`）

- 最小化 DOM 的内存开销。

  对大部分 32／64 位机器而言，每个 JSON 值只占 16 或 20 字节（不包含字符串）。

- 支持快速的预设分配器。

  - 它是一个堆栈形式的分配器（顺序分配，不容许单独释放，适合解析过程之用）。
  - 使用者也可提供一个预分配的缓冲区。（有可能达至无需 CRT 分配就能解析多个 JSON）

- 部分 C++11 支持

  - 右值引用（rvalue reference）
  - 支持 `noexcept` 修饰符
  - 支持简洁的范围 for 循环
  
- Value 赋值使用move语义，而不是 copy 语义。也就是说，拷贝构造和拷贝赋值函数都是用 move 语义实现的。

- 当 string 的生命周期不足时，Value 应该使用 Copy-string 存储策略，否则 value 无法长期存储字符串。

- StringBuffer 是一个简单的输出流，当该缓冲区满溢时会自动增加容量（默认为 256 个字符）。

- FileReadStream / FileWriteStream，和IStreamWrapper / OStreamWrapper，它们都是字节流，不处理编码。若输入流或输出流数据是非 UTF-8 编码时，输入流数据需要用 EncodedInputStream 或AutoUTFInputStream 包装，而输出流数据需要用 EncodedOutputStream 或 AutoUTFOutputStream 包装。

- **不建议**使用 wistream 和 wostream。

## 2.5. 查询 Value

### 2.5.1. 查询 Array

缺省情况下，`SizeType` 是 `unsigned` 的 typedef。在多数系统中，Array 最多能存储 $2^{32}-1$ 个元素。

用法与 `std::vector` 类似，同时也支持 C++11 的 for 循环迭代。

### 2.5.2. 查询 Object

- `FindMember()`:  检查成员是否存在并返回它的 `Value`。

  用于查询 Object ，比普通的迭代遍历对象要高效。因为当 `operator[](const char*)` 找不到成员，它会断言失败。若我们不确定一个成员是否存在，便需要在调用 `operator[](const char*)` 前先调用 `HasMember()`，这会导致两次查找，降低了效率。

  ```c++
  // 普通查询
  static const char* kTypeNames[] = 
      { "Null", "False", "True", "Object", "Array", "String", "Number" };
   
  for (Value::ConstMemberIterator itr = document.MemberBegin();
      itr != document.MemberEnd(); ++itr) {
      printf("Type of member %s is %s\n",
          itr->name.GetString(), kTypeNames[itr->value.GetType()]);
  }
  
  // 采用FindMember()
  Value::ConstMemberIterator itr = document.FindMember("hello");
  if (itr != document.MemberEnd()) {
      printf("%s\n", itr->value.GetString());
  }
  ```

rapidjson 为了最大化性能，大量使用了**浅拷贝**，使用之前一定要了解清楚。如果采用了浅拷贝，特别要注意局部对象的使用，以防止对象已被析构了，却还在被使用。

### 2.5.3. 查询 String

RapidJSON 提供两个 String 的存储策略。

1. copy-string: 分配缓冲区，然后把来源数据复制至它。
2. const-string: 简单地储存字符串的指针。

- 支持处理包含 `\0` 的字符。

  根据 RFC 4627，JSON String 可包含 Unicode 字符 `U+0000`，在 JSON 中会表示为 `"\u0000"`。问题是，C/C++ 通常使用空字符结尾字符串（null-terminated string），这种字符串把 `‘\0’` 作为结束符号。为了符合 RFC 4627，RapidJSON 支持包含 `U+0000` 的 String。若要获取这种字符串的 长度，调用  `GetStringLength()` 函数即可。

- API
  - `GetString()`
  -  `GetStringLength()`

# 3. 内部原理

简称

- SAX 是 Simple API for XML 的缩写。
- DOM 是 Document Object Model(文件对象模型)的缩写。

核心 API。

- 源码路径 `include/rapidjson` 下有很多的文件，掌握下面几个核心的文件就能用好 rapidjson 常用的功能。
  -  `allocators.h`
  - `document.h`
  - `rapidjson.h`
  - `reader.h`
  - `writer.h`
  - `stringbuffer.h`

## 3.1. Reader

`Reader`: （`GenericReader<...>` 的 typedef）是 JSON 的 SAX 风格**解析器**。

`Reader` 从输入流解析一个 JSON。当它从流中读取字符时，它会基于 JSON 的语法去分析字符，并向处理器发送事件。

`Reader` 是 `GenericReader` 模板类的别名，位于 `reader.h` 文件中。 

```cpp
namespace rapidjson {

template <typename SourceEncoding, typename TargetEncoding, typename Allocator = MemoryPoolAllocator<> >
class GenericReader {
    // ...
};

typedef GenericReader<UTF8<>, UTF8<> > Reader;

} // namespace rapidjson
```



## 3.2. Writer  

`Writer`: （`GenericWriter<...>` 的 typedef）是 JSON 的 SAX 风格**生成器**。`Writer` 将数据生成 Json 格式数据。

`Writer` 是一个模板类，而不是一个 typedef，并没有 `GenericWriter`，位于 `writer.h` 中。下面是 `Write` 模板类的声明：

```cpp
namespace rapidjson {
    
template<typename OutputStream, typename SourceEncoding = UTF8<>, typename TargetEncoding = UTF8<>, typename StackAllocator = CrtAllocator, unsigned writeFlags = kWriteDefaultFlags>
class Writer {
public:
    typedef typename SourceEncoding::Ch Ch;

    static const int kDefaultMaxDecimalPlaces = 324;

    //! Constructor
    /*! \param os Output stream.
        \param stackAllocator User supplied allocator. If it is null, it will create a private one.
        \param levelDepth Initial capacity of stack.
    */
    explicit
    Writer(OutputStream& os, StackAllocator* stackAllocator = 0, size_t levelDepth = kDefaultLevelDepth) : 
        os_(&os), level_stack_(stackAllocator, levelDepth * sizeof(Level)), maxDecimalPlaces_(kDefaultMaxDecimalPlaces), hasRoot_(false) {}

    explicit
    Writer(StackAllocator* allocator = 0, size_t levelDepth = kDefaultLevelDepth) :
        os_(0), level_stack_(allocator, levelDepth * sizeof(Level)), maxDecimalPlaces_(kDefaultMaxDecimalPlaces), hasRoot_(false) {}

#if RAPIDJSON_HAS_CXX11_RVALUE_REFS
    Writer(Writer&& rhs) :
        os_(rhs.os_), level_stack_(std::move(rhs.level_stack_)), maxDecimalPlaces_(rhs.maxDecimalPlaces_), hasRoot_(rhs.hasRoot_) {
        rhs.os_ = 0;
    }
};
    
}
```

生成 json 格式的数据有两种方式。

1. 用字符串缓冲结合 `Writer`。

   ```cpp
   #include "rapidjson/writer.h"
   #include "rapidjson/stringbuffer.h"
   #include <iostream>
    
   using namespace rapidjson;
   using namespace std;
    
   void main() {
       StringBuffer s;
       Writer<StringBuffer> writer(s);
       
       writer.StartObject();
       writer.Key("hello");
       writer.String("world");
       writer.Key("t");
       writer.Bool(true);
       writer.Key("f");
       writer.Bool(false);
       writer.Key("n");
       writer.Null();
       writer.Key("i");
       writer.Uint(123);
       writer.Key("pi");
       writer.Double(3.1416);
       writer.Key("a");
       writer.StartArray();
       for (unsigned i = 0; i < 4; i++)
           writer.Uint(i);
       writer.EndArray();
       writer.EndObject();
    
       cout << s.GetString() << endl;
   }
   ```

   ```cpp
   // 输出
   {"hello":"world","t":true,"f":false,"n":null,"i":123,"pi":3.1416,"a":[0,1,2,3]}
   ```

2. 用 `Document` 类、分配器(Allocator)结合 `AddMember()` 函数。

## 3.3. Document

`Document`: 是模板类 `GenericDocument<UTF8<> >` 的别名，位于 `document.h` 文件中。

```cpp
namespace rapidjson {

template <typename Encoding, typename Allocator = RAPIDJSON_DEFAULT_ALLOCATOR, typename StackAllocator = RAPIDJSON_DEFAULT_STACK_ALLOCATOR >
class GenericDocument : public GenericValue<Encoding, Allocator> {
public:
    ...
};

//! GenericDocument with UTF8 encoding
typedef GenericDocument<UTF8<> > Document;
    
}
```

## 3.4. Value

`Value`: 是 DOM API 的核心，它是模板类 `GenericValue<UTF8<> >` 的别名，位于 `document.h` 中。

```cpp
namespace rapidjson {
    
template <typename Encoding, typename Allocator = RAPIDJSON_DEFAULT_ALLOCATOR >
class GenericValue {
public:
    ...
};

//! GenericValue with UTF8 encoding
typedef GenericValue<UTF8<> > Value;
}
```

`Value` 是一个可变类型。在 RapidJSON 的上下文中，一个 `Value` 的实例可以包含6种 JSON 数据类型。

- Null
- String
- Bool
- Object
- Array
- Number

注意点：

- 为了减少在64位架构上的内存消耗，`SizeType` 被定义为 `unsigned` 而不是 `size_t`。
- 32位整数的零填充可能被放在实际类型的前面或后面，这依赖于字节序。这使得它可以将32位整数不经过任何转换就可以解释为64位整数。
- `Int` 永远是 `Int64`

- `Handler`: 用于处理来自 `Reader` 的事件（函数调用）。处理器必须包含以下的成员函数。

  ```cpp
  class Handler {
      bool Null();
      bool Bool(bool b);
      bool Int(int i);
      bool Uint(unsigned i);
      bool Int64(int64_t i);
      bool Uint64(uint64_t i);
      bool Double(double d);
      bool RawNumber(const Ch* str, SizeType length, bool copy);
      bool String(const Ch* str, SizeType length, bool copy);
      bool StartObject();
      bool Key(const Ch* str, SizeType length, bool copy);
      bool EndObject(SizeType memberCount);
      bool StartArray();
      bool EndArray(SizeType elementCount);
  };
  ```

## 3.5. Allocator

Allocator(分配器)，位于 `allocators.h` 文件中。

`MemoryPoolAllocator` 是 DOM 的默认内存分配器。它只申请内存而不释放内存。这对于构建 DOM 树非常合适。

在它的内部，它从基础的内存分配器申请内存块（默认为 `CrtAllocator`）并将这些内存块存储为单向链表。当用户请求申请内存，它会遵循下列步骤来申请内存：

1. 如果可用，使用用户提供的缓冲区。（见 User Buffer section in DOM）
2. 如果用户提供的缓冲区已满，使用当前内存块。
3. 如果当前内存块已满，申请新的内存块。

```cpp
namespace rapidjson {

class CrtAllocator {
public:
    static const bool kNeedFree = true;
    void* Malloc(size_t size) { 
        if (size) //  behavior of malloc(0) is implementation defined.
            return RAPIDJSON_MALLOC(size);
        else
            return NULL; // standardize to returning NULL.
    }
    void* Realloc(void* originalPtr, size_t originalSize, size_t newSize) {
        (void)originalSize;
        if (newSize == 0) {
            RAPIDJSON_FREE(originalPtr);
            return NULL;
        }
        return RAPIDJSON_REALLOC(originalPtr, newSize);
    }
    static void Free(void *ptr) RAPIDJSON_NOEXCEPT { RAPIDJSON_FREE(ptr); }

    bool operator==(const CrtAllocator&) const RAPIDJSON_NOEXCEPT {
        return true;
    }
    bool operator!=(const CrtAllocator&) const RAPIDJSON_NOEXCEPT {
        return false;
    }
};
    
    
template <typename BaseAllocator = CrtAllocator>
class MemoryPoolAllocator {
	...
};
	
}
```

# 4. Reference

- 官方 Github: https://github.com/Tencent/rapidjson/
- 英文文档: https://rapidjson.org/
- 中文文档: https://rapidjson.org/zh-cn/index.html
- Unicode 字符代码: https://www.rapidtables.org/zh-CN/code/text/unicode-characters.html
- Unicode 15.0 Character Code Charts: https://www.unicode.org/charts/

