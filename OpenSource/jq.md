<!--
 * @Author: JohnJeep
 * @Date: 2025-04-01 00:40:42
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 19:37:08
 * @Description: jq usage
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# Introduction

jq 是 stedolan 开发的一个轻量级的和灵活的命令行 JSON 处理器。

它主要用于在命令行界面处理 JSON 输入，并使用给定的过滤条件来过滤符合条件的新的 JSON 串。

通常在类 Unix 环境下，我们可以快速的使用 `jq` 来进行 JSON 数据格式化过滤和处理。同时需要注意的是，该命令行工具和
awk/sed/grep
工具一样，属于系统的默认命令，如果系统没有该命令，可以尝试使用如下方式进行安装。

```bash
# Ubuntu/Debian 系列
$ sudo apt-get install jq 

# CentOS 系列
$ yum install jq 
```

验证安装

```bash
jq --version
```


# Usage

例如：有一个 json 文件：data.json

```json
{
  "a":11,
  "b":22,
  "person": {
      "name": "Bob",
      "age": 25,
      "city": "New York"
  },
  "fruits": ["apple", "banana", "cherry"],
  "meta": {
    "data" :{
      "id": 123,
      "value": "example"
    }
  }
}
```

## 标准 JSON 数据格式化输出

只压缩成一行，不进行转义
```bash
jq -c . data.json
{"a":11,"b":22,"person":{"name":"Bob","age":25,"city":"New York"},"fruits":["apple","banana","cherry"],"meta":{"data":{"id":123,"value":"example"}}}
```

## 将 json 数据转义为字符串，但中间的换行符也会被转义

```bash
jq -sR . data.json 
"{\n  \"a\":11,\n  \"b\":22,\n  \"person\": {\n      \"name\": \"Bob\",\n      \"age\": 25,\n      \"city\": \"New York\"\n  },\n  \"fruits\": [\"apple\", \"banana\", \"cherry\"],\n  \"meta\": {\n    \"data\" :{\n      \"id\": 123,\n      \"value\": \"example\"\n    }\n  }\n}"

```

## 将标准 JSON 转换为转义后的 JSON 字符串格式，去掉中间的换行符

```bash
jq 'tojson' data.json
"{\"a\":11,\"b\":22,\"person\":{\"name\":\"Bob\",\"age\":25,\"city\":\"New York\"},\"fruits\":[\"apple\",\"banana\",\"cherry\"],\"meta\":{\"data\":{\"id\":123,\"value\":\"example\"}}}"

# 另外一种写法
jq -c . data.json | jq -sR .
"{\"a\":11,\"b\":22,\"person\":{\"name\":\"Bob\",\"age\":25,\"city\":\"New York\"},\"fruits\":[\"apple\",\"banana\",\"cherry\"],\"meta\":{\"data\":{\"id\":123,\"value\":\"example\"}}}\n"
```

## 将转义后的 JSON 字符串格式转换为标准 JSON 格式

字符串的内容为下面的内容，存在 escape.txt 中
```
"{\"a\":11,\"b\":22,\"person\":{\"name\":\"Bob\",\"age\":25,\"city\":\"New York\"},\"fruits\":[\"apple\",\"banana\",\"cherry\"],\"meta\":{\"data\":{\"id\":123,\"value\":\"example\"}}}"
```

执行 `jq 'fromjson' escape.txt` 后，输出结果为：
```json
{
  "a":11,
  "b":22,
  "person": {
      "name": "Bob",
      "age": 25,
      "city": "New York"
  },
  "fruits": ["apple", "banana", "cherry"],
  "meta": {
    "data" :{
      "id": 123,
      "value": "example"
    }
  }
}

```


# References

- 官网：https://stedolan.github.io/jq/
- 在线 Playground：https://jqplay.org/
- 内置帮助：`jq --help`
- 手册页：`man jq`
