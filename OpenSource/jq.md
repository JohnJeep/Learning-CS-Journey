<!--
 * @Author: JohnJeep
 * @Date: 2025-04-01 00:40:42
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-02-13 10:03:27
 * @Description: 
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->


# Introduction

jq 是 stedolan 开发的一个轻量级的和灵活的命令行 JSON 处理器。

它主要用于在命令行界面处理 JSON 输入，并使用给定的过滤条件来过滤符合条件的新的 JSON 串。

通常在类 Unix 环境下，我们可以快速的使用 `jq` 来进行 JSON 数据格式化过滤和处理。同时需要注意的是，该命令行工具和 awk/sed/grep 工具一样，属于系统的默认命令，如果系统没有该命令，可以尝试使用如下方式进行安装。

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

json 字符串紧凑输出

```bash
q -c . data.json
{"a":11,"b":22,"person":{"name":"Bob","age":25,"city":"New York"},"fruits":["apple","banana","cherry"],"meta":{"data":{"id":123,"value":"example"}}}
```

将 json 转义为字符串

```bash
jq -sR . data.json 
"{\n  \"a\":11,\n  \"b\":22,\n  \"person\": {\n      \"name\": \"Bob\",\n      \"age\": 25,\n      \"city\": \"New York\"\n  },\n  \"fruits\": [\"apple\", \"banana\", \"cherry\"],\n  \"meta\": {\n    \"data\" :{\n      \"id\": 123,\n      \"value\": \"example\"\n    }\n  }\n}"

```

json 转义为字符串并紧凑输出

```bash
jq -c . data.json | jq -sR .
"{\"a\":11,\"b\":22,\"person\":{\"name\":\"Bob\",\"age\":25,\"city\":\"New York\"},\"fruits\":[\"apple\",\"banana\",\"cherry\"],\"meta\":{\"data\":{\"id\":123,\"value\":\"example\"}}}\n"

# 另外一种写法
jq -c "tostring" data.json
"{\"a\":11,\"b\":22,\"person\":{\"name\":\"Bob\",\"age\":25,\"city\":\"New York\"},\"fruits\":[\"apple\",\"banana\",\"cherry\"],\"meta\":{\"data\":{\"id\":123,\"value\":\"example\"}}}"
```


# References

- 官网：https://stedolan.github.io/jq/
- 在线 Playground：https://jqplay.org/
- 内置帮助：`jq --help`
- 手册页：`man jq`
