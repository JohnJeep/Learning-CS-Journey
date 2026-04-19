<!--
 * @Author: JohnJeep
 * @Date: 2026-04-19
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-04-19 16:48:57
 * @Description: English project overview
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
-->

<div align="center">

# Learning Computer Science Journey

A continuously evolving computer science knowledge base.

From hardware foundations to application-layer systems, this repository records practical learning notes, deep dives, and long-term growth.

[Knowledge Map](#knowledge-map)<br>
[Start Reading](#programming-languages)<br>
[Feedback](#feedback)<br>
[Growth Timeline](STARCHARTS.md)<br>
[Chinese README](README.md)

</div>

---

## Project Overview

This project is a structured and continuously maintained computer science knowledge base, developed since 2020. It covers a complete learning path from low-level hardware concepts to high-level software engineering topics.

All content comes from personal study, work experience, and hands-on practice, with a strong focus on understanding both how things work and why they work.

### Highlights

- Complete coverage across multiple domains: architecture, assembly, C/C++, Go, OS, networking, databases, cloud native, robotics, and more.
- Continuously updated content with a focus on principles, implementation details, and practical takeaways.
- Integrated notes from top courses, including MIT, Stanford, and Tsinghua materials.
- Practice-oriented writing with code examples, configs, and debugging workflows.
- Diagram-driven explanations using draw.io and mind maps.

---

## Knowledge Map

```
                        ┌──────────────────────┐
                        │   Learning CS Journey │
                        └──────────┬───────────┘
           ┌───────────────┬───────┴───────┬───────────────┐
           ▼               ▼               ▼               ▼
    ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
    │ Hardware     │ │ Programming  │ │ Systems &    │ │ Applications │
    │ Foundations  │ │ Languages    │ │ Networking   │ │ & Frontiers  │
    └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘
           │               │               │               │
    Architecture     C / C++          OS (MIT/THU)   Cloud Native
    Assembly         Go               Linux           Distributed
    Embedded         Python           Network(CS144)  AI / AIGC
    GNU              Shell/JS/TS      MySQL / Redis   Robotics (ROS2)
                                      Nginx           AutoDrive
```

---

## Table of Contents

- [Learning Computer Science Journey](#learning-computer-science-journey)
  - [Project Overview](#project-overview)
    - [Highlights](#highlights)
  - [Knowledge Map](#knowledge-map)
  - [Table of Contents](#table-of-contents)
  - [Hardware Foundations](#hardware-foundations)
    - [Chip and CPU Architecture](#chip-and-cpu-architecture)
    - [Assembly Language](#assembly-language)
    - [Embedded Development](#embedded-development)
    - [GNU Toolchain](#gnu-toolchain)
  - [Programming Languages](#programming-languages)
    - [C](#c)
    - [C++](#c-1)
    - [Go](#go)
    - [Other Languages](#other-languages)
  - [Operating Systems](#operating-systems)
    - [MIT 6.828](#mit-6828)
    - [OSTEP](#ostep)
    - [Tsinghua OS](#tsinghua-os)
    - [Core Reading](#core-reading)
  - [Computer Networks](#computer-networks)
    - [Stanford CS144](#stanford-cs144)
  - [Data Structures and Algorithms](#data-structures-and-algorithms)
  - [Design Patterns](#design-patterns)
  - [Databases](#databases)
  - [Cloud Native and Distributed Systems](#cloud-native-and-distributed-systems)
    - [Cloud Native](#cloud-native)
    - [Distributed Systems](#distributed-systems)
  - [AI and Frontier Topics](#ai-and-frontier-topics)
    - [AI](#ai)
    - [Autonomous Driving](#autonomous-driving)
    - [Robotics](#robotics)
  - [Linux Systems](#linux-systems)
  - [Developer Tools and Productivity](#developer-tools-and-productivity)
    - [Version Control](#version-control)
    - [Industrial Protocols](#industrial-protocols)
    - [Useful Tools](#useful-tools)
    - [Documentation and Knowledge Management](#documentation-and-knowledge-management)
    - [Repository Maintenance Scripts](#repository-maintenance-scripts)
  - [Recommended Resources](#recommended-resources)
    - [Self-study Roadmaps](#self-study-roadmaps)
    - [Recommended Books](#recommended-books)
  - [Feedback](#feedback)
  - [License](#license)

---

## Hardware Foundations

### Chip and CPU Architecture

Directory: `Architecture/`

Learning notes on major architectures including x86, ARM, and NVIDIA.

### Assembly Language

Directory: `Assembly/`

- [Assembly Fundamentals](Assembly/assembly.md): registers, addressing modes, and common instructions.

### Embedded Development

Directory: `Embedded/`

STM32-focused notes and code (F103 / F207 / F407), including peripherals, drivers, and RTOS-related topics.

### GNU Toolchain

Directory: `GNU/`

Core GNU ecosystem topics such as GCC, glibc, and ABI details.

---

## Programming Languages

### C

Directory: `C/`

C is the first language in this journey. The notes focus on practical and low-level essentials such as memory, pointers, structs, callbacks, and common pitfalls.

<details>
<summary>Expand key C topics</summary>

**Keywords and specifiers**
- [volatile](C/6_volatile.md)
- [const](C/10_const.md)
- [restrict](C/11_restrict.md)
- [extern](C/12_extern.md)
- [static](C/38_static.md)
- [typedef](C/17_typedef.md)
- [register and auto](C/33_register_auto.md)

**Pointers and memory**
- [Pointer basics](C/21_pointer.md)
- [Function pointers](C/9_pointer_function_function_pointer.md)
- [Arrays and pointers](C/24_array_pointer.md)
- [Callbacks](C/37_callback_function.md)
- [Memory layout](C/30_memory.md)
- [memset](C/14_memset.md)
- [memcpy vs strncpy](C/15_strncpy_memcpy.md)
- [memcmp vs strcmp](C/16_memcmp_strcmp.md)
- [Struct alignment](C/18_struct_bytealigned.md)
- [packed attribute](C/28___attribute__((packed)).md)
- [sizeof vs strlen](C/31_sizeof_strlen.md)

**Fundamentals and advanced practice**
- [Character encoding](C/2_character_encoding.md)
- [Byte order](C/3_byte_order.md)
- [Segmentation fault analysis](C/7_segment_fault.md)
- [Enumerations](C/22_enumerations.md)
- [State machines](C/23_state_machine.md)
- [Thread pool](C/27_thread_pool.md)
- [Dynamic libraries](C/40_dynamic_library.md)
- [Coding conventions](C/0_code_of_conduct.md)

</details>

### C++

Directory: `Cpp/`

Systematic notes from fundamentals to modern C++ features, STL internals, memory management, and concurrency.

- [C++ Basics](Cpp/Novice.md)
- [C++ Advanced](Cpp/Advance.md)
- [Modern C++ Features](Cpp/C++11.md), [C++14](Cpp/C++14.md), [C++17](Cpp/C++17.md), [C++20](Cpp/C++20.md)
- [STL](Cpp/STL.md)
- [Memory Management](Cpp/MemoryManagement.md)
- [Concurrency](Cpp/Concurrency.md)
- [Move Semantics](Cpp/MoveSemantics.md), [Smart Pointers](Cpp/SmartPointer.md), [Style Guide](Cpp/StyleGuide.md)

### Go

Directory: `Go/`

Comprehensive coverage from language basics to runtime internals.

| Category | Content |
|------|------|
| Fundamentals | [Go Basics](Go/Go.md)<br>[Standard Library](Go/GoStandardLibrary.md)<br>[Coding Style](Go/GolangCodeConduct.md)<br>[Testing](Go/GoTest.md) |
| Advanced | [CGO](Go/CGO.md)<br>[Compiler](Go/Compiler.md)<br>[Performance](Go/Performance.md)<br>[Third-party Packages](Go/ThirdPackages.md) |
| Internals | [Go Internals](Go/GoInternals.md)<br>[GPM Scheduler](Go/GPM.md)<br>[Map](Go/Map.md)<br>[Slice](Go/Slice.md) |

### Other Languages

| Language | Directory | Description |
|------|------|------|
| Python | `Python/` | Python basics and standard library |
| JavaScript | `JavaScript/` | JavaScript language notes |
| TypeScript | `TypeScript/` | TypeScript setup and usage |
| Node.js | `node.js/` | Runtime and package ecosystem |
| Shell | `Shell/` | Shell scripting, tmux, and crontab |
| Qt | `Qt/` | Qt components and GUI development |

---

## Operating Systems

Directory: `OS/`

A structured OS learning path combining top courses and classic references.

### MIT 6.828

- [MIT 6.828 Operating System Engineering](OS/MIT-CS6.828/CS6.828.md)

### OSTEP

- [Operating Systems: Three Easy Pieces](http://pages.cs.wisc.edu/~remzi/OSTEP/)
- Virtualization and concurrency notes are organized under `OS/Operating-System-Three-Easy-Pieces/`.

### Tsinghua OS

- [Tsinghua OS Course (2019)](https://chyyuu.gitbooks.io/os_course_info/content/)
- [uCore OS Lab Guide](https://learningos.github.io/ucore_os_webdocs/)

### Core Reading

- Computer Systems: A Programmer's Perspective (CSAPP)
- Linkers and Loaders related engineering notes
- Practical systems-level books and reverse engineering references

---

## Computer Networks

Directory: `Network/`

- [Network Fundamentals](Network/NetworkPrimer.md)
- [Network Terms and Abbreviations](Network/AbbrNetworkTerms.md)
- [HTTP Basics](Network/HTTP.md)
- [Wireshark Analysis](Network/Wireshark.md)

### Stanford CS144

- [CS 144 Course Site](https://cs144.github.io/)
- Top-down networking references and Wireshark labs are included in the notes.

---

## Data Structures and Algorithms

Directory: `DataStructure/`

- [Data Structures and Algorithms Notes](DataStructure/DataStructure.md)

---

## Design Patterns

Directory: `DesignPattern/`

- [23 Design Patterns in C++](DesignPattern/DesignPattern.md)

---

## Databases

| Database | Content |
|------|------|
| MySQL | [MySQL Fundamentals](MySQL/MySQL.md): SQL tuning, connection pools, architecture |
| Redis | [Redis Fundamentals](Redis/Redis.md): data structures, cluster topics, source-level notes |
| Nginx | [Nginx Basics](Nginx/Nginx.md): configuration, reverse proxy, load balancing |

---

## Cloud Native and Distributed Systems

### Cloud Native

Directory: `CloudNative/`

| Stack | Notes |
|------|------|
| Containerization | Docker, Kubernetes |
| Service Communication | [gRPC](CloudNative/gRPC/)<br>[Protobuf](CloudNative/Probobuf/)<br>[MQTT](CloudNative/Mqtt.md) |
| Messaging | [Kafka](CloudNative/Kafka/)<br>[RocketMQ](CloudNative/RocketMQ/) |
| Governance | [Zookeeper](CloudNative/Zookeeper/)<br>[Actor Model](CloudNative/Actor.md) |
| Observability and CI/CD | [InfluxDB](CloudNative/Influxdb.md)<br>[Jenkins](CloudNative/Jenkins/) |

### Distributed Systems

Directory: `Distributed/`

- MIT 6.824-related learning path and references.
- Topics include microservices, API gateway, and high-availability practices.

---

## AI and Frontier Topics

### AI

Directory: `AI/`

- [AIGC Overview](AI/AIGC.md)
- [Ollama](AI/Ollama.md): local LLM deployment and usage notes

### Autonomous Driving

Directory: `AutoDrive/`

- [Autoware](AutoDrive/Autoware.md)

### Robotics

Directory: `Robotics/`

ROS2-focused learning path from fundamentals to practical workflows:

- Core framework and DDS communication
- Navigation and SLAM
- Perception with LiDAR and point clouds
- Simulation (Gazebo), behavior trees, and ROSControl
- URDF modeling and tf transformations

---

## Linux Systems

Directory: `Linux/`

- [Linux Fundamentals](Linux/linux-primer.md)
- [Compile and Link](Linux/compile-link.md)
- [GDB](Linux/gdb.md), [VIM](Linux/vim.md)
- [CMake](Linux/CMake/cmake-tutorial.md), [Makefile](Linux/CMake/Makefile.md)
- [inode](Linux/inode.md), [System Programming](Linux/system-program.md), [Tooling](Linux/linux-tools.md)

---

## Developer Tools and Productivity

### Version Control

Directory: `Git-SVN/`

- [Git: From Basics to Internals](Git-SVN/Git.md)
- [SVN](Git-SVN/SVN.md)
- [Install and Setup](Git-SVN/Install.md)
- [Development Guide](Git-SVN/Develop.md)
- [FAQ](Git-SVN/FAQ.md)
- [Commit Convention](Git-SVN/commit-convention.md)
- [CI/CD Pipeline](Git-SVN/Pipeline.md)
- [Git Internals](Git-SVN/Internals.md)

### Industrial Protocols

Directory: `Protocol/`

Covers Modbus, Omron, Siemens, Beckhoff, Fanuc, WebRTC, SMTP, and related protocol notes.

### Useful Tools

| Tool | Link |
|------|------|
| VS Code | [Shortcuts and tips](StudyTool/VSCode.md) |
| JetBrains | [Plugins and usage](StudyTool/JetbrainsPlugins.md) |
| draw.io | [Shortcuts](StudyTool/draw.io.md) |
| Windows Productivity | [Windows shortcuts](StudyTool/Windows10.md) |
| Tablet Tooling | [iPad tools list](StudyTool/iPad-tools.md) |
| Learning Resources | [Useful websites](StudyTool/WebsiteReferences.md) |

### Documentation and Knowledge Management

Directory: `Markdown/`

- [Typora guide](Markdown/TyporaMarkdown.md)
- [LaTeX formulas](Markdown/LaTex.md)
- [Greek alphabet reference](Markdown/GreekAlphabet.md)
- [Jupyter guide](Markdown/Jupyter.md)
- [Jupyter notebook example](Markdown/Jupyter.ipynb)

### Repository Maintenance Scripts

Root directory:

- [add_frontmatter.py](add_frontmatter.py): batch-add front matter metadata to documents.
- [format_all_md.sh](format_all_md.sh): batch-format Markdown files.
- [clean.bat](clean.bat): cleanup script for Windows environments.

---

## Recommended Resources

### Self-study Roadmaps

- [cs_study_plan](https://github.com/spring2go/cs_study_plan)
- [TeachYourselfCS-CN](https://github.com/keithnull/TeachYourselfCS-CN/blob/master/TeachYourselfCS-CN.md)

### Recommended Books

| Area | Books |
|------|------|
| Computer Systems | CSAPP, systems programming and linking references |
| Operating Systems | Operating Systems: Three Easy Pieces |
| Computer Networks | Computer Networking: A Top-Down Approach |
| Distributed Systems | Designing Data-Intensive Applications |

---

## Feedback

Suggestions and discussions are welcome via Issues or Discussions.

Before opening an issue, please read [convention.md](convention.md).

Recommended reading for better questions:

- [How To Ask Questions The Smart Way](https://github.com/ryanhanwu/How-To-Ask-Questions-The-Smart-Way)
- [How to Ask Questions in Open Source Communities](https://github.com/seajs/seajs/issues/545)

---

## License

Copyright (c) 2026 by John Jeep. All Rights Reserved.
