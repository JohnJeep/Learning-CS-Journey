<!--
 * @Author: JohnJeep
 * @Date: 2025-12-07 23:39:49
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-03-07 15:23:17
 * @Description: VCS Tools in ROS
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

- [1. VCS](#1-vcs)
  - [1.1. VCS 工具概述](#11-vcs-工具概述)
  - [1.2. 主要功能](#12-主要功能)
    - [1.2.1. 批量仓库管理](#121-批量仓库管理)
    - [1.2.2. 支持多种VCS](#122-支持多种vcs)
    - [1.2.3. 仓库状态检查](#123-仓库状态检查)
  - [1.3. 典型使用场景](#13-典型使用场景)
    - [1.3.1. YAML 配置文件示例 (`depend_repos.yml`)](#131-yaml-配置文件示例-depend_reposyml)
    - [1.3.2. 常用命令](#132-常用命令)
  - [1.4. 在你的脚本中的应用](#14-在你的脚本中的应用)
  - [1.5. 为什么 ROS 项目使用 VCS？](#15-为什么-ros-项目使用-vcs)
  - [1.6. 与 Git 子模块的对比](#16-与-git-子模块的对比)
  - [1.7. 安装 VCS](#17-安装-vcs)
  - [1.8. 实际示例](#18-实际示例)
- [2. Google repo](#2-google-repo)
  - [2.1. 核心概念](#21-核心概念)
    - [2.1.1. 解决什么问题](#211-解决什么问题)
    - [2.1.2. 基本工作原理](#212-基本工作原理)
  - [2.2. 主要功能](#22-主要功能)
    - [2.2.1. 初始化工作区](#221-初始化工作区)
    - [2.2.2. 同步所有仓库](#222-同步所有仓库)
    - [2.2.3. 常用命令](#223-常用命令)
  - [2.3. Manifest 文件示例](#23-manifest-文件示例)
  - [2.4. Repo 工作流程示例](#24-repo-工作流程示例)
  - [2.5. Repo vs Git 子模块 vs VCS](#25-repo-vs-git-子模块-vs-vcs)
  - [2.6. Repo 的高级特性](#26-repo-的高级特性)
    - [2.6.1. 分层 manifest](#261-分层-manifest)
    - [2.6.2. 工作区管理](#262-工作区管理)
    - [2.6.3. Gerrit 集成](#263-gerrit-集成)
  - [2.7. 实际使用案例](#27-实际使用案例)
    - [2.7.1. AOSP (Android Open Source Project)](#271-aosp-android-open-source-project)
    - [2.7.2. 企业多仓库项目](#272-企业多仓库项目)
  - [2.8. Repo 的优势](#28-repo-的优势)
  - [2.9. 安装和使用](#29-安装和使用)


# 1. VCS

`vcs` 是 **VCS (Version Control System) Tools** 的缩写，这是一个专门为 **ROS (Robot Operating System)** 生态系统开发的工具集，用于管理多个代码仓库的版本控制。

## 1.1. VCS 工具概述

VCS 工具是 ROS 社区开发的一组 Python 脚本，用于同时管理多个 Git 仓库（或其他版本控制系统仓库），类似于 Google 的 `repo` 工具或 Git 的子模块，但更简单易用。

## 1.2. 主要功能

### 1.2.1. 批量仓库管理
```bash
# 从一个YAML文件导入多个仓库
vcs import < directory < repos.yaml

# 导出当前仓库状态
vcs export --exact > repos.yaml
```

### 1.2.2. 支持多种VCS
- Git（最常用）
- Mercurial
- SVN

### 1.2.3. 仓库状态检查
```bash
# 检查所有仓库的状态
vcs status

# 列出所有仓库
vcs list
```

## 1.3. 典型使用场景

### 1.3.1. YAML 配置文件示例 (`depend_repos.yml`)
```yaml
repositories:
  ros2/common_interfaces:
    type: git
    url: https://github.com/ros2/common_interfaces.git
    version: galactic
  
  navigation2:
    type: git
    url: https://github.com/ros-planning/navigation2.git
    version: main
  
  custom_package:
    type: git
    url: git@gitlab.example.com:team/custom_package.git
    version: develop
```

### 1.3.2. 常用命令

```bash
# 1. 导入仓库
vcs import src < repos.yaml

# 2. 拉取所有仓库的更新
vcs pull

# 3. 检出特定分支/标签
vcs checkout --repos ./src navigation2 main

# 4. 自定义操作（在所有仓库执行）
vcs custom --git --args "status"
```

## 1.4. 在你的脚本中的应用

在你的脚本中：
```bash
mkdir -p depend_repos && vcs import depend_repos < depend_repos.yml
```

这行代码：
1. 创建 `depend_repos` 目录
2. 使用 `vcs import` 命令从 `depend_repos.yml` 文件中读取配置
3. 克隆或检出所有指定的依赖仓库到 `depend_repos` 目录

## 1.5. 为什么 ROS 项目使用 VCS？

1. **模块化架构**：ROS 项目通常由多个独立的包组成
2. **依赖管理**：需要从不同来源获取各种驱动、算法包
3. **版本一致性**：确保团队所有成员使用相同版本的依赖
4. **简化工作流**：避免手动管理数十个 Git 仓库

## 1.6. 与 Git 子模块的对比

| 特性     | VCS Tools            | Git 子模块             |
| -------- | -------------------- | ---------------------- |
| 学习曲线 | 简单                 | 较陡峭                 |
| 灵活性   | 高，支持多种VCS      | 仅 Git                 |
| 配置方式 | YAML 文件            | .gitmodules            |
| ROS 集成 | 原生支持             | 需要额外配置           |
| 常用命令 | import, export, pull | submodule update, init |

## 1.7. 安装 VCS

```bash
# 在 Ubuntu/Debian 上
sudo apt-get install python3-vcstool

# 通过 pip 安装
pip install vcstool
```

## 1.8. 实际示例

假设你有一个 ROS 项目，依赖多个外部包：

```bash
# 创建依赖文件
cat > dependencies.repos << EOF
repositories:
  rviz:
    type: git
    url: https://github.com/ros-visualization/rviz.git
    version: ros2
  
  teleop_twist_keyboard:
    type: git
    url: https://github.com/ros-teleop/teleop_twist_keyboard.git
    version: ros2
EOF

# 导入所有依赖
vcs import src < dependencies.repos

# 现在 src/ 目录下会有两个仓库
# src/rviz/ 和 src/teleop_twist_keyboard/
```

VCS 工具使得管理复杂的 ROS 项目依赖变得非常简单和高效，特别是在需要确保团队所有成员使用完全相同版本的依赖时。

# 2. Google repo

**Google Repo 工具** 是 Google 为 Android 项目开发的一个**多仓库管理工具**，用于管理基于 Git 的多个代码仓库。它最初是为管理 Android 庞大的代码库而创建的。

## 2.1. 核心概念

### 2.1.1. 解决什么问题
Android 代码库包含数百个独立的 Git 仓库（框架、应用、内核、驱动等）。手动管理这些仓库非常困难：
- 需要同步所有仓库的特定版本
- 跨仓库提交代码
- 批量操作（拉取、推送、状态检查）

### 2.1.2. 基本工作原理

- 一个顶层的 **manifest 仓库**（包含 XML 文件）
- 多个独立的 **Git 仓库**
- Repo 作为包装器，批量操作所有仓库

## 2.2. 主要功能

### 2.2.1. 初始化工作区
```bash
# 下载 repo 工具
curl https://storage.googleapis.com/git-repo-downloads/repo > ~/bin/repo
chmod a+x ~/bin/repo

# 初始化 Android 源码树
repo init -u https://android.googlesource.com/platform/manifest -b android-14.0.0_r1
```

### 2.2.2. 同步所有仓库

```bash
# 下载所有代码（数百个仓库）
repo sync -j4
```

### 2.2.3. 常用命令

```bash
# 查看所有仓库状态
repo status

# 在所有仓库执行命令
repo forall -c 'git log --oneline -5'

# 上传更改到 Gerrit（代码审查）
repo upload

# 创建和管理分支
repo start feature-branch --all
```

## 2.3. Manifest 文件示例

**default.xml**（位于 .repo/manifests/）：
```xml
<?xml version="1.0" encoding="UTF-8"?>
<manifest>
  <remote name="aosp"
          fetch="https://android.googlesource.com/" />
  <default revision="master"
           remote="aosp"
           sync-j="4" />
  
  <!-- 项目定义 -->
  <project path="build/make" name="platform/build" />
  <project path="frameworks/base" name="platform/frameworks/base" />
  <project path="packages/apps/Settings" name="platform/packages/apps/Settings" />
  <project path="kernel/msm" name="kernel/msm" revision="android-msm-sunfish-4.14-android14" />
  
  <!-- 包含其他 manifest 文件 -->
  <include name="vendor/google.xml" />
</manifest>
```

## 2.4. Repo 工作流程示例

```bash
# 1. 初始化
repo init -u https://github.com/company/manifest.git -b main

# 2. 同步代码
repo sync

# 3. 创建分支
repo start feature/login-flow --all

# 4. 在多个仓库中修改代码
#    - 修改 frameworks/base
#    - 修改 packages/apps/Launcher

# 5. 查看状态
repo status

# 6. 提交更改
cd frameworks/base
git add .
git commit -m "Add login authentication"
cd ../../packages/apps/Launcher
git add .
git commit -m "Update login UI"

# 7. 批量上传审查
repo upload
```

## 2.5. Repo vs Git 子模块 vs VCS

| 特性              | Google Repo             | Git 子模块   | ROS VCS    |
| ----------------- | ----------------------- | ------------ | ---------- |
| **设计目标**      | 管理超大项目（Android） | 嵌入外部项目 | ROS 包管理 |
| **复杂性**        | 高                      | 中等         | 低         |
| **manifest 格式** | XML                     | .gitmodules  | YAML       |
| **跨仓库操作**    | 强大（forall, upload）  | 无           | 基本       |
| **代码审查集成**  | 原生支持 Gerrit         | 无           | 无         |
| **学习曲线**      | 陡峭                    | 中等         | 平缓       |

## 2.6. Repo 的高级特性

### 2.6.1. 分层 manifest
```xml
<!-- 基础 manifest -->
<include name="core.xml" />

<!-- 设备特定 manifest -->
<include name="devices/samsung.xml" />

<!-- 产品定制 manifest -->
<include name="products/pixel.xml" />
```

### 2.6.2. 工作区管理
```bash
# 列出所有 manifest 分支
repo manifests

# 清理工作区
repo forall -c 'git clean -xdf'

# 批量重置
repo forall -c 'git reset --hard'
```

### 2.6.3. Gerrit 集成
```bash
# 配置 Gerrit
git config --global review.gerrit.company.com.username yourname

# 上传并请求审查
repo upload --cbr --no-verify
```

## 2.7. 实际使用案例

### 2.7.1. AOSP (Android Open Source Project)

```bash
# 下载整个 Android 源码
repo init -u https://android.googlesource.com/platform/manifest
repo sync -j8

# 切换到特定版本
repo init -b android-14.0.0_r1
repo sync
```

### 2.7.2. 企业多仓库项目

```bash
# 公司内部项目
repo init -u ssh://git@company.com/platform/manifest -b release-2024

# 只同步部分仓库
repo sync platform/kernel platform/framework
```

## 2.8. Repo 的优势

1. **规模管理**：轻松管理数百个仓库
2. **版本一致性**：确保所有组件版本匹配
3. **批量操作**：一次性操作所有仓库
4. **灵活配置**：通过 XML manifest 精确控制
5. **审查流程**：内置代码审查支持

## 2.9. 安装和使用

```bash
# 安装 repo
mkdir -p ~/bin
curl https://storage.googleapis.com/git-repo-downloads/repo > ~/bin/repo
chmod a+x ~/bin/repo

# 添加到 PATH
export PATH="$HOME/bin:$PATH"

# 验证安装
repo version
```

**Google Repo** 是一个非常强大的工具，特别适合管理像 Android 这样的大型、复杂的多仓库项目。虽然学习曲线较陡，但对于需要精确控制多个相关代码库版本的企业级项目来说，它是不可或缺的工具。
