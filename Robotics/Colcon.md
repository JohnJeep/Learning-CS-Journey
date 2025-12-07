## colcon

### create

```bash
# 创建新包（Python）
ros2 pkg create my_python_pkg --build-type ament_python

# 创建新包（C++）
ros2 pkg create my_cpp_pkg --build-type ament_cmake


```

### build

```bash
# 构建新包
colcon build --packages-select my_python_pkg

# 构建并安装到 install 目录
colcon build --symlink-install

# 构建时允许并行处理
colcon build --parallel-workers 8

# 只构建有变化的包
colcon build

# 清理构建缓存
colcon build --cmake-clean-cache

# 构建指定类型的包（CMake 或 Python）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select my_package --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 继续构建，即使某些包失败
colcon build --continue-on-error
```

### test

```bash
# 构建并运行测试
colcon test

# 运行特定包的测试
colcon test --packages-select package_name

# 查看测试结果
colcon test-result --verbose
```

### 环境设置

构建后需要 source 安装目录的 setup 文件：

```bash
# 在 ~/.bashrc 中添加
source ~/ros2_ws/install/setup.bash

# 或手动 source
source install/local_setup.bash  # 只 source 当前工作空间
source install/setup.bash        # source 所有工作空间
```