<!--
 * @Author: JohnJeep
 * @Date: 2026-06-06 21:22:29
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-06-06 21:26:12
 * @Description: WSL2 filesystem only read fix manual
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

## WSL2 文件系统只读修复手册

### 快速判断

```bash
dmesg | grep -E "error|readonly" | tail -5
```

出现 `mounting fs with errors` → 需要 e2fsck 修复。

------

### 修复步骤

**第一步：确认 Ubuntu 的设备号**

```powershell
wsl -d Ubuntu-22.04 --exec lsblk
```

找到挂载在 `/` 的设备，记下来（本机是 `sdc`）。

**第二步：安装 Debian（只需第一次）**

```powershell
wsl --install -d Debian
```

**第三步：用 vhd 方式挂载 Ubuntu 的磁盘**

```powershell
wsl --shutdown
Start-Sleep -Seconds 5
wsl --mount --vhd "I:\wsl2\ext4.vhdx" --bare
```

> vhdx 路径按实际情况替换。

**第四步：确认新出现的设备号**

```powershell
wsl -d Debian --exec lsblk
```

找到 1T、**没有挂载点**的那个设备（本机是 `sdc`）。

**第五步：修复**

```powershell
wsl -d Debian --exec sudo e2fsck -fy /dev/sdc
```

看到 `FILE SYSTEM WAS MODIFIED` 说明修复成功。

**第六步：重启验证**

```powershell
wsl --shutdown
wsl -d Ubuntu-22.04
dmesg | grep "EXT4-fs" | head -3
# 正常：mounted filesystem with ordered data mode（无 errors 字样）
```

------

### 预防

```powershell
# Windows 关机前先执行
wsl --shutdown
```