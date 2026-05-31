<!--
 * @Author: JohnJeep
 * @Date: 2020-04-25 19:37:43
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 18:55:07
 * @Description: Page tables
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

- 逻辑地址空间是应用程序直接使用的地址空间。
- 段机制启动、页机制未启动：逻辑地址->段机制处理->线性地址=物理地址
- 段机制启动、页机制都未启动：逻辑地址->段机制处理->线性地址->页机制处理->物理地址
- 页表(page table): 存储虚拟地址到物理地址的映射(mapping)，是一种数据结构。每一个映射称为页表项(PTE: page table entry)。








