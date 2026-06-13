<!--
 * @Author: JohnJeep
 * @Date: 2026-01-31 17:58:24
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-01-31 20:31:59
 * @Description: math formulas
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

# math formulas

数学中常用的三角函数公式以及特殊角对应的数值：

正弦和角公式（sine addition formula）：

$\sin(A + B) = \sin A \cos B + \cos A \sin B$

余弦和角公式（cosine addition formula）：

$\cos(A + B) = \cos A \cos B - \sin A \sin B$

正切和角公式（tangent addition formula）：

$\tan(A + B) = \frac{\tan A + \tan B}{1 - \tan A \tan B}$

✅ 推导过程：

我们可以从正弦和余弦的和角公式出发：
- $\sin(A + B) = \sin A \cos B + \cos A \sin B$
- $\cos(A + B) = \cos A \cos B - \sin A \sin B$

因为 $\tan(A + B) = \dfrac{\sin(A + B)}{\cos(A + B)}$，所以：

$\tan(A + B) = \frac{\sin A \cos B + \cos A \sin B}{\cos A \cos B - \sin A \sin B}$

分子分母同时除以 $\cos A \cos B$（假设 $\cos A \ne 0$ 且 $\cos B \ne 0$）：

分子： $\frac{\sin A \cos B}{\cos A \cos B} + \frac{\cos A \sin B}{\cos A \cos B} = \tan A + \tan B$

分母： $\frac{\cos A \cos B}{\cos A \cos B} - \frac{\sin A \sin B}{\cos A \cos B} = 1 - \tan A \tan B$

因此： $\tan(A + B) = \frac{\tan A + \tan B}{1 - \tan A \tan B}$


🔁 差角公式（令 $B \to -B$）：

由于 $\tan(-B) = -\tan B$，代入上式可得： $\tan(A - B) = \frac{\tan A - \tan B}{1 + \tan A \tan B}$

⚠️ 注意事项：
- 公式成立的前提是所有涉及的正切值都有定义（即角度不能是 $\frac{\pi}{2} + k\pi$，其中 $k$ 为整数）。
- 分母 $1 - \tan A \tan B \ne 0$，否则 $\tan(A + B)$ 无定义（对应 $A + B = \frac{\pi}{2} + k\pi$）。

 📌 示例：

计算 $\tan\left(\frac{\pi}{4} + \frac{\pi}{6}\right)$：

$\tan\frac{\pi}{4} = 1, \tan\frac{\pi}{6} = \frac{1}{\sqrt{3}}$

代入公式：

$\tan\left(\frac{\pi}{4} + \frac{\pi}{6}\right) = \frac{1 + \frac{1}{\sqrt{3}}}{1 - 1 \cdot \frac{1}{\sqrt{3}}}
= \frac{\frac{\sqrt{3} + 1}{\sqrt{3}}}{\frac{\sqrt{3} - 1}{\sqrt{3}}}
= \frac{\sqrt{3} + 1}{\sqrt{3} - 1}$

有理化后可得 $2 + \sqrt{3}$，这与 $\tan\left(\frac{5\pi}{12}\right)$ 一致。


**🔢 特殊角三角函数值表（0° ～ 360°）**

以下是 **常见角度**（0° 到 360°）的 **正弦**（sin）、**余弦**（cos） 和 **正切**（tan）
值表。这些角度通常是特殊角（如 0°, 30°, 45°, 60°, 90° 等）及其在四个象限中的对应角。

| 角度 θ | 弧度              | $\sin\theta$         | $\cos\theta$          | $\tan\theta$                                |
| ------ | ---------------- | ---------------------- | ----------------------- | --------------------------------------------- |
| 0°     | $0$             | $0$                   | $1$                   | $0$                                       |
| 30°    | $\frac{\pi}{6}$  | $\frac{1}{2}$         | $\frac{\sqrt{3}}{2}$  | $\frac{1}{\sqrt{3}} = \frac{\sqrt{3}}{3}$   |
| 45°    | $\frac{\pi}{4}$  | $\frac{\sqrt{2}}{2}$  | $\frac{\sqrt{2}}{2}$  | $1$                                         |
| 60°    | $\frac{\pi}{3}$  | $\frac{\sqrt{3}}{2}$  | $\frac{1}{2}$         | $\sqrt{3}$                                  |
| 90°    | $\frac{\pi}{2}$  | $1$                   | $0$                   | —（无定义）                                   |
| 120°   | $\frac{2\pi}{3}$ | $\frac{\sqrt{3}}{2}$  | $-\frac{1}{2}$        | $-\sqrt{3}$                                 |
| 135°   | $\frac{3\pi}{4}$ | $\frac{\sqrt{2}}{2}$  | $-\frac{\sqrt{2}}{2}$ | $-1$                                        |
| 150°   | $\frac{5\pi}{6}$ | $\frac{1}{2}$         | $-\frac{\sqrt{3}}{2}$ | $-\frac{1}{\sqrt{3}} = -\frac{\sqrt{3}}{3}$ |
| 180°   | $\pi$            | $0$                   | $-1$                  | $0$                                         |
| 210°   | $\frac{7\pi}{6}$  | $-\frac{1}{2}$        | $-\frac{\sqrt{3}}{2}$ | $\frac{1}{\sqrt{3}} = \frac{\sqrt{3}}{3}$   |
| 225°   | $\frac{5\pi}{4}$  | $-\frac{\sqrt{2}}{2}$ | $-\frac{\sqrt{2}}{2}$ | $1$                                         |
| 240°   | $\frac{4\pi}{3}$  | $-\frac{\sqrt{3}}{2}$ | $-\frac{1}{2}$        | $-\sqrt{3}$                                  |
| 270°   | $\frac{3\pi}{2}$  | $-1$                  | $0$                   | —（无定义）                                   |
| 300°   | $\frac{5\pi}{3}$  | $-\frac{\sqrt{3}}{2}$ | $\frac{1}{2}$         | $-\sqrt{3}$                                  |
| 315°   | $\frac{7\pi}{4}$  | $-\frac{\sqrt{2}}{2}$ | $\frac{\sqrt{2}}{2}$  | $-1$                                         |
| 330°   | $\frac{11\pi}{6}$ | $-\frac{1}{2}$        | $\frac{\sqrt{3}}{2}$  | $-\frac{1}{\sqrt{3}} = -\frac{\sqrt{3}}{3}$ |
| 360°   | $2\pi$            | $0$                   | $1$                   | $0$                                         |
