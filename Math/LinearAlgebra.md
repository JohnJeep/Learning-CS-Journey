<!--
 * @Author: JohnJeep
 * @Date: 2025-12-30 23:24:02
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-01-06 00:29:57
 * @Description: Linear Algebra
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->
# 1. introduction to linear algebra

线性代数的鼻祖：美国 MIT 的 **Gilbert Strang** 教授。


线性代数的核心概念包括：向量、矩阵、线性方程组、矩阵乘法、矩阵的逆、行列式、特征值和特征向量。

# 2. terms

以下是线性代数中常用术语的中英对照表：

| 英文术语                         | 中文术语              |
| :------------------------------- | :-------------------- |
| Linear Algebra                   | 线性代数              |
| Scalar                           | 标量                  |
| Vector                           | 向量                  |
| Aligned vector                   | 同向向量              |
| anti-aligned vector              | 反向向量              |
| Matrix                           | 矩阵                  |
| Set                              | 集合                  |
| System of Linear Equations       | 线性方程组            |
| Solution                         | 解                    |
| combination                      | 组合                  |
| sparse                           | 稀疏                  |
| residual                         | 残差                  |
| de-meaning                       | 去均值                |
| dependent variable               | 因变量                |
| independent variable             | 自变量                |
| distance                         | 距离                  |
| inequality                       | 不等式                |
| intercept                        | 截距                  |
| interpolation                    | 插值                  |
| evalution                        | 求值                  |
| multiplier                       | 乘数                  |
| polynomial                       | 多项式                |
| symmetric                        | 对称                  |
| specificity                      | 奇异性                |
| spherical                        | 球面距离              |
| nonnegative                      | 非负                  |
| **Matrix Related**               | **矩阵相关**          |
| Row                              | 行                    |
| Column                           | 列                    |
| Entry / Element                  | 元素                  |
| gradient                         | 梯度                  |
| Square Matrix                    | 方阵                  |
| Identity Matrix                  | 单位矩阵              |
| Zero Matrix / Null Matrix        | 零矩阵                |
| Diagonal Matrix                  | 对角矩阵              |
| transpose                        | 转置                  |
| Symmetric Matrix                 | 对称矩阵              |
| Inverse Matrix                   | 逆矩阵                |
| Singular Matrix                  | 奇异矩阵 / 不可逆矩阵 |
| Nonsingular Matrix               | 非奇异矩阵 / 可逆矩阵 |
| Determinant                      | 行列式                |
| Trace                            | 迹                    |
| Adjacency Matirx                 | 邻接矩阵              |
| Return Matrix                    | 收益矩阵              |
| circular difference matrix       | 循环差分矩阵          |
| column major                     | 列优先                |
| compliance matrix                | 柔度矩阵              |
| confusion matrix                 | 混淆矩阵              |
| controllability matrix           | 可控矩阵              |
| covariance matrix                | 协方差矩阵            |
| Augmented matrix                 | 增广矩阵              |
| power of matrix                  | 矩阵的幂              |
| second difference matrix         | 二阶差分矩阵          |
| tall matrix                      | 高形矩阵              |
|                                  |                       |
| **Vector Space Related**         | **向量空间相关**      |
| Vector Space                     | 向量空间              |
| unit vector                      | 单位向量              |
| column space                     | 列空间                |
| row space                        | 行空间                |
| cross product                    | 叉乘                  |
| dot product                      | 点积                  |
| inner product                    | 内积                  |
| outer product                    | 外积                  |
| projection                       | 投影                  |
| proportion                       | 比例                  |
| cross-validation                 | 交叉验证              |
| Subspace                         | 子空间                |
| Linear Combination               | 线性组合              |
| linear equation                  | 线性方程组            |
| homogeneous equation             | 齐次方程组            |
| Span                             | 生成空间              |
| Linear Independence              | 线性无关              |
| Linear Dependence                | 线性相关              |
| versus Linear                    | 与线性                |
| Basis                            | 基                    |
| dual                             | 对偶                  |
| Dimension                        | 维度                  |
| Coordinate                       | 坐标                  |
| location vector                  | 位置向量              |
| reverse                          | 逆序                  |
| rotation                         | 旋转                  |
| trajectory                       | 轨迹                  |
|                                  |                       |
|                                  |                       |
| **Linear Transformation**        | **线性变换**          |
| Linear Transformation            | 线性变换              |
| Kernel / Null Space              | 核空间 / 零空间       |
| Image / Column Space             | 像空间 / 列空间       |
| Rank                             | 秩                    |
| Nullity                          | 零化度                |
|                                  |                       |
|                                  |                       |
| **Eigenvalues and Eigenvectors** | **特征值与特征向量**  |
| Eigenvalue                       | 特征值                |
| Eigenvector                      | 特征向量              |
| Characteristic Polynomial        | 特征多项式            |
| Diagonalization                  | 对角化                |
| Similar Matrices                 | 相似矩阵              |
|                                  |                       |
| **Other Important Concepts**     | **其他重要概念**      |
| Inner Product / Dot Product      | 内积 / 点积           |
| Orthogonal                       | 正交                  |
| Orthonormal                      | 标准正交              |
| Projection                       | 投影                  |
| Gram-Schmidt Process             | 格拉姆-施密特过程     |
| Least Squares Solution           | 最小二乘解            |
| Least norm                       | 最小范数              |
| QR factorization                 | QR分解                |
| acute angle                      | 锐角                  |
| obtuse angle                     | 钝角                  |
| orthogonal angle                 | 直角                  |
| triangle                         | 三角                  |
| approximation                    | 近似                  |
| backslash notation               | 反斜杠记号            |
|                                  |                       |
| calculus                         | 微积分                |
| categorical feature              | 分类特征              |
| optimal                          | 最优                  |
| coefficient                      | 系数                  |
| complexity                       | 复杂度                |
| conservation of mass             | 质量守恒              |
| state feedback                   | 状态反馈              |
| correlation coefficient          | 相关系数              |
| convolution                      | 卷积                  |
| data fitting                     | 数据拟合              |
| derivative                       | 导数                  |
| dynamic                          | 动力学                |
| supply chain                     | 供应链                |
| mechanical                       | 力学的                |
| Gaussian elimination             | 高斯消元法            |
| LU decomposition                 | LU 分解               |
| QR decomposition                 | QR 分解               |
| loss function                    | 损失函数              |
| over-fit                         | 过拟合                |
| stratified                       | 分层                  |
| partial derivative               | 偏导数                |
| square root                      | 平方根                |
| sum of squares                   | 平方和                |
| root mean square                 | 均方根                |
| round-off error                  | 舍入误差              |
|                                  |                       |
|                                  |                       |


- 线性组合(linear combination)：由向量通过加权求和得到的新向量。
- 线性无关(linearly independent)：一组向量中没有一个可以表示为其他向量的线性组合。
- 线性相关(linearly dependent)：一组向量中至少有一个可以表示为其他向量的线性组合。
- 基(basis)：一组线性无关的向量，可以生成整个向量空间。
- 维度(dimension)：向量空间中基的数量，表示空间的大小。
- 内积(inner product)：两个向量的乘积，表示它们之间的关系。
- 外积(outer product)：两个向量的乘积，生成一个矩阵。
- 正交(orthogonal)：两个向量的内积为零，表示它们相互垂直。
- 标准正交(orthonormal)：一组向量既正交又单位化。
- 投影(projection)：将一个向量映射到另一个向量上的过程。
- 范数(norm)：向量的长度或大小，通常使用欧几里得范数。
- 距离(distance)：两个向量之间的距离，通常使用欧几里得距离。
- 叉乘(cross product)：仅适用于三维空间的向量运算，生成一个新的向量。
- 点积(dot product)：两个向量的乘积，生成一个标量。
- 齐次坐标(homogeneous coordinates)：用于表示点和向量的扩展坐标系，常用于计算机图形学。
- 轨迹(trajectory)：表示物体运动路径的向量序列。
- 旋转(rotation)：表示向量在空间中旋转的操作。
- 比例(proportion)：表示向量之间的比例关系。
- 组合(combination)：表示向量通过加权求和得到的新向量。
- 投影(projection)：表示将一个向量映射到另一个向量上的过程。
- 反斜杠记号(backslash notation)：用于表示线性方程组的求解方法。
- 最小二乘解(least squares solution)：用于求解过定线性方程组的近似解。
- 最小范数(least norm)：表示向量的最小长度或大小。
- 锐角(acute angle)：两个向量之间的夹角小于90度。
- 钝角(obtuse angle)：两个向量之间的夹角大于90度。
- 直角(orthogonal angle)：两个向量之间的夹角等于90度。
- 近似(approximation)：表示向量的近似值或近似解。
- 微积分(calculus)：用于描述向量变化的数学工具。
- 最优(optimal)：表示向量在某种意义下的最佳状态。
- 系数(coefficient)：表示向量在某种线性组合中的权重。
- 复杂度(complexity)：表示向量运算的计算复杂度。
- 导数(derivative)：表示向量变化率的数学工具。
- 偏导数(partial derivative)：表示多变量向量函数中某一变量的变化率。
- 平方根(square root)：表示向量长度的平方根。
- 均方根(root mean square)：表示向量长度的均方根。
- 舍入误差(round-off error)：表示向量运算中的数值误差。
- 数据拟合(data fitting)：表示通过向量数据进行拟合的过程。
- 卷积(convolution)：表示向量之间的卷积运算。
- 相关系数(correlation coefficient)：表示两个向量之间的相关关系。
- 状态反馈(state feedback)：表示通过向量状态进行反馈控制的过程。
- 质量守恒(conservation of mass)：表示向量在物理系统中的质量守恒性质。
- 力学的(mechanical)：表示与向量相关的力学性质。
- 动力学(dynamic)：表示与向量相关的动力学性质。
- 供应链(supply chain)：表示与向量相关的供应链管理问题。
- 高斯消元法(Gaussian elimination)：用于求解线性方程组的向量运算方法。
- 格拉姆-施密特过程(Gram-Schmidt process)：用于构造正交向量组的向量运算方法。
- QR分解(QR factorization)：用于将矩阵分解为正交矩阵和上三角矩阵的向量运算方法。
- 损失函数(loss function)：用于衡量向量预测误差的函数。
- 过拟合(over-fit)：表示向量模型在训练数据上表现过好的现象。
- 分层(stratified)：表示向量数据按某种标准进行分层的过程。
- 分类特征(categorical feature)：表示向量数据中的分类变量。


# vectors

向量(vector)：向量是具有大小和方向的数学对象，通常表示为有序数对或数列。向量的基本类型包括：
- 零向量(zero vector)：所有分量均为零的向量，记作 0。
- 单位向量(unit vector)：长度为 1 的向量，通常用于表示方向。
- 位置向量(position vector)：表示从原点到某一点的向量。
- 方向向量(direction vector)：表示某一方向的向量。
- 列向量(column vector)：以列形式表示的向量。
- 行向量(row vector)：以行形式表示的向量。
- 反向量(negative vector)：与原向量方向相反的向量，记作 -v。
- 同向量(aligned vector)：与原向量方向相同的向量。

标量(scalar)：标量是一个单一的数值，用于表示大小或数量。标量的基本类型包括：
- 实数(real number)：表示连续数值的标量。
- 复数(complex number)：表示具有实部和虚部的标量。
- 整数(integer)：表示没有小数部分的标量。
- 有理数(rational number)：表示可以表示为两个整数之比的标量。
- 无理数(irrational number)：表示不能表示为两个整数之比的标量。
- 自然数(natural number)：表示非负整数的标量。

标量运算包括：
- 标量乘法(scalar multiplication)：标量与向量或矩阵的乘法运算。
- 标量减法(scalar subtraction)：标量与向量或矩阵的减法运算。
- 标量加法(scalar addition)：标量与向量或矩阵的加法运算。

向量 a, b 的内积定义为：

$$
a{^T}b = a \cdot b = a_1b_1 + a_2b_2 + ... + a_nb_n
$$


向量 a, b 的外积定义为：

$$
a \times b = 
\begin{vmatrix}
i & j & k \\
a_1 & a_2 & a_3 \\
b_1 & b_2 & b_3
\end{vmatrix}
$$

范数(euclid )定义：向量元素平方和的平方根。

$$
|| x || = \sqrt{x_1^2 + x_2^2 + ... + x_n^2}
$$

范数总是非负的，且只有在向量为零向量时范数为零。

向量 x, y 的范数距离

$$
dist(x, y) = || x - y ||
$$


向量 x, y 的夹角 θ 定义为

$$
cos(θ) = \frac{ x \cdot y }{ ||x|| \cdot ||y|| }
$$

向量 x, y 之间的夹角表示为: 

$$
{\angle(x, y)}
$$

向量 x 和 y 的正交表示为 

$$
{x \bot y}
$$


# matrices

矩阵是写在方括号或圆括号中的一组数字或符号，按行和列排列。矩阵用于表示线性变换和线性方程组。

一个 m x n 的矩阵用 方括号表示为：

$$
A = 
\begin{bmatrix}
a_{11} & a_{12} & ... & a_{1n} \\
a_{21} & a_{22} & ... & a_{2n} \\
... & ... & ... & ... \\
a_{m1} & a_{m2} & ... & a_{mn}
\end{bmatrix}
$$

一个 m x n 的矩阵用 圆括号表示为：

$$
A = 
\begin{pmatrix}
a_{11} & a_{12} & ... & a_{1n} \\
a_{21} & a_{22} & ... & a_{2n} \\
... & ... & ... & ... \\
a_{m1} & a_{m2} & ... & a_{mn}
\end{pmatrix}
$$

---
矩阵(matrix)：矩阵是一个由行和列组成的二维数组，用于表示线性变换和线性方程组。矩阵的基本类型包括：
- 行矩阵(row matrix)：只有一行的矩阵。
- 列矩阵(column matrix)：只有一列的矩阵。
- 方阵(square matrix)：行数和列数相等的矩阵。
- 高形矩阵(tall matrix)：行数大于列数的矩阵。
- 宽形矩阵(wide matrix)：列数大于行数的矩阵。
- 零矩阵(zero matrix)：所有元素均为零的矩阵。
- 单位矩阵(identity matrix)：对角线元素为 1，其余元素为 0 的方阵，记作 I。
- 对角矩阵(diagonal matrix)：非对角线元素均为零的矩阵。
- 上三角矩阵(upper triangular matrix)：所有非零元素位于主对角线及其上方的矩阵。
- 下三角矩阵(lower triangular matrix)：所有非零元素位于主对角线及其下方的矩阵。
- 对称矩阵(symmetric matrix)：转置矩阵等于原矩阵的矩阵，即 $A^T = A$。
- 特征矩阵(eigen matrix)：用于表示特征值和特征向量的矩阵。
- 稠密矩阵(dense matrix)：大部分元素非零的矩阵。
- 稀疏矩阵(sparse matrix)：大部分元素为零的矩阵。
- 满秩矩阵(full rank matrix)：秩等于其行数或列数的矩阵。
- 秩缺矩阵(rank-deficient matrix)：秩小于其行数或列数的矩阵。
- 奇异矩阵(singular matrix)：不可逆的方阵，其行列式为零。
- 非奇异矩阵(nonsingular matrix)：可逆的方阵，其行列式不为零。
- 伴随矩阵(adjoint matrix)：由矩阵的代数余子式组成的矩阵。
- 余子式矩阵(minor matrix)：通过删除矩阵的某一行和某一列得到的子矩阵。
- 余因子矩阵(cofactor matrix)：由矩阵的余子式乘以相应的符号组成的矩阵。
- 转置矩阵(transpose matrix)：将矩阵的行和列互换得到的矩阵，记作 $A^T$。
- 置换矩阵(permutation matrix)：通过对单位矩阵的行或列进行置换得到的矩阵。
- 旋转矩阵(rotation matrix)：用于表示空间中旋转变换的矩阵。
- 分块矩阵(block matrix)：由多个子矩阵组成的矩阵。
- 增广矩阵(augmented matrix)：将线性方程组的系数矩阵和常数项矩阵合并得到的矩阵。

---
一个 3x3 的单位矩阵表示为：

$$
I = 
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

---
一个 3x3 的零矩阵表示为：

$$
0 = 
\begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
$$

---
一个 3x3 的对角矩阵表示为：

$$
D = 
\begin{bmatrix}
d_{11} & 0 & 0 \\
0 & d_{22} & 0 \\
0 & 0 & d_{33}
\end{bmatrix}
$$

一个对角矩阵既是上三角矩阵又是下三角矩阵。

---
一个 3x3 的上三角矩阵表示为：

$$
U = 
\begin{bmatrix}
u_{11} & u_{12} & u_{13} \\
0 & u_{22} & u_{23} \\
0 & 0 & u_{33}
\end{bmatrix}
$$

---
一个 3x3 的下三角矩阵表示为：

$$
L = 
\begin{bmatrix}
l_{11} & 0 & 0 \\
l_{21} & l_{22} & 0 \\
l_{31} & l_{32} & l_{33}
\end{bmatrix}
$$


## matrix addition

矩阵加法：两个大小相同的矩阵对应元素相加。

$$
C = A + B
$$

其中 $C_{ij} = A_{ij} + B_{ij}$。


## matrix subtraction

矩阵减法：两个大小相同的矩阵对应元素相减。

$$
C = A - B
$$

其中 $C_{ij} = A_{ij} - B_{ij}$。


## matrix multiplication

矩阵乘法：矩阵 A 的列数必须等于矩阵 B 的行数。矩阵乘法的结果是一个新的矩阵 C，其行数等于矩阵 A 的行数，列数等于矩阵 B 的列数。

$$
C = AB
$$

其中 $C_{ij} = \sum_{k=1}^{n} A_{ik} B_{kj}$。

矩阵乘法运算规则：
- 矩阵乘法是行列式的乘积。
- 矩阵乘法不满足交换律，但满足结合律和分配律。若果 A、B、C 是矩阵，则有：
  - 结合律：(AB)C = A(BC)
  - 分配律：A(B + C) = AB + AC 和 (A + B)C = AC + BC
  - 标量乘法与矩阵乘法满足结合律：k(AB) = (kA)B = A(kB)，其中 k 是标量。
  - 单位矩阵 I 满足 AI = IA = A。
  - 若两个矩阵 A 和 B 满足 AB = BA，则称它们是可交换的，当这种条成立时，A 和 B 必然是 方阵。
- 矩阵乘法的结果是一个新的矩阵，其行数等于第一个矩阵的行数，列数等于第二个矩阵的列数。

```
matrix * columns = columns
matrix * rows = rows
columns * rows = matrix
```

---
示例：计算矩阵乘法

$$
A = 
\begin{bmatrix}
1 & 2 & 3 \\
4 & 5 & 6
\end{bmatrix}
, \quad
B = 
\begin{bmatrix}7 & 8 \\
9 & 10 \\
11 & 12
\end{bmatrix}
$$

则

$$
\begin{aligned}
C &= AB\\
C_{11} &= 1\times7 + 2\times9 + 3\times11 = 58\\
C_{12} &= 1\times8 + 2\times10 + 3\times12 = 64\\
C_{21} &= 4\times7 + 5\times9 + 6\times11 = 139\\
C_{22} &= 4\times8 + 5\times10 + 6\times12 = 154\\
&\Rightarrow
C = 
\begin{bmatrix}
58 & 64\\
139 & 154
\end{bmatrix}
\end{aligned}
$$

---
矩阵与向量的乘法：矩阵 A 与向量 x 相乘，结果是一个新的向量 b。

$$
b = Ax
$$

其中 $b_i = \sum_{j=1}^{n} A_{ij} x_j$。

示例：

$$
A = 
\begin{bmatrix}
1 & 2 & 3 \\
4 & 5 & 6
\end{bmatrix}
, \quad
x = 
\begin{bmatrix}7 \\
8 \\
9
\end{bmatrix}
$$

则

$$
\begin{aligned}
b &= Ax\\
b_{1} &= 1\times7 + 2\times8 + 3\times9 = 50\\
b_{2} &= 4\times7 + 5\times8 + 6\times9 = 122\\
&\Rightarrow
b = 
\begin{bmatrix}
50 \\
122
\end{bmatrix}
\end{aligned}
$$


## transpose matrix

转置矩阵(transpose matrix)：将矩阵的行和列互换得到的矩阵，记作 $A^T$ (或 $A^{\prime}$ 或 $A^*$)。转置矩阵的性质包括：
- $(A^T)^T = A$
- $(A + B)^T = A^T + B^T$
- $(kA)^T = kA^T$（$k$为标量）
- $(AB)^T = B^T A^T$

示例：一个 2x3 的矩阵 A 

$$
A = 
\begin{bmatrix}
1 & 2 & 3 \\
4 & 5 & 6
\end{bmatrix}
$$

其转置为：

$$
A^T = 
\begin{bmatrix}
1 & 4 \\
2 & 5 \\
3 & 6
\end{bmatrix}
$$


## inverse matrix

逆矩阵(inverse matrix)：对于一个方阵A，如果存在一个矩阵B，使得AB=BA=I（单位矩阵），则称B为A的逆矩阵，记作 $A^{-1}$。

逆矩阵的性质包括：
- 逆矩阵的逆矩阵是它本身。
- 逆矩阵的乘积等于单位矩阵。
- 逆矩阵的存在性：只有当矩阵是非奇异的（行列式不为零）时，才存在逆矩阵。

---
- 逆矩阵的计算可以通过高斯消元法、伴随矩阵法等方法实现。
- 逆矩阵的应用包括求解线性方程组、计算特征值和特征向量等。
- 逆矩阵的性质包括：逆矩阵的逆矩阵是它本身，逆矩阵的乘积等于单位矩阵，逆矩阵的存在性（只有当矩阵是非奇异的，即行列式不为零时，才存在逆矩阵），逆矩阵的计算可以通过高斯消元法、伴随矩阵法等方法实现。


## square matrix

方阵(square matrix)：行数和列数相等的矩阵称为方阵。方阵具有以下性质：
- 方阵的行列式(det(A))是一个标量，表示矩阵的可逆性。
- 方阵的特征值和特征向量用于描述矩阵的性质。
- 方阵的迹(trace(A))是矩阵对角线元素之和。
- 方阵的秩(rank(A))表示矩阵中线性无关行或列的数量。
- 方阵的逆矩阵存在当且仅当行列式不为零。


## determinant

行列式(determinant)：行列式是一个标量值，用于描述方阵的性质。行列式的计算方法包括：
- 二阶行列式：对于2x2矩阵 $A = [[a, b], [c, d]]$，行列式 $\text{det}(A) = ad - bc$。
- 三阶及以上行列式：可以使用展开法、拉普拉斯展开等方法计算。
- 行列式的性质包括：行列式的乘积等于行列式的乘积，行列式的转置等于原行列式，行列式的行交换会改变符号等。
- 行列式的应用包括判断矩阵的可逆性（行列式不为零时矩阵可逆）、计算特征值和特征向量等。


## QR decomposition

QR 分解(QR decomposition)：将矩阵分解为一个正交矩阵Q和一个上三角矩阵R的乘积。QR分解的计算方法包括：
- 格拉姆-施密特过程(Gram-Schmidt process)：通过正交化向量组来构造Q矩阵。
- 豪斯霍尔德变换(Householder transformation)：通过反射变换来构造Q矩阵。
- 吉文斯旋转(Givens rotation)：通过旋转变换来构造Q矩阵。


QR分解的应用包括求解线性方程组、最小二乘问题等。

QR分解的性质包括：Q矩阵是正交矩阵，R矩阵是上三角矩阵，QR分解唯一性等。


# apply

## 3. practice

实现QR分解、LU分解、Cholesky分解

用SVD实现图像压缩

实现共轭梯度法求解大型稀疏系统

用齐次坐标实现3D变换

实现卡尔曼滤波器（协方差矩阵更新）


## 4. python library

需要使用的库

- numPy: 核心库，提供数组和矩阵运算
- matplotlib: 绘图库，用于可视化
- scipy: 基于NumPy，提供更多科学计算工具
- sympy: 符号计算，用于公式推导
- tensorflow: 深度学习库，支持张量运算
- pytorch: 深度学习库，支持动态计算图
- opencv: 计算机视觉库
- pyserial: 串口通信库
- pyusb: USB设备通信库


# 5. references

- 18.06 Linear Algebra - Gilbert Strang: https://web.mit.edu/18.06/www/
- github 18.06: https://github.com/mitmath/1806

---
notes

- MIT 18.06 线性代数笔记: https://github.com/apachecn/mit-18.06-linalg-notes
- MIT-Linear-Algebra-Notes(pdf): https://github.com/MLNLP-World/MIT-Linear-Algebra-Notes
- MIT 18.06 线性代数笔记: https://linalg.apachecn.org/
- cs-self-learning: https://github.com/pkuflyingpig/cs-self-learning/


