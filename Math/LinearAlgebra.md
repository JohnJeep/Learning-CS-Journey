<!--
 * @Author: JohnJeep
 * @Date: 2025-12-30 23:24:02
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-01-05 15:58:38
 * @Description: Linear Algebra
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->
# 1. Introduction to Linear Algebra

线性代数的鼻祖：美国 MIT 的 **Gilbert Strang** 教授。


线性代数的核心概念包括：向量、矩阵、线性方程组、矩阵乘法、矩阵的逆、行列式、特征值和特征向量。

# 2. Terms

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





# 3. 练习

实现QR分解、LU分解、Cholesky分解

用SVD实现图像压缩

实现共轭梯度法求解大型稀疏系统

用齐次坐标实现3D变换

实现卡尔曼滤波器（协方差矩阵更新）


# 4. python 语言实践

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



置换矩阵(transpose matrix)：就是用来交换矩阵行或列的矩阵。


矩阵乘法运算规则：
- 矩阵乘法是行列式的乘积。
- 矩阵乘法不满足交换律，但满足结合律和分配律。
- 矩阵乘法的结果是一个新的矩阵，其行数等于第一个矩阵的行数，列数等于第二个矩阵的列数。



matrix * columns = columns
matrix * rows = rows 

columns * rows = matrix





逆矩阵(inverse matrix)：对于一个方阵A，如果存在一个矩阵B，使得AB=BA=I（单位矩阵），则称B为A的逆矩阵，记作A^(-1)。逆矩阵的性质包括：
- 逆矩阵的逆矩阵是它本身。
- 逆矩阵的乘积等于单位矩阵。
- 逆矩阵的存在性：只有当矩阵是非奇异的（行列式不为零）时，才存在逆矩阵。



- 逆矩阵的计算可以通过高斯消元法、伴随矩阵法等方法实现。
- 逆矩阵的应用包括求解线性方程组、计算特征值和特征向量等。
- 逆矩阵的性质包括：逆矩阵的逆矩阵是它本身，逆矩阵的乘积等于单位矩阵，逆矩阵的存在性（只有当矩阵是非奇异的，即行列式不为零时，才存在逆矩阵），逆矩阵的计算可以通过高斯消元法、伴随矩阵法等方法实现。


方阵(square matrix)：行数和列数相等的矩阵称为方阵。方阵具有以下性质：
- 方阵的行列式(det(A))是一个标量，表示矩阵的可逆性。
- 方阵的特征值和特征向量用于描述矩阵的性质。
- 方阵的迹(trace(A))是矩阵对角线元素之和。
- 方阵的秩(rank(A))表示矩阵中线性无关行或列的数量。
- 方阵的逆矩阵存在当且仅当行列式不为零。




行列式(determinant)：行列式是一个标量值，用于描述方阵的性质。行列式的计算方法包括：
- 二阶行列式：对于2x2矩阵A = [[a, b], [c, d]]，行列式det(A) = ad - bc。
- 三阶及以上行列式：可以使用展开法、拉普拉斯展开等方法计算。
- 行列式的性质包括：行列式的乘积等于行列式的乘积，行列式的转置等于原行列式，行列式的行交换会改变符号等。
- 行列式的应用包括判断矩阵的可逆性（行列式不为零时矩阵可逆）、计算特征值和特征向量等。



# 5. references

- 18.06 Linear Algebra - Gilbert Strang: https://web.mit.edu/18.06/www/
- github 18.06: https://github.com/mitmath/1806

---
notes

- MIT 18.06 线性代数笔记: https://github.com/apachecn/mit-18.06-linalg-notes
- MIT-Linear-Algebra-Notes(pdf): https://github.com/MLNLP-World/MIT-Linear-Algebra-Notes
- MIT 18.06 线性代数笔记: https://linalg.apachecn.org/
- cs-self-learning: https://github.com/pkuflyingpig/cs-self-learning/


