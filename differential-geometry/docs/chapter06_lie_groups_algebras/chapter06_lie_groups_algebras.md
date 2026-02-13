# 第6章 李群与李代数

## 本章导读

李群与李代数是描述连续对称性的数学工具，在机器人运动学、计算机视觉和理论物理中有广泛应用。本章介绍李群和李代数的基本概念、它们之间的对应关系，以及在机器人旋转和平移描述中的应用。通过本章学习，你将掌握SO(3)和SE(3)的数学结构，理解指数映射和对数映射的几何意义。

---

## 6.1 李群的基本概念

### 6.1.1 群的回顾

定义 6.1（群） 集合 $G$ 配备二元运算 $\cdot$ 称为**群**，如果满足：
1. **封闭性**：$a, b \in G \Rightarrow a \cdot b \in G$
2. **结合律**：$(a \cdot b) \cdot c = a \cdot (b \cdot c)$
3. **单位元**：存在 $e \in G$ 使得 $e \cdot a = a \cdot e = a$
4. **逆元**：对每个 $a \in G$，存在 $a^{-1} \in G$ 使得 $a \cdot a^{-1} = e$

### 6.1.2 李群的定义

定义 6.2（李群） **李群**（Lie Group）是同时具有群结构和光滑流形结构的集合，且群运算（乘法和取逆）是光滑映射。

**关键性质**
- 李群是光滑流形
- 群乘法 $G \times G \to G$ 是光滑的
- 取逆 $G \to G$ 是光滑的

### 6.1.3 常见的李群

**一般线性群 GL(n, ℝ)**

$$GL(n, \mathbb{R}) = \{A \in M_{n \times n}(\mathbb{R}) : \det A \neq 0\}$$

- 群运算：矩阵乘法
- 维数：$n^2$

**特殊线性群 SL(n, ℝ)**

$$SL(n, \mathbb{R}) = \{A \in GL(n, \mathbb{R}) : \det A = 1\}$$

**正交群 O(n)**

$$O(n) = \{R \in GL(n, \mathbb{R}) : R^T R = I\}$$

保持内积不变：$(Rx) \cdot (Ry) = x \cdot y$

**特殊正交群 SO(n)**

$$SO(n) = \{R \in O(n) : \det R = 1\}$$

表示 $n$ 维空间的旋转。

**欧几里得群 E(n)**

$$E(n) = \left\{\begin{pmatrix} R & t \\ 0 & 1 \end{pmatrix} : R \in O(n), t \in \mathbb{R}^n\right\}$$

**特殊欧几里得群 SE(n)**

$$SE(n) = \left\{\begin{pmatrix} R & t \\ 0 & 1 \end{pmatrix} : R \in SO(n), t \in \mathbb{R}^n\right\}$$

表示 $n$ 维空间的刚体变换（旋转+平移）。

---

## 6.2 李代数

### 6.2.1 李代数的定义

定义 6.3（李代数） **李代数**是向量空间 $\mathfrak{g}$ 配备双线性运算 $[\cdot, \cdot]: \mathfrak{g} \times \mathfrak{g} \to \mathfrak{g}$（称为**李括号**），满足：
1. **反对称性**：$[X, Y] = -[Y, X]$
2. **Jacobi恒等式**：$[X, [Y, Z]] + [Y, [Z, X]] + [Z, [X, Y]] = 0$

### 6.2.2 李群的李代数

**左不变向量场**

设 $G$ 是李群，$g \in G$。**左平移** $L_g: G \to G$ 定义为：
$$L_g(h) = gh$$

向量场 $X$ 称为**左不变的**，如果 $(L_g)_* X = X$ 对所有 $g \in G$。

**定义 6.4（李群的李代数）** 李群 $G$ 的**李代数** $\mathfrak{g}$ 是 $G$ 上所有左不变向量场的集合。

等价地，$\mathfrak{g} \cong T_e G$（单位元处的切空间）。

### 6.2.3 常见李群的李代数

**一般线性群的李代数 gl(n, ℝ)**

$$\mathfrak{gl}(n, \mathbb{R}) = M_{n \times n}(\mathbb{R})$$

李括号：$[X, Y] = XY - YX$（矩阵交换子）

**特殊线性群的李代数 sl(n, ℝ)**

$$\mathfrak{sl}(n, \mathbb{R}) = \{X \in M_{n \times n}(\mathbb{R}) : \text{tr}(X) = 0\}$$

**正交群的李代数 so(n)**

$$\mathfrak{so}(n) = \{X \in M_{n \times n}(\mathbb{R}) : X^T + X = 0\}$$

即斜对称矩阵。

**特殊欧几里得群的李代数 se(n)**

$$\mathfrak{se}(n) = \left\{\begin{pmatrix} \Omega & v \\ 0 & 0 \end{pmatrix} : \Omega \in \mathfrak{so}(n), v \in \mathbb{R}^n\right\}$$

---

## 6.3 指数映射与对数映射

### 6.3.1 指数映射

**矩阵指数**

定义 6.5（矩阵指数） 对于方阵 $X$，**矩阵指数**定义为：
$$\exp(X) = e^X = \sum_{k=0}^\infty \frac{X^k}{k!}$$

**指数映射**

定义 6.6（指数映射） **指数映射** $\exp: \mathfrak{g} \to G$ 定义为：
$$\exp(X) = \gamma(1)$$

其中 $\gamma(t)$ 是李群上满足 $\gamma(0) = e$，$\dot{\gamma}(0) = X$ 的单参数子群。

对于矩阵李群：
$$\exp(X) = e^X$$

### 6.3.2 对数映射

**对数映射**

在指数映射的逆映射存在的区域，定义**对数映射** $\log: G \to \mathfrak{g}$：
$$\log(g) = X \quad \text{其中} \quad e^X = g$$

**性质**

- 指数映射在单位元附近是局部微分同胚
- 对数映射在 $e$ 的某个邻域内有定义

### 6.3.3 SO(3)的指数映射

**so(3)的表示**

$\mathfrak{so}(3)$ 的元素可以表示为斜对称矩阵：
$$\Omega = \begin{pmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{pmatrix} = [\omega]_\times$$

其中 $\omega = (\omega_1, \omega_2, \omega_3)^T$。

**Rodrigues旋转公式**

对于 $\Omega \in \mathfrak{so}(3)$，$\theta = \|\omega\|$：
$$R = \exp(\Omega) = I + \frac{\sin\theta}{\theta}\Omega + \frac{1-\cos\theta}{\theta^2}\Omega^2$$

这是绕轴 $\omega/\|\omega\|$ 旋转角度 $\theta$ 的旋转矩阵。

### 6.3.4 SE(3)的指数映射

**se(3)的表示**

$\mathfrak{se}(3)$ 的元素：
$$\xi = \begin{pmatrix} \Omega & v \\ 0 & 0 \end{pmatrix} \in \mathfrak{se}(3)$$

可以用6维向量 $\xi = (\omega, v)^T$ 表示。

**指数映射公式**

$$\exp(\xi) = \begin{pmatrix} \exp(\Omega) & Vv \\ 0 & 1 \end{pmatrix}$$

其中：
$$V = I + \frac{1-\cos\theta}{\theta^2}\Omega + \frac{\theta-\sin\theta}{\theta^3}\Omega^2$$

---

## 6.4 伴随表示

### 6.4.1 伴随映射

定义 6.7（伴随映射） 对于 $g \in G$，**伴随映射** $\text{Ad}_g: \mathfrak{g} \to \mathfrak{g}$ 定义为：
$$\text{Ad}_g(X) = (L_g)_* (R_{g^{-1}})_* X = gXg^{-1}$$

（对于矩阵李群）

### 6.4.2 伴随表示

定义 6.8（伴随表示） **伴随表示** $Ad: G \to GL(\mathfrak{g})$ 定义为：
$$Ad(g) = \text{Ad}_g$$

### 6.4.3 李代数的伴随表示

定义 6.9（李代数的伴随表示） **李代数的伴随表示** $ad: \mathfrak{g} \to \mathfrak{gl}(\mathfrak{g})$ 定义为：
$$ad(X)(Y) = [X, Y]$$

**关系**

$$Ad(\exp(X)) = \exp(ad(X))$$

---

## 6.5 李群在机器人中的应用

### 6.5.1 旋转的表示

**旋转矩阵 SO(3)**

优点：
- 没有奇异点
- 复合旋转简单（矩阵乘法）

缺点：
- 9个参数，只有3个自由度（有约束）

**旋转向量（轴角）**

$\omega \in \mathbb{R}^3$，旋转角度 $\theta = \|\omega\|$，旋转轴 $\omega/\theta$。

与SO(3)的转换：
- $\omega \to R$：Rodrigues公式
- $R \to \omega$：对数映射

**欧拉角**

用三个角度表示旋转（如ZYX欧拉角：偏航-俯仰-翻滚）。

缺点：存在万向节锁（gimbal lock）奇异点。

**四元数**

单位四元数 $q = (w, x, y, z)$，$\|q\| = 1$。

与SO(3)的关系：双重覆盖 $S^3 \to SO(3)$。

### 6.5.2 刚体变换的表示

**齐次变换矩阵 SE(3)**

$$T = \begin{pmatrix} R & t \\ 0 & 1 \end{pmatrix} \in SE(3)$$

其中 $R \in SO(3)$，$t \in \mathbb{R}^3$。

** twist 表示**

使用 $\mathfrak{se}(3)$ 的元素 $\xi = (\omega, v)^T$ 表示刚体运动。

**指数坐标**

任何刚体变换可以表示为某个 twist 的指数：
$$T = \exp(\xi)$$

### 6.5.3 机器人运动学

**正向运动学**

对于串联机械臂：
$$T_{0n} = T_{01} T_{12} \cdots T_{(n-1)n}$$

其中每个 $T_{i(i+1)} \in SE(3)$。

**雅可比矩阵**

空间雅可比：
$$J^s = \begin{pmatrix} J_1^s & J_2^s & \cdots & J_n^s \end{pmatrix}$$

其中每一列是对应关节的 twist。

**速度传播**

$$V = J \dot{\theta}$$

其中 $V \in \mathfrak{se}(3)$ 是末端执行器的 twist。

---

## 6.6 本章小结

本章介绍了李群与李代数：

1. **李群**：
   - 群结构 + 光滑流形结构
   - 常见例子：GL(n), SO(n), SE(n)

2. **李代数**：
   - 李群的切空间（单位元处）
   - 李括号运算
   - 常见例子：gl(n), so(n), se(n)

3. **指数映射与对数映射**：
   - 连接李群和李代数
   - Rodrigues公式（SO(3)）
   - SE(3)的指数映射

4. **机器人应用**：
   - 旋转的多种表示
   - 刚体变换
   - 运动学建模

李群与李代数为机器人运动学提供了优雅的数学框架。

---

## 习题

### 基础题

6.1 证明SO(3)是李群。

6.2 验证so(3)在李括号 $[X,Y] = XY - YX$ 下构成李代数。

6.3 推导Rodrigues旋转公式。

6.4 计算给定旋转矩阵对应的旋转向量（对数映射）。

### 提高题

6.5 证明指数映射在单位元附近是局部微分同胚。

6.6 推导SE(3)的指数映射公式。

6.7 证明伴随表示的性质：$Ad([g,h]) = [Ad(g), Ad(h)]$。

6.8 用李群方法推导机械臂的正向运动学。

### 编程题

6.9 实现SO(3)的指数映射和对数映射。

6.10 实现机械臂正向运动学的李群计算方法。

---

## 参考文献

1. Murray R M, Li Z, Sastry S S. A Mathematical Introduction to Robotic Manipulation[M]. CRC Press, 1994.
2. Selig J M. Geometric Fundamentals of Robotics[M]. Springer, 2005.
3. Hall B C. Lie Groups, Lie Algebras, and Representations[M]. Springer, 2015.
4. Stillwell J. Naive Lie Theory[M]. Springer, 2008.
