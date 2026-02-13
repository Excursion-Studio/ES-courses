# 第4章 微分流形

## 本章导读

微分流形是微分几何的核心研究对象，它是在拓扑流形的基础上增加了光滑结构。本章介绍微分流形的严格定义、光滑映射、切空间和余切空间等基础概念。通过本章学习，你将理解"光滑"的数学含义，掌握切向量的多种等价定义，为后续学习张量分析和黎曼几何奠定基础。

---

## 4.1 微分流形的定义和性质

### 4.1.1 光滑坐标卡

回顾：拓扑流形上的坐标卡 $(U, \varphi)$ 给出局部同胚 $\varphi: U \to \mathbb{R}^n$。

**光滑相容性**

定义 4.1（光滑相容） 设 $(U, \varphi)$ 和 $(V, \psi)$ 是拓扑流形 $M$ 上的两个坐标卡，且 $U \cap V \neq \varnothing$。如果坐标转换映射：
$$\psi \circ \varphi^{-1}: \varphi(U \cap V) \to \psi(U \cap V)$$
是光滑的（$C^\infty$），则称这两个坐标卡是**光滑相容的**（smoothly compatible）。

**注**：由于 $(\psi \circ \varphi^{-1})^{-1} = \varphi \circ \psi^{-1}$，光滑相容是对称关系。

### 4.1.2 微分流形的定义

定义 4.2（微分流形） 设 $M$ 是 $n$ 维拓扑流形。$M$ 上的**光滑结构**（smooth structure）是 $M$ 的一个图册 $\mathcal{A} = \{(U_\alpha, \varphi_\alpha)\}$，满足：

(D1) **覆盖性**：$\bigcup_\alpha U_\alpha = M$

(D2) **光滑相容性**：任意两个坐标卡光滑相容

(D3) **极大性**：$\mathcal{A}$ 是最大的满足 (D1) 和 (D2) 的图册

带有光滑结构的拓扑流形称为**微分流形**（differentiable manifold）或**光滑流形**（smooth manifold）。

**注**：
- 条件 (D3) 是技术性要求，确保光滑结构的唯一性
- 实际上，任何满足 (D1) 和 (D2) 的图册可以唯一地扩展为满足 (D3) 的光滑结构
- 维数 $n$ 称为流形的**维数**

### 4.1.3 微分流形的例子

**例 4.1 欧几里得空间**

$\mathbb{R}^n$ 是 $n$ 维微分流形，图册为 $\{(\mathbb{R}^n, \text{id})\}$。

**例 4.2 开子集**

设 $M$ 是微分流形，$W \subseteq M$ 是开子集，则 $W$ 也是微分流形。

**例 4.3 单位球面**

$S^n = \{x \in \mathbb{R}^{n+1} : \|x\| = 1\}$ 是 $n$ 维微分流形。

图册：用两个球极投影坐标卡
- 北极投影：$U_N = S^n \setminus \{(0, \ldots, 0, 1)\}$
- 南极投影：$U_S = S^n \setminus \{(0, \ldots, 0, -1)\}$

**例 4.4 一般线性群**

$GL(n, \mathbb{R}) = \{A \in M_{n \times n}(\mathbb{R}) : \det A \neq 0\}$ 是 $n^2$ 维微分流形。

这是矩阵李群的重要例子。

**例 4.5 实射影空间**

$\mathbb{R}P^n$（$\mathbb{R}^{n+1}$ 中所有过原点的直线）是 $n$ 维微分流形。

### 4.1.4 光滑函数

定义 4.3（光滑函数） 设 $M$ 是微分流形，$f: M \to \mathbb{R}$ 是函数。称 $f$ 是**光滑的**（smooth），如果对 $M$ 的每个坐标卡 $(U, \varphi)$，复合函数：
$$f \circ \varphi^{-1}: \varphi(U) \to \mathbb{R}$$
是光滑的（作为 $\mathbb{R}^n$ 开子集上的函数）。

所有光滑函数的集合记为 $C^\infty(M)$。

---

## 4.2 切空间和余切空间

### 4.2.1 切向量的几何直观

在欧几里得空间中，切向量可以看作：
- 从一点出发的有向线段
- 过该点的光滑曲线的速度向量
- 在该点可微函数的方向导数算子

在流形上，第一种定义（有向线段）不再适用，因为流形没有线性结构。我们需要使用后两种定义。

### 4.2.2 切向量的定义

**通过曲线定义**

设 $M$ 是微分流形，$p \in M$。

定义 4.4（光滑曲线） 映射 $\gamma: (-\epsilon, \epsilon) \to M$ 称为过 $p$ 的**光滑曲线**，如果 $\gamma(0) = p$ 且 $\gamma$ 是光滑映射。

**通过导子定义**

定义 4.5（导子/切向量） 映射 $v: C^\infty(M) \to \mathbb{R}$ 称为 $p$ 点的**切向量**（tangent vector），如果满足：

1. **线性性**：$v(af + bg) = av(f) + bv(g)$，$a, b \in \mathbb{R}$

2. **Leibniz 法则**：$v(fg) = f(p)v(g) + g(p)v(f)$

所有 $p$ 点切向量的集合记为 $T_p M$，称为 $p$ 点的**切空间**（tangent space）。

### 4.2.3 切空间的结构

**定理 4.1** $T_p M$ 是 $n$ 维实向量空间。

**坐标基**

设 $(U, \varphi)$ 是包含 $p$ 的坐标卡，坐标为 $(x^1, \ldots, x^n)$。

定义坐标切向量：
$$\left.\frac{\partial}{\partial x^i}\right|_p: C^\infty(M) \to \mathbb{R}$$
$$\left.\frac{\partial}{\partial x^i}\right|_p(f) = \frac{\partial(f \circ \varphi^{-1})}{\partial x^i}(\varphi(p))$$

定理 4.2 $\left\{\left.\frac{\partial}{\partial x^i}\right|_p\right\}_{i=1}^n$ 构成 $T_p M$ 的一组基，称为**坐标基**。

任意切向量可表示为：
$$v = \sum_{i=1}^n v^i \left.\frac{\partial}{\partial x^i}\right|_p$$

其中 $v^i = v(x^i)$ 是 $v$ 的**分量**。

### 4.2.4 余切空间

**余向量的定义**

定义 4.6（余切空间） $T_p M$ 的对偶空间称为**余切空间**（cotangent space），记为 $T_p^* M$：
$$T_p^* M = (T_p M)^* = \{\omega: T_p M \to \mathbb{R} : \omega \text{ 是线性的}\}$$

$T_p^* M$ 中的元素称为**余向量**（covector）或**1-形式**（1-form）。

**坐标余基**

对应于坐标基 $\left\{\frac{\partial}{\partial x^i}\right\}$，定义**坐标余基** $\{dx^i\}$：
$$dx^i\left(\frac{\partial}{\partial x^j}\right) = \delta^i_j = \begin{cases} 1 & i = j \\ 0 & i \neq j \end{cases}$$

任意余向量可表示为：
$$\omega = \sum_{i=1}^n \omega_i dx^i$$

### 4.2.5 切丛和余切丛

**切丛**

定义 4.7（切丛） 流形 $M$ 的**切丛**（tangent bundle）定义为：
$$TM = \bigsqcup_{p \in M} T_p M = \{(p, v) : p \in M, v \in T_p M\}$$

切丛 $TM$ 本身是 $2n$ 维微分流形。

**余切丛**

定义 4.8（余切丛） 流形 $M$ 的**余切丛**（cotangent bundle）定义为：
$$T^*M = \bigsqcup_{p \in M} T_p^* M$$

余切丛也是 $2n$ 维微分流形，在哈密顿力学中有重要应用。

---

## 4.3 向量场和张量场

### 4.3.1 向量场

定义 4.9（向量场） 流形 $M$ 上的**向量场** $X$ 是在每点 $p \in M$ 指定一个切向量 $X_p \in T_p M$ 的映射：
$$X: M \to TM, \quad p \mapsto X_p$$

要求：对任意 $f \in C^\infty(M)$，函数 $X(f): M \to \mathbb{R}$，$p \mapsto X_p(f)$ 是光滑的。

所有光滑向量场的集合记为 $\mathfrak{X}(M)$。

**局部表示**

在坐标卡 $(U, \varphi)$ 中，向量场可表示为：
$$X = \sum_{i=1}^n X^i \frac{\partial}{\partial x^i}$$

其中 $X^i$ 是光滑函数，称为 $X$ 的**分量**。

### 4.3.2 张量场

**张量的定义**

定义 4.10（张量） $(k, l)$-型**张量**是在每点定义的多重线性映射：
$$T: \underbrace{T_p^* M \times \cdots \times T_p^* M}_{k} \times \underbrace{T_p M \times \cdots \times T_p M}_{l} \to \mathbb{R}$$

- $k$：逆变阶数
- $l$：协变阶数

**张量场**

定义 4.11（张量场） **$(k, l)$-型张量场**是在每点 $p \in M$ 光滑地指定一个 $(k, l)$-型张量的映射。

**特殊张量场**：
- $(0, 0)$-型：光滑函数
- $(1, 0)$-型：向量场
- $(0, 1)$-型：1-形式场（余向量场）
- $(0, 2)$-型：双线性形式场（如黎曼度量）

### 4.3.3 张量场的运算

**缩并**

对 $(k, l)$-型张量，可以在一个上指标和一个下指标进行**缩并**（contraction），得到 $(k-1, l-1)$-型张量。

**张量积**

设 $S$ 是 $(k, l)$-型，$T$ 是 $(m, n)$-型，则**张量积** $S \otimes T$ 是 $(k+m, l+n)$-型：
$$(S \otimes T)(\omega_1, \ldots, \omega_{k+m}, X_1, \ldots, X_{l+n})$$
$$= S(\omega_1, \ldots, \omega_k, X_1, \ldots, X_l) \cdot T(\omega_{k+1}, \ldots, \omega_{k+m}, X_{l+1}, \ldots, X_{l+n})$$

---

## 4.4 本章小结

本章介绍了微分流形的基础理论：

1. **微分流形的定义**：
   - 光滑相容的坐标卡
   - 光滑结构：极大光滑图册
   - 例子：$\mathbb{R}^n$、$S^n$、$GL(n, \mathbb{R})$

2. **切空间和余切空间**：
   - 切向量：导子定义
   - 切空间 $T_p M$：$n$ 维向量空间
   - 坐标基：$\{\partial/\partial x^i\}$
   - 余切空间 $T_p^* M$：对偶空间
   - 切丛 $TM$ 和余切丛 $T^*M$

3. **向量场和张量场**：
   - 向量场：光滑的切向量赋值
   - 张量场：多重线性映射场
   - 张量运算：缩并、张量积

这些概念为后续学习李群、黎曼几何和流形优化奠定了严格的数学基础。

---

## 习题

### 基础题

4.1 验证球极投影给出的坐标卡是光滑相容的。

4.2 证明：切空间 $T_p M$ 是 $n$ 维向量空间。

4.3 在 $\mathbb{R}^2$ 中，设 $X = x\frac{\partial}{\partial y} - y\frac{\partial}{\partial x}$，$Y = \frac{\partial}{\partial x}$。计算 $X(f)$ 和 $Y(f)$，其中 $f(x, y) = x^2 + y^2$。

4.4 证明：余切空间 $T_p^* M$ 也是 $n$ 维向量空间。

### 提高题

4.5 证明：光滑结构的定义中，极大性条件确保光滑结构的唯一性。

4.6 设 $M$ 和 $N$ 是微分流形，证明 $M \times N$ 也是微分流形。

4.7 定义李括号 $[X, Y] = XY - YX$，证明它是向量场。

4.8 证明：张量积满足结合律和分配律。

### 编程题

4.9 实现切向量的坐标变换：给定两个坐标卡，计算同一向量的分量变换。

4.10 编写程序计算球面上给定曲线的切向量场。

---

## 参考文献

1. Lee J M. Introduction to Smooth Manifolds[M]. Springer, 2013.
2. Boothby W M. An Introduction to Differentiable Manifolds and Riemannian Geometry[M]. Academic Press, 2002.
3. do Carmo M P. Differential Geometry of Curves and Surfaces[M]. Prentice-Hall, 1976.
4. 陈省身, 陈维桓. 微分几何讲义[M]. 北京大学出版社, 2001.
