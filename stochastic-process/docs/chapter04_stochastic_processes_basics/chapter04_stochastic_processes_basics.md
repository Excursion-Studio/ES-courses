# 第4章 随机向量与随机过程基础

## 本章导读

随机过程是概率论的动态扩展，研究随时间演化的随机现象。本章介绍随机向量的联合分布、条件分布和高斯分布，以及随机过程的基本概念和分类。通过本章学习，你将掌握随机过程的数学描述方法，为后续学习马尔可夫过程、平稳过程等奠定基础。

---

## 4.1 随机向量

### 4.1.1 联合分布函数

定义 4.1（联合分布函数） 设 $\mathbf{X} = (X_1, X_2, \ldots, X_n)^T$ 是 $n$ 维随机向量。其**联合分布函数**定义为：
$$F_{\mathbf{X}}(\mathbf{x}) = P(X_1 \leq x_1, X_2 \leq x_2, \ldots, X_n \leq x_n)$$

**联合概率密度**

若存在 $f_{\mathbf{X}}(\mathbf{x}) \geq 0$ 使得：
$$F_{\mathbf{X}}(\mathbf{x}) = \int_{-\infty}^{x_1} \cdots \int_{-\infty}^{x_n} f_{\mathbf{X}}(\mathbf{u}) d\mathbf{u}$$

则称 $f_{\mathbf{X}}(\mathbf{x})$ 为**联合概率密度函数**。

### 4.1.2 边缘分布与条件分布

**边缘分布**

随机向量中部分分量的分布称为**边缘分布**：
$$F_{X_1}(x_1) = F_{\mathbf{X}}(x_1, +\infty, \ldots, +\infty)$$

$$f_{X_1}(x_1) = \int_{-\infty}^{+\infty} \cdots \int_{-\infty}^{+\infty} f_{\mathbf{X}}(\mathbf{x}) dx_2 \cdots dx_n$$

**条件分布**

给定 $X_2 = x_2, \ldots, X_n = x_n$ 下 $X_1$ 的条件分布：
$$f_{X_1|X_2,\ldots,X_n}(x_1|x_2,\ldots,x_n) = \frac{f_{\mathbf{X}}(\mathbf{x})}{f_{X_2,\ldots,X_n}(x_2,\ldots,x_n)}$$

### 4.1.3 多元正态分布

定义 4.2（多元正态分布） 随机向量 $\mathbf{X}$ 服从**多元正态分布** $N(\boldsymbol{\mu}, \Sigma)$，如果其概率密度函数为：
$$f_{\mathbf{X}}(\mathbf{x}) = \frac{1}{(2\pi)^{n/2} |\Sigma|^{1/2}} \exp\left(-\frac{1}{2}(\mathbf{x} - \boldsymbol{\mu})^T \Sigma^{-1} (\mathbf{x} - \boldsymbol{\mu})\right)$$

其中：
- $\boldsymbol{\mu} = E[\mathbf{X}]$ 是均值向量
- $\Sigma = \text{Cov}(\mathbf{X})$ 是协方差矩阵

**性质**

定理 4.1
1. 线性变换：若 $\mathbf{X} \sim N(\boldsymbol{\mu}, \Sigma)$，则 $\mathbf{Y} = A\mathbf{X} + \mathbf{b} \sim N(A\boldsymbol{\mu} + \mathbf{b}, A\Sigma A^T)$
2. 边缘分布：多元正态的边缘分布仍是正态分布
3. 条件分布：多元正态的条件分布仍是正态分布
4. 独立性：对于联合正态分布，不相关等价于独立

**条件分布公式**

将 $\mathbf{X}$ 分为两部分 $\mathbf{X} = (\mathbf{X}_1^T, \mathbf{X}_2^T)^T$，相应划分：
$$\boldsymbol{\mu} = \begin{pmatrix} \boldsymbol{\mu}_1 \\ \boldsymbol{\mu}_2 \end{pmatrix}, \quad \Sigma = \begin{pmatrix} \Sigma_{11} & \Sigma_{12} \\ \Sigma_{21} & \Sigma_{22} \end{pmatrix}$$

则：
$$\mathbf{X}_1 | \mathbf{X}_2 = \mathbf{x}_2 \sim N(\boldsymbol{\mu}_{1|2}, \Sigma_{1|2})$$

其中：
$$\boldsymbol{\mu}_{1|2} = \boldsymbol{\mu}_1 + \Sigma_{12}\Sigma_{22}^{-1}(\mathbf{x}_2 - \boldsymbol{\mu}_2)$$
$$\Sigma_{1|2} = \Sigma_{11} - \Sigma_{12}\Sigma_{22}^{-1}\Sigma_{21}$$

---

## 4.2 随机过程的基本概念

### 4.2.1 随机过程的定义

定义 4.3（随机过程） 设 $(\Omega, \mathcal{F}, P)$ 是概率空间，$T$ 是指标集（通常是时间）。**随机过程**是映射：
$$X: T \times \Omega \to \mathbb{R}$$
$$(t, \omega) \mapsto X(t, \omega)$$

对于固定的 $t$，$X(t, \cdot)$ 是随机变量；对于固定的 $\omega$，$X(\cdot, \omega)$ 是样本函数（实现）。

**等价视角**

随机过程可以看作：
1. 一族随机变量 $\{X(t)\}_{t \in T}$
2. 定义在路径空间上的概率测度

### 4.2.2 有限维分布

定义 4.4（有限维分布） 随机过程的**有限维分布**定义为：
$$F_{t_1, \ldots, t_n}(x_1, \ldots, x_n) = P(X(t_1) \leq x_1, \ldots, X(t_n) \leq x_n)$$

**Kolmogorov 存在定理**

定理 4.2（Kolmogorov） 给定满足相容性条件的有限维分布族，存在随机过程以这些为有限维分布。

### 4.2.3 随机过程的数字特征

**均值函数**

$$\mu_X(t) = E[X(t)]$$

**自相关函数**

$$R_X(t_1, t_2) = E[X(t_1)X(t_2)]$$

**自协方差函数**

$$C_X(t_1, t_2) = \text{Cov}(X(t_1), X(t_2)) = R_X(t_1, t_2) - \mu_X(t_1)\mu_X(t_2)$$

**方差函数**

$$\sigma_X^2(t) = C_X(t, t) = \text{Var}(X(t))$$

---

## 4.3 随机过程的分类

### 4.3.1 按状态空间分类

- **离散状态过程**：$X(t)$ 取离散值
- **连续状态过程**：$X(t)$ 取连续值

### 4.3.2 按时间参数分类

- **离散时间过程**：$T = \{0, 1, 2, \ldots\}$（随机序列）
- **连续时间过程**：$T = [0, +\infty)$ 或 $T = \mathbb{R}$

### 4.3.3 按统计特性分类

**独立过程**

定义 4.5（独立过程） 若对任意 $t_1, \ldots, t_n$，$X(t_1), \ldots, X(t_n)$ 相互独立，则称 $\{X(t)\}$ 为**独立过程**。

**独立增量过程**

定义 4.6（独立增量过程） 若对任意 $t_1 < t_2 < \cdots < t_n$，增量 $X(t_2) - X(t_1), \ldots, X(t_n) - X(t_{n-1})$ 相互独立，则称 $\{X(t)\}$ 为**独立增量过程**。

**平稳过程**

定义 4.7（严平稳） 若对任意 $t_1, \ldots, t_n$ 和 $\tau$，$(X(t_1), \ldots, X(t_n))$ 与 $(X(t_1+\tau), \ldots, X(t_n+\tau))$ 同分布，则称 $\{X(t)\}$ 为**严平稳过程**（strictly stationary）。

定义 4.8（宽平稳） 若：
- $\mu_X(t) = \mu$（常数）
- $R_X(t_1, t_2) = R_X(t_1 - t_2)$（仅依赖于时间差）

则称 $\{X(t)\}$ 为**宽平稳过程**（wide-sense stationary）。

**关系**：严平稳 $\Rightarrow$ 宽平稳（在二阶矩存在时）。

### 4.3.4 马尔可夫过程

定义 4.9（马尔可夫性） 随机过程 $\{X(t)\}$ 具有**马尔可夫性**，如果：
$$P(X(t_{n+1}) \leq x | X(t_n), X(t_{n-1}), \ldots, X(t_1)) = P(X(t_{n+1}) \leq x | X(t_n))$$

即：给定现在，未来与过去独立。

具有马尔可夫性的过程称为**马尔可夫过程**。

---

## 4.4 高斯过程

### 4.4.1 高斯过程的定义

定义 4.10（高斯过程） 随机过程 $\{X(t)\}_{t \in T}$ 称为**高斯过程**，如果对任意有限个时刻 $t_1, \ldots, t_n$，$(X(t_1), \ldots, X(t_n))$ 服从多元正态分布。

**性质**

高斯过程完全由其均值函数 $\mu(t)$ 和协方差函数 $C(s, t)$ 确定。

### 4.4.2 布朗运动

定义 4.11（布朗运动） 随机过程 $\{B(t)\}_{t \geq 0}$ 称为**标准布朗运动**（Brownian motion），如果：
1. $B(0) = 0$（a.s.）
2. 具有独立增量
3. $B(t) - B(s) \sim N(0, t-s)$，$t > s$
4. 样本路径连续

**性质**

- $B(t) \sim N(0, t)$
- $E[B(t)] = 0$
- $\text{Var}(B(t)) = t$
- $E[B(s)B(t)] = \min(s, t)$

**几何布朗运动**

$$S(t) = S(0) \exp\left(\left(\mu - \frac{\sigma^2}{2}\right)t + \sigma B(t)\right)$$

在金融建模中有重要应用。

---

## 4.5 本章小结

本章介绍了随机向量和随机过程的基础理论：

1. **随机向量**：
   - 联合分布、边缘分布、条件分布
   - 多元正态分布及其性质
   - 条件分布公式

2. **随机过程**：
   - 定义：一族随机变量
   - 有限维分布
   - 数字特征：均值、自相关、自协方差

3. **随机过程的分类**：
   - 按状态空间：离散/连续
   - 按时间：离散/连续
   - 按统计特性：独立、平稳、马尔可夫

4. **高斯过程**：
   - 由均值和协方差完全确定
   - 布朗运动：最重要的随机过程之一

这些概念为后续学习马尔可夫链、平稳过程、随机微分方程等奠定了基础。

---

## 习题

### 基础题

4.1 设 $\mathbf{X} \sim N(\mathbf{0}, I_n)$，$A$ 是正交矩阵。证明 $A\mathbf{X} \sim N(\mathbf{0}, I_n)$。

4.2 计算布朗运动的自相关函数 $R_B(s, t)$。

4.3 证明：严平稳过程在宽平稳的定义下是宽平稳的（假设二阶矩存在）。

4.4 设 $\{X(t)\}$ 是独立增量过程，$X(0) = 0$。证明：$\text{Var}(X(t)) = \text{Var}(X(t) - X(0))$ 是 $t$ 的非减函数。

### 提高题

4.5 推导多元正态分布的条件分布公式。

4.6 证明布朗运动的样本路径几乎处处不可微。

4.7 设 $\{X(t)\}$ 是高斯过程，证明宽平稳等价于严平稳。

4.8 设计一个算法生成高斯过程的样本路径。

### 编程题

4.9 实现多元正态分布的随机数生成器。

4.10 模拟布朗运动并可视化样本路径。

---

## 参考文献

1. Ross S M. Stochastic Processes[M]. Wiley, 1996.
2. Karlin S, Taylor H M. A First Course in Stochastic Processes[M]. Academic Press, 1975.
3. Durrett R. Essentials of Stochastic Processes[M]. Springer, 2012.
4. Øksendal B. Stochastic Differential Equations[M]. Springer, 2003.
