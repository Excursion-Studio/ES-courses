# 第2章 随机变量的数字特征

## 本章导读

数字特征是描述随机变量分布特性的重要工具，它们提供了分布的简洁概括。本章系统介绍数学期望、方差、矩和特征函数等核心概念，并深入讨论大数定律和中心极限定理。通过本章学习，你将掌握随机变量数字特征的计算方法，理解极限定理的深刻含义。

---

## 2.1 数学期望和方差

### 2.1.1 数学期望的定义

**直观理解**

数学期望是随机变量取值的"加权平均"，权重是相应的概率。

**离散型随机变量的期望**

定义 2.1（离散型期望） 设 $X$ 是离散型随机变量，概率质量函数为 $p(x_i)$。若 $\sum_i |x_i| p(x_i) < \infty$，则 $X$ 的**数学期望**（expectation）定义为：
$$E[X] = \sum_i x_i p(x_i)$$

**连续型随机变量的期望**

定义 2.2（连续型期望） 设 $X$ 是连续型随机变量，概率密度函数为 $f(x)$。若 $\int_{-\infty}^{+\infty} |x| f(x) dx < \infty$，则 $X$ 的**数学期望**定义为：
$$E[X] = \int_{-\infty}^{+\infty} x f(x) dx$$

**函数的期望**

定理 2.1（期望的函数形式） 设 $Y = g(X)$：
- 离散型：$E[g(X)] = \sum_i g(x_i) p(x_i)$
- 连续型：$E[g(X)] = \int_{-\infty}^{+\infty} g(x) f(x) dx$

### 2.1.2 期望的性质

定理 2.2 数学期望具有以下性质：

1. **线性性**：$E[aX + b] = aE[X] + b$，其中 $a, b$ 是常数

2. **可加性**：$E[X + Y] = E[X] + E[Y]$

3. **单调性**：若 $X \leq Y$（a.s.），则 $E[X] \leq E[Y]$

4. **独立性**：若 $X, Y$ 独立，则 $E[XY] = E[X]E[Y]$

*证明*：

(1) 线性性：
$$E[aX + b] = \int (ax + b) f(x) dx = a\int x f(x) dx + b\int f(x) dx = aE[X] + b$$

(2) 可加性（连续型）：
$$E[X + Y] = \iint (x + y) f(x, y) dx dy = \iint x f(x, y) dx dy + \iint y f(x, y) dx dy = E[X] + E[Y]$$

### 2.1.3 方差和标准差

**方差的定义**

定义 2.3（方差） 设 $X$ 是随机变量，$E[X^2] < \infty$。$X$ 的**方差**（variance）定义为：
$$\text{Var}(X) = E[(X - E[X])^2] = E[X^2] - (E[X])^2$$

**标准差**

定义 2.4（标准差） **标准差**（standard deviation）定义为：
$$\sigma_X = \sqrt{\text{Var}(X)}$$

**方差的性质**

定理 2.3 
1. $\text{Var}(X) \geq 0$，且 $\text{Var}(X) = 0$ 当且仅当 $X = c$（常数，a.s.）

2. $\text{Var}(aX + b) = a^2 \text{Var}(X)$

3. 若 $X, Y$ 独立，则 $\text{Var}(X + Y) = \text{Var}(X) + \text{Var}(Y)$

*证明*：

(2) $\text{Var}(aX + b) = E[(aX + b - aE[X] - b)^2] = E[a^2(X - E[X])^2] = a^2 \text{Var}(X)$

(3) 设 $E[X] = \mu_X, E[Y] = \mu_Y$：
$$\text{Var}(X + Y) = E[(X + Y - \mu_X - \mu_Y)^2]$$
$$= E[(X - \mu_X)^2] + E[(Y - \mu_Y)^2] + 2E[(X - \mu_X)(Y - \mu_Y)]$$
$$= \text{Var}(X) + \text{Var}(Y) + 2\text{Cov}(X, Y)$$

若 $X, Y$ 独立，则 $\text{Cov}(X, Y) = 0$。$\square$

### 2.1.4 常见分布的期望和方差

| 分布 | 期望 | 方差 |
|------|------|------|
| Bernoulli($p$) | $p$ | $p(1-p)$ |
| Binomial($n, p$) | $np$ | $np(1-p)$ |
| Poisson($\lambda$) | $\lambda$ | $\lambda$ |
| Uniform($a, b$) | $\frac{a+b}{2}$ | $\frac{(b-a)^2}{12}$ |
| Normal($\mu, \sigma^2$) | $\mu$ | $\sigma^2$ |
| Exponential($\lambda$) | $\frac{1}{\lambda}$ | $\frac{1}{\lambda^2}$ |

---

## 2.2 协方差和相关系数

### 2.2.1 协方差的定义

定义 2.5（协方差） 设 $X, Y$ 是随机变量，$E[X^2] < \infty, E[Y^2] < \infty$。$X$ 和 $Y$ 的**协方差**（covariance）定义为：
$$\text{Cov}(X, Y) = E[(X - E[X])(Y - E[Y])] = E[XY] - E[X]E[Y]$$

**协方差的性质**

定理 2.4
1. 对称性：$\text{Cov}(X, Y) = \text{Cov}(Y, X)$
2. 双线性：$\text{Cov}(aX + bY, Z) = a\text{Cov}(X, Z) + b\text{Cov}(Y, Z)$
3. $\text{Cov}(X, X) = \text{Var}(X)$
4. 若 $X, Y$ 独立，则 $\text{Cov}(X, Y) = 0$（反之不成立）

### 2.2.2 相关系数

定义 2.6（相关系数） $X$ 和 $Y$ 的**相关系数**（correlation coefficient）定义为：
$$\rho_{X,Y} = \frac{\text{Cov}(X, Y)}{\sigma_X \sigma_Y}$$

**相关系数的性质**

定理 2.5
1. $-1 \leq \rho_{X,Y} \leq 1$
2. $|\rho_{X,Y}| = 1$ 当且仅当 $Y = aX + b$（a.s.），其中 $a \neq 0$
3. $\rho_{X,Y} = 0$ 表示 $X$ 和 $Y$ **不相关**

*证明*：由 Cauchy-Schwarz 不等式：
$$|\text{Cov}(X, Y)|^2 \leq \text{Var}(X) \text{Var}(Y)$$
即 $|\rho_{X,Y}| \leq 1$。$\square$

**独立与不相关**

- 独立 $\Rightarrow$ 不相关
- 不相关 $\nRightarrow$ 独立（除非是联合正态分布）

### 2.2.3 协方差矩阵

定义 2.7（协方差矩阵） 设 $\mathbf{X} = (X_1, \ldots, X_n)^T$ 是随机向量。其**协方差矩阵**定义为：
$$\Sigma = \text{Cov}(\mathbf{X}) = E[(\mathbf{X} - \boldsymbol{\mu})(\mathbf{X} - \boldsymbol{\mu})^T]$$

其中 $\boldsymbol{\mu} = E[\mathbf{X}]$。

**协方差矩阵的性质**

定理 2.6
1. 对称性：$\Sigma^T = \Sigma$
2. 半正定性：对任意向量 $\mathbf{a}$，$\mathbf{a}^T \Sigma \mathbf{a} \geq 0$
3. 对角元：$\Sigma_{ii} = \text{Var}(X_i)$
4. 非对角元：$\Sigma_{ij} = \text{Cov}(X_i, X_j)$

---

## 2.3 矩和特征函数

### 2.3.1 矩的定义

定义 2.8（矩） 设 $X$ 是随机变量，$k$ 是正整数：
- **k阶原点矩**：$E[X^k]$
- **k阶中心矩**：$E[(X - E[X])^k]$

**特殊矩**：
- 一阶原点矩：期望
- 二阶中心矩：方差
- 三阶中心矩：与偏度（skewness）相关
- 四阶中心矩：与峰度（kurtosis）相关

### 2.3.2 特征函数

**定义**

定义 2.9（特征函数） 随机变量 $X$ 的**特征函数**定义为：
$$\varphi_X(t) = E[e^{itX}] = \begin{cases} \sum_j e^{itx_j} p(x_j) & \text{离散型} \\ \int_{-\infty}^{+\infty} e^{itx} f(x) dx & \text{连续型} \end{cases}$$

其中 $i = \sqrt{-1}$。

**特征函数的性质**

定理 2.7
1. $\varphi(0) = 1$，$|\varphi(t)| \leq 1$
2. 共轭对称：$\varphi(-t) = \overline{\varphi(t)}$
3. 一致连续性：$\varphi(t)$ 在 $\mathbb{R}$ 上一致连续
4. 非负定性：对任意 $n$，$t_1, \ldots, t_n \in \mathbb{R}$，$z_1, \ldots, z_n \in \mathbb{C}$：
   $$\sum_{j,k} \varphi(t_j - t_k) z_j \bar{z}_k \geq 0$$

**唯一性定理**

定理 2.8（唯一性） 随机变量的分布函数由其特征函数唯一确定。

**常见分布的特征函数**

| 分布 | 特征函数 |
|------|----------|
| Bernoulli($p$) | $1 - p + pe^{it}$ |
| Poisson($\lambda$) | $\exp(\lambda(e^{it} - 1))$ |
| Normal($\mu, \sigma^2$) | $\exp(i\mu t - \frac{1}{2}\sigma^2 t^2)$ |

### 2.3.3 矩生成函数

定义 2.10（矩生成函数） 随机变量 $X$ 的**矩生成函数**定义为：
$$M_X(t) = E[e^{tX}]$$

**性质**：若 $M_X(t)$ 在 $t=0$ 的某邻域内存在，则：
$$E[X^k] = M_X^{(k)}(0)$$

---

## 2.4 大数定律和中心极限定理

### 2.4.1 大数定律

**直观理解**

大数定律说明：随着试验次数增加，样本均值收敛于理论期望。

**弱大数定律**

定理 2.9（弱大数定律，WLLN） 设 $X_1, X_2, \ldots$ 是独立同分布随机变量，$E[X_i] = \mu$，$\text{Var}(X_i) = \sigma^2 < \infty$。令 $\bar{X}_n = \frac{1}{n}\sum_{i=1}^n X_i$，则对任意 $\epsilon > 0$：
$$\lim_{n \to \infty} P(|\bar{X}_n - \mu| \geq \epsilon) = 0$$

即 $\bar{X}_n \xrightarrow{P} \mu$（依概率收敛）。

*证明*：由 Chebyshev 不等式：
$$P(|\bar{X}_n - \mu| \geq \epsilon) \leq \frac{\text{Var}(\bar{X}_n)}{\epsilon^2} = \frac{\sigma^2}{n\epsilon^2} \to 0$$

**强大数定律**

定理 2.10（强大数定律，SLLN） 在定理 2.9 的条件下：
$$P\left(\lim_{n \to \infty} \bar{X}_n = \mu\right) = 1$$

即 $\bar{X}_n \xrightarrow{a.s.} \mu$（几乎必然收敛）。

### 2.4.2 中心极限定理

**直观理解**

中心极限定理说明：大量独立随机变量之和（适当标准化）近似服从正态分布，无论原始分布是什么。

**林德伯格-列维中心极限定理**

定理 2.11（CLT） 设 $X_1, X_2, \ldots$ 是独立同分布随机变量，$E[X_i] = \mu$，$\text{Var}(X_i) = \sigma^2 < \infty$。则：
$$\frac{\sum_{i=1}^n X_i - n\mu}{\sigma\sqrt{n}} = \frac{\bar{X}_n - \mu}{\sigma/\sqrt{n}} \xrightarrow{d} N(0, 1)$$

即依分布收敛于标准正态分布。

*证明概要*：

设 $Y_i = \frac{X_i - \mu}{\sigma}$，则 $E[Y_i] = 0$，$\text{Var}(Y_i) = 1$。

令 $Z_n = \frac{1}{\sqrt{n}}\sum_{i=1}^n Y_i$，其特征函数为：
$$\varphi_{Z_n}(t) = \left[\varphi_{Y_1}\left(\frac{t}{\sqrt{n}}\right)\right]^n$$

由 Taylor 展开：
$$\varphi_{Y_1}(t) = 1 - \frac{t^2}{2} + o(t^2)$$

因此：
$$\varphi_{Z_n}(t) = \left[1 - \frac{t^2}{2n} + o\left(\frac{1}{n}\right)\right]^n \to e^{-t^2/2}$$

这正是 $N(0, 1)$ 的特征函数。$\square$

**应用意义**

中心极限定理的重要性：
- 解释了为什么正态分布在自然界中如此普遍
- 为统计推断提供了理论基础
- 在机器人中用于误差分析和状态估计

### 2.4.3 大数定律在机器人中的应用

**蒙特卡洛方法**

在机器人状态估计中，常用粒子滤波（Particle Filter）进行近似计算：
- 用大量粒子表示概率分布
- 根据大数定律，粒子均值收敛于真实期望

**传感器融合**

多个传感器测量同一量时：
- 根据大数定律，平均值比单次测量更可靠
- 中心极限定理说明平均值的分布趋于正态

---

## 2.5 本章小结

本章系统介绍了随机变量的数字特征：

1. **数学期望**：
   - 定义：加权平均
   - 性质：线性性、单调性
   - 计算：离散求和、连续积分

2. **方差和标准差**：
   - 定义：偏离期望的程度
   - 性质：$\text{Var}(aX + b) = a^2\text{Var}(X)$
   - 独立随机变量和的方差等于方差之和

3. **协方差和相关系数**：
   - 协方差：$\text{Cov}(X, Y) = E[XY] - E[X]E[Y]$
   - 相关系数：$\rho = \frac{\text{Cov}(X, Y)}{\sigma_X \sigma_Y}$
   - 协方差矩阵：描述随机向量的相关性

4. **矩和特征函数**：
   - 矩：描述分布的形状
   - 特征函数：唯一确定分布
   - 矩生成函数：生成各阶矩

5. **极限定理**：
   - 大数定律：样本均值收敛于期望
   - 中心极限定理：标准化和趋于正态分布

这些数字特征和极限定理为随机过程分析和机器人状态估计提供了理论基础。

---

## 习题

### 基础题

2.1 证明方差的计算公式：$\text{Var}(X) = E[X^2] - (E[X])^2$。

2.2 设 $X \sim \text{Uniform}(0, 1)$，计算 $E[X^n]$ 和 $\text{Var}(X^n)$。

2.3 证明 Cauchy-Schwarz 不等式：$|E[XY]|^2 \leq E[X^2]E[Y^2]$。

2.4 设 $X_1, \ldots, X_n$ 独立同分布，$E[X_i] = \mu$，$\text{Var}(X_i) = \sigma^2$。求 $\bar{X} = \frac{1}{n}\sum_{i=1}^n X_i$ 的期望和方差。

### 提高题

2.5 证明特征函数的一致连续性。

2.6 设 $X$ 和 $Y$ 是联合正态分布，证明不相关等价于独立。

2.7 用特征函数证明中心极限定理。

2.8 证明强大数定律（假设四阶矩存在）。

### 编程题

2.9 编写程序模拟大数定律：生成不同样本量的随机数，观察样本均值收敛。

2.10 实现蒙特卡洛积分，并与数值积分方法比较。

---

## 参考文献

1. 盛骤, 谢式千, 潘承毅. 概率论与数理统计[M]. 高等教育出版社, 2008.
2. Durrett R. Probability: Theory and Examples[M]. Cambridge University Press, 2019.
3. Grimmett G, Stirzaker D. Probability and Random Processes[M]. Oxford University Press, 2001.
4. Billingsley P. Probability and Measure[M]. Wiley, 2012.
