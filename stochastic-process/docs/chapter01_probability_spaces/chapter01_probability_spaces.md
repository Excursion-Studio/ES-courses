# 第1章 概率空间与随机变量

## 本章导读

概率论是研究随机现象数量规律的数学分支，是随机过程理论的基础。本章从概率空间的公理化定义出发，系统介绍随机变量的概念、常见概率分布及其性质。通过本章学习，你将掌握概率论的严格数学框架，理解随机变量的本质，为后续学习随机过程奠定坚实基础。

---

## 1.1 概率空间的定义

### 1.1.1 样本空间与事件

**随机试验与样本空间**

定义 1.1（随机试验） **随机试验**是指在相同条件下可以重复进行，且每次试验结果具有不确定性的试验。随机试验具有以下特点：
- 可以在相同条件下重复进行
- 每次试验的可能结果不止一个
- 试验前不能确定会出现哪个结果

定义 1.2（样本空间） 随机试验所有可能结果构成的集合称为**样本空间**（sample space），记为 $\Omega$。样本空间中的元素称为**样本点**或**基本事件**。

**例 1.1**
- 抛一枚硬币：$\Omega = \{H, T\}$（H表示正面，T表示反面）
- 掷一颗骰子：$\Omega = \{1, 2, 3, 4, 5, 6\}$
- 测量某零件的长度：$\Omega = [0, +\infty)$

**事件**

定义 1.3（事件） 样本空间 $\Omega$ 的子集称为**事件**（event）。事件 $A$ 发生当且仅当试验结果 $\omega \in A$。

**特殊事件**：
- 必然事件：$\Omega$（每次试验都发生）
- 不可能事件：$\varnothing$（每次试验都不发生）
- 基本事件：单点集 $\{\omega\}$

**事件的运算**

设 $A, B$ 是事件：
- **并事件** $A \cup B$：$A$ 或 $B$ 发生
- **交事件** $A \cap B$（或 $AB$）：$A$ 和 $B$ 都发生
- **补事件** $\bar{A}$ 或 $A^c$：$A$ 不发生
- **差事件** $A \setminus B = A \cap B^c$：$A$ 发生但 $B$ 不发生

### 1.1.2 事件域（σ-代数）

为了对事件赋予概率，我们需要事件族具有一定的结构。

定义 1.4（σ-代数/事件域） 设 $\mathcal{F}$ 是 $\Omega$ 的子集族。如果 $\mathcal{F}$ 满足：

(σ1) $\Omega \in \mathcal{F}$

(σ2) 若 $A \in \mathcal{F}$，则 $A^c \in \mathcal{F}$（对补运算封闭）

(σ3) 若 $A_1, A_2, \ldots \in \mathcal{F}$，则 $\bigcup_{i=1}^\infty A_i \in \mathcal{F}$（对可数并封闭）

则称 $\mathcal{F}$ 为 **σ-代数**（sigma-algebra）或**事件域**，$(\Omega, \mathcal{F})$ 称为**可测空间**。

**σ-代数的性质**

定理 1.1 若 $\mathcal{F}$ 是 σ-代数，则：
- $\varnothing \in \mathcal{F}$
- 对有限并封闭：若 $A_1, \ldots, A_n \in \mathcal{F}$，则 $\bigcup_{i=1}^n A_i \in \mathcal{F}$
- 对可数交封闭：若 $A_1, A_2, \ldots \in \mathcal{F}$，则 $\bigcap_{i=1}^\infty A_i \in \mathcal{F}$
- 对差运算封闭：若 $A, B \in \mathcal{F}$，则 $A \setminus B \in \mathcal{F}$

*证明*：由 De Morgan 定律和 σ-代数的定义直接得到。$\square$

**生成的 σ-代数**

定义 1.5（生成的 σ-代数） 设 $\mathcal{C}$ 是 $\Omega$ 的子集族，包含 $\mathcal{C}$ 的最小 σ-代数称为由 $\mathcal{C}$ **生成的 σ-代数**，记为 $\sigma(\mathcal{C})$。

**Borel σ-代数**

定义 1.6（Borel σ-代数） 在 $\mathbb{R}$ 上，由所有开区间生成的 σ-代数称为**Borel σ-代数**，记为 $\mathcal{B}(\mathbb{R})$。

$\mathcal{B}(\mathbb{R})$ 包含所有开集、闭集、$G_\delta$ 集（可数个开集的交）、$F_\sigma$ 集（可数个闭集的并）等。

### 1.1.3 概率测度

**概率的公理化定义**

定义 1.7（概率测度） 设 $(\Omega, \mathcal{F})$ 是可测空间。映射 $P: \mathcal{F} \to [0, 1]$ 称为**概率测度**（probability measure），如果满足：

(P1) **非负性**：$P(A) \geq 0$ 对所有 $A \in \mathcal{F}$

(P2) **规范性**：$P(\Omega) = 1$

(P3) **可列可加性**：若 $A_1, A_2, \ldots \in \mathcal{F}$ 两两不交，则：
$$P\left(\bigcup_{i=1}^\infty A_i\right) = \sum_{i=1}^\infty P(A_i)$$

**概率空间**

定义 1.8（概率空间） 三元组 $(\Omega, \mathcal{F}, P)$ 称为**概率空间**（probability space），其中：
- $\Omega$ 是样本空间
- $\mathcal{F}$ 是事件域
- $P$ 是概率测度

**概率的基本性质**

定理 1.2 设 $(\Omega, \mathcal{F}, P)$ 是概率空间：

1. $P(\varnothing) = 0$

2. **有限可加性**：若 $A_1, \ldots, A_n$ 两两不交，则 $P(\bigcup_{i=1}^n A_i) = \sum_{i=1}^n P(A_i)$

3. **补事件概率**：$P(A^c) = 1 - P(A)$

4. **单调性**：若 $A \subseteq B$，则 $P(A) \leq P(B)$

5. **容斥原理**：
   $$P(A \cup B) = P(A) + P(B) - P(A \cap B)$$

6. **连续性**：
   - 若 $A_1 \subseteq A_2 \subseteq \cdots$，则 $P(\bigcup_{i=1}^\infty A_i) = \lim_{n \to \infty} P(A_n)$
   - 若 $A_1 \supseteq A_2 \supseteq \cdots$，则 $P(\bigcap_{i=1}^\infty A_i) = \lim_{n \to \infty} P(A_n)$

*证明*：

(1) 令 $A_1 = \Omega, A_2 = A_3 = \cdots = \varnothing$，由可列可加性：
$$P(\Omega) = P(\Omega) + \sum_{i=2}^\infty P(\varnothing)$$
故 $P(\varnothing) = 0$。

(3) $A \cup A^c = \Omega$ 且 $A \cap A^c = \varnothing$，由有限可加性：
$$P(A) + P(A^c) = P(\Omega) = 1$$

其余类似可证。$\square$

### 1.1.4 条件概率

**条件概率的定义**

定义 1.9（条件概率） 设 $A, B \in \mathcal{F}$ 且 $P(B) > 0$。在事件 $B$ 发生的条件下，事件 $A$ 发生的**条件概率**定义为：
$$P(A|B) = \frac{P(A \cap B)}{P(B)}$$

**条件概率的性质**

定理 1.3 固定 $B$ 且 $P(B) > 0$，则 $P(\cdot|B)$ 是 $\mathcal{F}$ 上的概率测度：
- $P(A|B) \geq 0$
- $P(\Omega|B) = 1$
- 可列可加性成立

**乘法公式**

定理 1.4 若 $P(A_1 A_2 \cdots A_{n-1}) > 0$，则：
$$P(A_1 A_2 \cdots A_n) = P(A_1) P(A_2|A_1) P(A_3|A_1 A_2) \cdots P(A_n|A_1 \cdots A_{n-1})$$

**全概率公式**

定理 1.5（全概率公式） 设 $\{B_i\}_{i=1}^\infty$ 是 $\Omega$ 的一个划分（即 $B_i$ 两两不交且 $\bigcup_i B_i = \Omega$），且 $P(B_i) > 0$，则对任意 $A \in \mathcal{F}$：
$$P(A) = \sum_{i=1}^\infty P(B_i) P(A|B_i)$$

**贝叶斯公式**

定理 1.6（贝叶斯公式） 在定理 1.5 的条件下，若 $P(A) > 0$，则：
$$P(B_i|A) = \frac{P(B_i) P(A|B_i)}{\sum_{j=1}^\infty P(B_j) P(A|B_j)}$$

**例 1.2 贝叶斯推断**

假设某种疾病的发病率为 0.1%（先验概率）。检测的准确率为：患者检测阳性概率 99%，健康者检测阴性概率 98%。若某人检测阳性，求其患病的概率。

解：设 $D$ 为患病事件，$T$ 为检测阳性事件。
- $P(D) = 0.001$
- $P(T|D) = 0.99$
- $P(T|D^c) = 0.02$

由贝叶斯公式：
$$P(D|T) = \frac{P(D)P(T|D)}{P(D)P(T|D) + P(D^c)P(T|D^c)} = \frac{0.001 \times 0.99}{0.001 \times 0.99 + 0.999 \times 0.02} \approx 0.047$$

即使检测阳性，患病概率也只有约 4.7%，这是因为疾病本身很罕见。

---

## 1.2 随机变量的概念

### 1.2.1 随机变量的定义

**直观理解**

随机变量是将随机试验的结果数量化的工具。形式上，它是样本空间上的可测函数。

**严格定义**

定义 1.10（随机变量） 设 $(\Omega, \mathcal{F}, P)$ 是概率空间，$(E, \mathcal{E})$ 是可测空间。映射 $X: \Omega \to E$ 称为**随机变量**（random variable），如果对任意 $B \in \mathcal{E}$，有：
$$X^{-1}(B) = \{\omega \in \Omega : X(\omega) \in B\} \in \mathcal{F}$$

即 $X$ 是 $(\Omega, \mathcal{F})$ 到 $(E, \mathcal{E})$ 的**可测映射**。

当 $E = \mathbb{R}$ 且 $\mathcal{E} = \mathcal{B}(\mathbb{R})$ 时，称 $X$ 为**实值随机变量**。

**分布**

定义 1.11（分布） 随机变量 $X$ 诱导的概率测度 $P_X$ 在 $(E, \mathcal{E})$ 上定义为：
$$P_X(B) = P(X^{-1}(B)) = P(X \in B), \quad B \in \mathcal{E}$$

$P_X$ 称为 $X$ 的**分布**（distribution）或**概率分布**。

### 1.2.2 离散型随机变量

定义 1.12（离散型随机变量） 若随机变量 $X$ 只取有限个或可数个值，则称 $X$ 为**离散型随机变量**。

设 $X$ 的可能取值为 $\{x_1, x_2, \ldots\}$，则 $X$ 的**概率质量函数**（PMF）定义为：
$$p(x_i) = P(X = x_i), \quad i = 1, 2, \ldots$$

满足：
- $p(x_i) \geq 0$
- $\sum_i p(x_i) = 1$

**分布函数**

定义 1.13（分布函数） 随机变量 $X$ 的**分布函数**（CDF）定义为：
$$F(x) = P(X \leq x), \quad x \in \mathbb{R}$$

对于离散型随机变量：
$$F(x) = \sum_{x_i \leq x} p(x_i)$$

### 1.2.3 连续型随机变量

定义 1.14（连续型随机变量） 若存在非负可积函数 $f(x)$ 使得随机变量 $X$ 的分布函数可表示为：
$$F(x) = \int_{-\infty}^x f(t) dt$$

则称 $X$ 为**连续型随机变量**，$f(x)$ 称为**概率密度函数**（PDF）。

**概率密度函数的性质**：
- $f(x) \geq 0$
- $\int_{-\infty}^{+\infty} f(x) dx = 1$
- $P(a < X \leq b) = F(b) - F(a) = \int_a^b f(x) dx$

**注意**：对于连续型随机变量，$P(X = x) = 0$ 对所有 $x$ 成立。

### 1.2.4 分布函数的性质

定理 1.7 分布函数 $F(x)$ 具有以下性质：

1. **单调不减**：若 $x_1 < x_2$，则 $F(x_1) \leq F(x_2)$

2. **右连续**：$\lim_{x \to x_0^+} F(x) = F(x_0)$

3. **极限性质**：
   $$\lim_{x \to -\infty} F(x) = 0, \quad \lim_{x \to +\infty} F(x) = 1$$

反之，满足上述三条的函数必是某随机变量的分布函数。

---

## 1.3 常见概率分布

### 1.3.1 离散型分布

**1. 伯努利分布 Bernoulli($p$)**

随机变量 $X$ 只取 0 和 1：
$$P(X = 1) = p, \quad P(X = 0) = 1 - p$$

其中 $0 \leq p \leq 1$。

期望：$E[X] = p$

方差：$\text{Var}(X) = p(1-p)$

**2. 二项分布 Binomial($n, p$)**

$n$ 次独立伯努利试验中成功的次数：
$$P(X = k) = \binom{n}{k} p^k (1-p)^{n-k}, \quad k = 0, 1, \ldots, n$$

期望：$E[X] = np$

方差：$\text{Var}(X) = np(1-p)$

**3. 泊松分布 Poisson($\lambda$)**

$$P(X = k) = \frac{\lambda^k e^{-\lambda}}{k!}, \quad k = 0, 1, 2, \ldots$$

其中 $\lambda > 0$。

期望和方差：$E[X] = \text{Var}(X) = \lambda$

**泊松分布的性质**：
- 可加性：若 $X_1 \sim \text{Poisson}(\lambda_1)$，$X_2 \sim \text{Poisson}(\lambda_2)$ 独立，则 $X_1 + X_2 \sim \text{Poisson}(\lambda_1 + \lambda_2)$
- 稀有事件极限：当 $n \to \infty$，$p \to 0$ 且 $np \to \lambda$ 时，二项分布收敛到泊松分布

**4. 几何分布 Geometric($p$)**

首次成功所需的试验次数：
$$P(X = k) = (1-p)^{k-1} p, \quad k = 1, 2, \ldots$$

期望：$E[X] = \frac{1}{p}$

方差：$\text{Var}(X) = \frac{1-p}{p^2}$

**无记忆性**：$P(X > m + n | X > m) = P(X > n)$

### 1.3.2 连续型分布

**1. 均匀分布 Uniform($a, b$)**

$$f(x) = \begin{cases} \frac{1}{b-a} & a \leq x \leq b \\ 0 & \text{其他} \end{cases}$$

期望：$E[X] = \frac{a+b}{2}$

方差：$\text{Var}(X) = \frac{(b-a)^2}{12}$

**2. 正态分布 Normal($\mu, \sigma^2$)**

$$f(x) = \frac{1}{\sqrt{2\pi}\sigma} \exp\left(-\frac{(x-\mu)^2}{2\sigma^2}\right), \quad x \in \mathbb{R}$$

记为 $X \sim N(\mu, \sigma^2)$。

期望：$E[X] = \mu$

方差：$\text{Var}(X) = \sigma^2$

**正态分布的性质**：
- 标准化：若 $X \sim N(\mu, \sigma^2)$，则 $Z = \frac{X-\mu}{\sigma} \sim N(0, 1)$（标准正态分布）
- 可加性：独立正态随机变量的和仍服从正态分布
- 中心极限定理的基础

**3. 指数分布 Exponential($\lambda$)**

$$f(x) = \begin{cases} \lambda e^{-\lambda x} & x \geq 0 \\ 0 & x < 0 \end{cases}$$

其中 $\lambda > 0$。

期望：$E[X] = \frac{1}{\lambda}$

方差：$\text{Var}(X) = \frac{1}{\lambda^2}$

**指数分布的性质**：
- 无记忆性：$P(X > s + t | X > s) = P(X > t)$
- 与泊松过程的关系：泊松过程中事件间隔时间服从指数分布

**4. 伽马分布 Gamma($\alpha, \beta$)**

$$f(x) = \begin{cases} \frac{\beta^\alpha}{\Gamma(\alpha)} x^{\alpha-1} e^{-\beta x} & x > 0 \\ 0 & x \leq 0 \end{cases}$$

其中 $\alpha > 0$（形状参数），$\beta > 0$（速率参数），$\Gamma(\alpha)$ 是伽马函数。

期望：$E[X] = \frac{\alpha}{\beta}$

方差：$\text{Var}(X) = \frac{\alpha}{\beta^2}$

**特殊情形**：
- $\alpha = 1$：指数分布
- $\alpha = n/2$，$\beta = 1/2$：卡方分布 $\chi^2(n)$

### 1.3.3 多维随机变量

**联合分布**

定义 1.15（联合分布函数） 随机向量 $(X, Y)$ 的**联合分布函数**定义为：
$$F(x, y) = P(X \leq x, Y \leq y)$$

**联合概率密度**

若存在 $f(x, y) \geq 0$ 使得：
$$F(x, y) = \int_{-\infty}^x \int_{-\infty}^y f(u, v) du dv$$

则称 $f(x, y)$ 为**联合概率密度函数**。

**边缘分布**

$$f_X(x) = \int_{-\infty}^{+\infty} f(x, y) dy$$

$$f_Y(y) = \int_{-\infty}^{+\infty} f(x, y) dx$$

**独立性**

定义 1.16（独立性） 随机变量 $X$ 和 $Y$ **独立**，如果：
$$F(x, y) = F_X(x) F_Y(y)$$

或等价地：
$$f(x, y) = f_X(x) f_Y(y)$$

**条件分布**

$$f_{X|Y}(x|y) = \frac{f(x, y)}{f_Y(y)}, \quad f_Y(y) > 0$$

---

## 1.4 本章小结

本章系统介绍了概率空间的基础理论：

1. **概率空间**：由样本空间 $\Omega$、事件域 $\mathcal{F}$ 和概率测度 $P$ 组成的三元组 $(\Omega, \mathcal{F}, P)$。

2. **条件概率与贝叶斯公式**：
   - 条件概率：$P(A|B) = P(AB)/P(B)$
   - 全概率公式：$P(A) = \sum_i P(B_i)P(A|B_i)$
   - 贝叶斯公式：$P(B_i|A) = P(B_i)P(A|B_i)/P(A)$

3. **随机变量**：样本空间上的可测函数，将随机结果数量化。
   - 离散型：概率质量函数（PMF）
   - 连续型：概率密度函数（PDF）
   - 分布函数（CDF）：统一描述两种类型

4. **常见分布**：
   - 离散型：伯努利、二项、泊松、几何
   - 连续型：均匀、正态、指数、伽马

5. **多维随机变量**：联合分布、边缘分布、条件分布、独立性

这些概念为后续学习随机过程、状态估计和机器人应用奠定了概率论基础。

---

## 习题

### 基础题

1.1 证明概率测度的连续性：若 $A_1 \subseteq A_2 \subseteq \cdots$，则 $P(\bigcup_{i=1}^\infty A_i) = \lim_{n \to \infty} P(A_n)$。

1.2 设 $X \sim \text{Binomial}(n, p)$，求 $P(X \geq 1)$ 和 $P(X = k | X \geq 1)$。

1.3 证明泊松分布的概率质量函数满足归一化条件：$\sum_{k=0}^\infty \frac{\lambda^k e^{-\lambda}}{k!} = 1$。

1.4 设 $X \sim N(0, 1)$，求 $P(X > 1.96)$ 和 $P(|X| < 1.96)$。

### 提高题

1.5 证明：若 $X$ 和 $Y$ 独立，则 $E[XY] = E[X]E[Y]$（假设期望存在）。

1.6 设 $X_1, X_2, \ldots, X_n$ 独立同分布于 $N(\mu, \sigma^2)$，求 $\bar{X} = \frac{1}{n}\sum_{i=1}^n X_i$ 的分布。

1.7 证明指数分布的无记忆性：$P(X > s + t | X > s) = P(X > t)$。

1.8 设 $(X, Y)$ 服从二元正态分布，求条件分布 $X|Y=y$。

### 编程题

1.9 实现常见分布的随机数生成器：
   - 用逆变换法生成指数分布随机数
   - 用 Box-Muller 变换生成正态分布随机数

1.10 编写程序验证大数定律：生成大量独立同分布随机变量，观察样本均值收敛到期望值。

---

## 参考文献

1. 盛骤, 谢式千, 潘承毅. 概率论与数理统计[M]. 高等教育出版社, 2008.
2. Blitzstein J K, Hwang J. Introduction to Probability[M]. CRC Press, 2019.
3. Ross S M. Introduction to Probability Models[M]. Academic Press, 2014.
4. Durrett R. Probability: Theory and Examples[M]. Cambridge University Press, 2019.
