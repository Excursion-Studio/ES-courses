# 第1章 拓扑空间的基本概念

## 本章导读

拓扑学是研究空间在连续变形下保持不变的性质的数学分支，是现代数学的基础语言之一。本章从拓扑空间的基本定义出发，介绍连续性、连通性、紧致性等核心概念，为后续学习微分流形奠定严格的数学基础。通过本章学习，你将理解"邻近"和"连续"的数学本质，并学会用拓扑语言描述空间结构。

---

## 1.1 拓扑空间的定义和性质

### 1.1.1 拓扑的公理化定义

**直观理解**

拓扑是描述空间中"邻近关系"的数学结构。在欧几里得空间中，我们知道什么是开区间、开球，以及点列的收敛。拓扑学将这些概念抽象化，使其适用于更广泛的空间。

**严格定义**

定义 1.1（拓扑空间） 设 $X$ 是一个集合，$\mathcal{T}$ 是 $X$ 的子集族。如果 $\mathcal{T}$ 满足以下公理：

(T1) $\varnothing \in \mathcal{T}$ 且 $X \in \mathcal{T}$

(T2) 任意并封闭：若 $\{U_\alpha\}\_{\alpha \in I} \subseteq \mathcal{T}$，则 $\bigcup_{\alpha \in I} U_\alpha \in \mathcal{T}$

(T3) 有限交封闭：若 $U_1, U_2, \ldots, U_n \in \mathcal{T}$，则 $\bigcap_{i=1}^n U_i \in \mathcal{T}$

则称 $\mathcal{T}$ 为 $X$ 上的一个**拓扑**（topology），$(X, \mathcal{T})$ 称为**拓扑空间**。$\mathcal{T}$ 中的元素称为**开集**（open sets）。

**例 1.1 离散拓扑**

对于任意集合 $X$，$\mathcal{T} = 2^X$（$X$ 的幂集，即所有子集的集合）是一个拓扑，称为**离散拓扑**（discrete topology）。

在离散拓扑中，每个单点集 $\{x\}$ 都是开集，因此所有子集都是开集。

**例 1.2 平凡拓扑**

对于任意集合 $X$，$\mathcal{T} = \{\varnothing, X\}$ 是一个拓扑，称为**平凡拓扑**（trivial topology）或**密着拓扑**（indiscrete topology）。

平凡拓扑是包含最少开集的拓扑。

**例 1.3 欧几里得拓扑**

在 $\mathbb{R}^n$ 上，子集 $U \subseteq \mathbb{R}^n$ 称为开的，如果对于每个 $x \in U$，存在 $\epsilon > 0$ 使得开球 $B(x, \epsilon) = \{y \in \mathbb{R}^n : \|y - x\| < \epsilon\} \subseteq U$。

所有这样的开集构成 $\mathbb{R}^n$ 上的**欧几里得拓扑**（Euclidean topology）或**标准拓扑**（standard topology）。

**例 1.4 余有限拓扑**

设 $X$ 是无限集，定义：
$$\mathcal{T} = \{U \subseteq X : X \setminus U \text{ 是有限集}\} \cup \{\varnothing\}$$

这是 $X$ 上的一个拓扑，称为**余有限拓扑**（cofinite topology）。

验证：
- (T1) 显然成立
- (T2) 设 $\{U_\alpha\}$ 是一族开集，则 $X \setminus \bigcup_\alpha U_\alpha = \bigcap_\alpha (X \setminus U_\alpha)$。若所有 $U_\alpha \neq \varnothing$，则每个 $X \setminus U_\alpha$ 有限，故交集也有限
- (T3) 设 $U_1, \ldots, U_n$ 是开集，则 $X \setminus \bigcap_{i=1}^n U_i = \bigcup_{i=1}^n (X \setminus U_i)$ 是有限集的并，故有限

### 1.1.2 开集、闭集与邻域

**闭集**

定义 1.2（闭集） 设 $(X, \mathcal{T})$ 是拓扑空间，子集 $F \subseteq X$ 称为**闭集**（closed set），如果其补集 $X \setminus F$ 是开集。

**闭集的性质**：

定理 1.1 
- $\varnothing$ 和 $X$ 都是闭集
- 任意闭集的交是闭集
- 有限个闭集的并是闭集

**重要说明**：开集和闭集不是互斥的概念。一个集合可以同时是开集和闭集（称为**既开又闭集**或**clopen 集**），也可以既不是开集也不是闭集。

**例 1.5** 在离散拓扑中，所有子集既开又闭。在 $\mathbb{R}$ 的标准拓扑中，$[0, 1)$ 既不是开集也不是闭集。

**邻域**

定义 1.3（邻域） 设 $(X, \mathcal{T})$ 是拓扑空间，$x \in X$。子集 $N \subseteq X$ 称为 $x$ 的**邻域**（neighborhood），如果存在开集 $U$ 使得 $x \in U \subseteq N$。

**注**：有些教材定义邻域为包含 $x$ 的开集。我们这里采用更一般的定义（邻域可以不是开的），称开邻域为包含 $x$ 的开集。

**邻域系**

点 $x$ 的所有邻域构成 $x$ 的**邻域系**，记为 $\mathcal{N}(x)$。

邻域系的性质：
- $x \in N$ 对所有 $N \in \mathcal{N}(x)$
- 若 $N \in \mathcal{N}(x)$ 且 $N \subseteq M$，则 $M \in \mathcal{N}(x)$
- 若 $N_1, N_2 \in \mathcal{N}(x)$，则 $N_1 \cap N_2 \in \mathcal{N}(x)$
- 若 $N \in \mathcal{N}(x)$，则存在 $M \in \mathcal{N}(x)$ 使得 $M \subseteq N$ 且 $M \in \mathcal{N}(y)$ 对所有 $y \in M$

### 1.1.3 拓扑的比较

定义 1.4（拓扑的粗细） 设 $\mathcal{T}_1$ 和 $\mathcal{T}_2$ 是 $X$ 上的两个拓扑：
- 若 $\mathcal{T}_1 \subseteq \mathcal{T}_2$，称 $\mathcal{T}_1$ 比 $\mathcal{T}_2$ **粗**（coarser）或 $\mathcal{T}_2$ 比 $\mathcal{T}_1$ **细**（finer）
- 若 $\mathcal{T}_1 \subsetneq \mathcal{T}_2$，称 $\mathcal{T}_1$ **严格粗于** $\mathcal{T}_2$

**例 1.6** 
- 平凡拓扑是任何拓扑的子集，因此是最粗的拓扑
- 离散拓扑包含任何拓扑，因此是最细的拓扑
- 在 $\mathbb{R}$ 上，标准拓扑比余有限拓扑细

### 1.1.4 拓扑基

定义 1.5（拓扑基） 设 $(X, \mathcal{T})$ 是拓扑空间，子集族 $\mathcal{B} \subseteq \mathcal{T}$ 称为 $\mathcal{T}$ 的**基**（basis），如果：

- 对每个 $x \in X$，存在 $B \in \mathcal{B}$ 使得 $x \in B$

- 若 $x \in B_1 \cap B_2$ 且 $B_1, B_2 \in \mathcal{B}$，则存在 $B_3 \in \mathcal{B}$ 使得 $x \in B_3 \subseteq B_1 \cap B_2$

**由基生成拓扑**

定理 1.2 若 $\mathcal{B}$ 满足 (B1) 和 (B2)，则所有 $\mathcal{B}$ 中元素的并构成的集合族：
$$\mathcal{T} = \left\{\bigcup_{\alpha \in I} B_\alpha : B_\alpha \in \mathcal{B}\right\}$$

是 $X$ 上的拓扑，且 $\mathcal{B}$ 是 $\mathcal{T}$ 的基。

**例 1.7 欧几里得拓扑的基**

在 $\mathbb{R}^n$ 中，所有开球 $\{B(x, r) : x \in \mathbb{R}^n, r > 0\}$ 构成标准拓扑的基。

实际上，只需有理中心点和有理半径的开球即可：$\{B(q, r) : q \in \mathbb{Q}^n, r \in \mathbb{Q}, r > 0\}$。这说明 $\mathbb{R}^n$ 有可数基（第二可数性）。

**例 1.8 乘积拓扑的基**

设 $(X, \mathcal{T}_X)$ 和 $(Y, \mathcal{T}_Y)$ 是拓扑空间。$X \times Y$ 上的**乘积拓扑**以：

$$\mathcal{B} = \{U \times V : U \in \mathcal{T}_X, V \in \mathcal{T}_Y\}$$

为基。

---

## 1.2 连续性和同胚

### 1.2.1 连续映射的定义

**定义 1.6（连续映射）** 设 $(X, \mathcal{T}_X)$ 和 $(Y, \mathcal{T}_Y)$ 是拓扑空间，映射 $f: X \to Y$ 称为**连续的**（continuous），如果对于每个开集 $V \in \mathcal{T}_Y$，其原像 $f^{-1}(V) \in \mathcal{T}_X$。

**注**：这里的 $f^{-1}(V) = \{x \in X : f(x) \in V\}$ 是原像，不要求 $f$ 可逆。

**点态连续性**

定义 1.7（在一点连续） 映射 $f: X \to Y$ 在点 $x \in X$ 连续，如果对于 $f(x)$ 的任意邻域 $V$，存在 $x$ 的邻域 $U$ 使得 $f(U) \subseteq V$。

定理 1.3 $f: X \to Y$ 连续当且仅当 $f$ 在 $X$ 的每一点连续。

**连续性的等价刻画**

定理 1.4 对于映射 $f: X \to Y$，以下条件等价：
- $f$ 连续
- 对任意闭集 $F \subseteq Y$，$f^{-1}(F)$ 是 $X$ 中的闭集
- 对任意 $A \subseteq X$，$f(\overline{A}) \subseteq \overline{f(A)}$

### 1.2.2 同胚映射

**定义 1.8（同胚）** 映射 $f: X \to Y$ 称为**同胚**（homeomorphism），如果：
- $f$ 是双射（一一对应）
- $f$ 连续
- $f^{-1}$ 连续

如果存在同胚 $f: X \to Y$，称 $X$ 和 $Y$ **同胚**（homeomorphic）或**拓扑等价**，记为 $X \cong Y$。

**拓扑不变量**

定义 1.9（拓扑不变量） 空间的某个性质称为**拓扑不变量**（topological invariant），如果同胚的空间具有相同的该性质。

例如：
- 连通性、紧致性都是拓扑不变量
- 维数（适当定义下）是拓扑不变量
- "有界性"不是拓扑不变量（$(0, 1) \cong \mathbb{R}$）

**例 1.9 开区间与实数线同胚**

映射 $f: (-\pi/2, \pi/2) \to \mathbb{R}$，$f(x) = \tan(x)$ 是同胚。因此任何开区间 $(a, b)$ 都与 $\mathbb{R}$ 同胚。

**例 1.10 圆周与区间不同胚**

$S^1 = \{(x, y) \in \mathbb{R}^2 : x^2 + y^2 = 1\}$ 与任何区间都不同胚（因为去掉一点后连通性不同）。

### 1.2.3 拓扑嵌入

定义 1.10（嵌入） 映射 $f: X \to Y$ 称为**嵌入**（embedding），如果：
- $f$ 是单射
- $f: X \to f(X)$ 是同胚（其中 $f(X)$ 有子空间拓扑）

嵌入将 $X$ "放入" $Y$ 中，保持拓扑结构。

**例 1.11** 
- $f: [0, 1) \to S^1$，$f(t) = (\cos(2\pi t), \sin(2\pi t))$ 是连续双射但不是同胚（逆映射不连续）
- 限制在 $[0, 1/2)$ 上是嵌入

---

## 1.3 拓扑空间的基本性质

### 1.3.1 分离公理

分离公理描述拓扑空间区分点和闭集的能力。

**定义 1.11（分离公理）** 设 $(X, \mathcal{T})$ 是拓扑空间：

- **T0（柯尔莫果洛夫）**：对任意不同点 $x \neq y$，存在开集包含其中一个但不包含另一个

- **T1（弗雷歇）**：对任意不同点 $x \neq y$，存在开集 $U$ 使得 $x \in U, y \notin U$，也存在开集 $V$ 使得 $y \in V, x \notin V$

- **T2（豪斯多夫）**：对任意不同点 $x \neq y$，存在不相交的开集 $U, V$ 使得 $x \in U, y \in V$

- **T3（正则）**：T1 且对任意闭集 $F$ 和点 $x \notin F$，存在不相交的开集分离 $F$ 和 $x$

- **T4（正规）**：T1 且对任意不相交闭集 $F_1, F_2$，存在不相交的开集分离它们

**关系**：T4 $\Rightarrow$ T3 $\Rightarrow$ T2 $\Rightarrow$ T1 $\Rightarrow$ T0

**例 1.12**
- 余有限拓扑是 T1 但不是 T2（除非 $X$ 有限）
- 度量空间满足所有分离公理（是 T4 的）
- 欧几里得空间 $\mathbb{R}^n$ 是 T4 的

### 1.3.2 连通性

**定义 1.12（连通空间）** 拓扑空间 $X$ 称为**连通的**（connected），如果不存在非空开集 $U, V$ 使得 $U \cap V = \varnothing$ 且 $U \cup V = X$。

等价地，$X$ 连通当且仅当 $X$ 的既开又闭子集只有 $\varnothing$ 和 $X$。

**连通分支**

定义 1.13（连通分支） 点 $x \in X$ 的**连通分支**是包含 $x$ 的最大连通子集。

连通分支构成 $X$ 的划分（互不相交，并集为 $X$）。

**道路连通**

定义 1.14（道路连通） 空间 $X$ 称为**道路连通的**（path-connected），如果对任意 $x, y \in X$，存在连续映射 $\gamma: [0, 1] \to X$ 使得 $\gamma(0) = x, \gamma(1) = y$。

定理 1.5 道路连通空间是连通的。反之不成立。

**例 1.13** 拓扑学家的正弦曲线是连通但不是道路连通的经典例子。

### 1.3.3 紧致性

**定义 1.15（开覆盖与紧致性）** 设 $X$ 是拓扑空间：
- 子集族 $\{U_\alpha\}_{\alpha \in I}$ 称为 $X$ 的**开覆盖**（open cover），如果每个 $U_\alpha$ 是开集且 $\bigcup_\alpha U_\alpha = X$
- $X$ 称为**紧致的**（compact），如果每个开覆盖都有有限子覆盖

**紧致性的等价刻画**

在度量空间中（特别是 $\mathbb{R}^n$ 中）：

定理 1.6（海涅-博雷尔） 子集 $K \subseteq \mathbb{R}^n$ 是紧致的当且仅当 $K$ 是闭且有界的。

**紧致性的性质**

定理 1.7 
- 紧致空间的闭子集是紧致的
- 紧致空间的连续像是紧致的
- 紧致空间的有限积是紧致的（吉洪诺夫定理：任意积也是紧致的）

**局部紧致**

定义 1.16（局部紧致） 空间 $X$ 称为**局部紧致的**（locally compact），如果每个点都有紧邻域。

$\mathbb{R}^n$ 是局部紧致的（每个闭球是紧致的）。

### 1.3.4 可数性公理

**第一可数性**

定义 1.17（第一可数） 空间 $X$ 称为**第一可数的**（first-countable），如果每个点都有可数的邻域基。

度量空间都是第一可数的（以有理半径的开球为可数基）。

**第二可数性**

定义 1.18（第二可数） 空间 $X$ 称为**第二可数的**（second-countable），如果 $X$ 有可数的拓扑基。

定理 1.8 第二可数空间是第一可数的、可分的（有可数的稠密子集）、Lindelöf 的（每个开覆盖有可数子覆盖）。

**例 1.14** $\mathbb{R}^n$ 是第二可数的（有理中心、有理半径的开球构成可数基）。

---

## 1.4 本章小结

本章介绍了拓扑空间的基础理论：

1. **拓扑空间**：由集合 $X$ 和满足三公理的开集族 $\mathcal{T}$ 组成，推广了"开集"的概念。

2. **基本概念**：
   - 开集、闭集、邻域
   - 拓扑基：生成拓扑的子集族
   - 拓扑的比较：粗细关系

3. **连续性与同胚**：
   - 连续映射：开集的原像是开集
   - 同胚：双射且双向连续，表示拓扑等价
   - 拓扑不变量：在同胚下保持的性质

4. **基本性质**：
   - 分离公理（T0-T4）：区分点和闭集的能力
   - 连通性：空间不能被分割为不相交的开集
   - 紧致性：每个开覆盖有有限子覆盖
   - 可数性：第一可数、第二可数

这些概念为后续学习流形、微分几何奠定了严格的拓扑基础。

---

## 习题

### 基础题

1.1 验证余可数拓扑（cocountable topology）确实是拓扑：设 $X$ 是不可数集，$\mathcal{T} = \{U \subseteq X : X \setminus U \text{ 是可数集}\} \cup \{\varnothing\}$。

1.2 证明：$f: X \to Y$ 连续当且仅当对任意 $A \subseteq X$，$f(\overline{A}) \subseteq \overline{f(A)}$。

1.3 证明：连通空间的连续像是连通的。

1.4 证明：紧致空间的闭子集是紧致的。

### 提高题

1.5 证明：$\mathbb{R}$ 上的标准拓扑比余有限拓扑严格细。

1.6 构造一个 T1 但不是 T2 的拓扑空间。

1.7 证明：道路连通空间是连通的。给出连通但不是道路连通的例子。

1.8 证明：紧致豪斯多夫空间是正则的（实际上是正规的）。

### 编程题

1.9 编写程序验证有限拓扑空间的性质：给定有限集 $X$ 和子集族，验证是否满足拓扑三公理。

1.10 实现基本拓扑运算：计算给定子集的闭包、内部、边界。

---

## 参考文献

1. Munkres J R. Topology[M]. Pearson, 2000.
2. Kelley J L. General Topology[M]. Springer, 1975.
3. Willard S. General Topology[M]. Dover, 2004.
4. 熊金城. 点集拓扑讲义[M]. 高等教育出版社, 2011.
