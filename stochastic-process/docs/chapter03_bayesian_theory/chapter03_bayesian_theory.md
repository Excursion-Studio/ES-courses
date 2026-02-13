# 第3章 条件概率与贝叶斯理论

## 本章导读

贝叶斯理论是现代概率统计和机器学习的重要基础，它提供了一种在观测数据下更新信念的数学框架。本章深入探讨条件概率、贝叶斯推断和贝叶斯网络等核心概念，并重点介绍其在机器人状态估计中的应用。通过本章学习，你将掌握贝叶斯推断的基本方法，理解卡尔曼滤波和粒子滤波的数学原理。

---

## 3.1 条件概率的深入理解

### 3.1.1 条件概率的链式法则

回顾条件概率的定义：
$$P(A|B) = \frac{P(A \cap B)}{P(B)}, \quad P(B) > 0$$

**乘法公式（链式法则）**

定理 3.1（链式法则） 对于事件 $A_1, A_2, \ldots, A_n$，若 $P(A_1 A_2 \cdots A_{n-1}) > 0$，则：
$$P(A_1 A_2 \cdots A_n) = P(A_1) P(A_2|A_1) P(A_3|A_1 A_2) \cdots P(A_n|A_1 \cdots A_{n-1})$$

**条件概率的公理化**

定理 3.2 固定事件 $B$ 且 $P(B) > 0$，则 $P(\cdot|B)$ 满足概率三公理：
1. $P(A|B) \geq 0$
2. $P(\Omega|B) = 1$
3. 可列可加性成立

### 3.1.2 条件独立性

定义 3.1（条件独立性） 事件 $A$ 和 $B$ 在给定 $C$ 下**条件独立**，如果：
$$P(A \cap B|C) = P(A|C) P(B|C)$$

或等价地：
$$P(A|B, C) = P(A|C)$$

**注意**：条件独立不意味着独立，反之亦然。

**例 3.1** 抛两枚硬币，$A$="第一枚正面"，$B$="第二枚正面"，$C$="至少一枚正面"。

$A$ 和 $B$ 独立，但在给定 $C$ 下不条件独立。

---

## 3.2 贝叶斯推断

### 3.2.1 贝叶斯定理的推导

由条件概率定义：
$$P(A|B) = \frac{P(A \cap B)}{P(B)}, \quad P(B|A) = \frac{P(A \cap B)}{P(A)}$$

联立得贝叶斯定理：
$$P(A|B) = \frac{P(B|A) P(A)}{P(B)}$$

**贝叶斯术语**
- $P(A)$：**先验概率**（prior）
- $P(B|A)$：**似然函数**（likelihood）
- $P(A|B)$：**后验概率**（posterior）
- $P(B)$：**证据**（evidence）或**边缘似然**

### 3.2.2 贝叶斯推断的基本框架

**参数估计视角**

设 $\theta$ 是待估参数，$D$ 是观测数据：
$$P(\theta|D) = \frac{P(D|\theta) P(\theta)}{P(D)} = \frac{P(D|\theta) P(\theta)}{\int P(D|\theta) P(\theta) d\theta}$$

**预测分布**

对新观测 $x_{new}$ 的预测：
$$P(x_{new}|D) = \int P(x_{new}|\theta) P(\theta|D) d\theta$$

### 3.2.3 共轭先验

定义 3.2（共轭先验） 若先验分布 $P(\theta)$ 和后验分布 $P(\theta|D)$ 属于同一分布族，则称该先验为**共轭先验**。

**常见共轭先验**

| 似然 | 共轭先验 | 后验 |
|------|----------|------|
| 二项分布 | Beta分布 | Beta分布 |
| 泊松分布 | Gamma分布 | Gamma分布 |
| 正态分布（已知方差） | 正态分布 | 正态分布 |
| 正态分布（已知均值） | 逆Gamma分布 | 逆Gamma分布 |

**例 3.2 Beta-Binomial 模型**

设 $X \sim \text{Binomial}(n, \theta)$，先验 $\theta \sim \text{Beta}(\alpha, \beta)$。

后验分布：
$$\theta|X \sim \text{Beta}(\alpha + X, \beta + n - X)$$

**例 3.3 正态-正态模型**

设 $X_i \sim N(\theta, \sigma^2)$（$\sigma^2$ 已知），先验 $\theta \sim N(\mu_0, \sigma_0^2)$。

后验分布：
$$\theta|\{X_i\} \sim N(\mu_n, \sigma_n^2)$$

其中：
$$\frac{1}{\sigma_n^2} = \frac{1}{\sigma_0^2} + \frac{n}{\sigma^2}$$
$$\mu_n = \frac{\frac{\mu_0}{\sigma_0^2} + \frac{n\bar{X}}{\sigma^2}}{\frac{1}{\sigma_0^2} + \frac{n}{\sigma^2}}$$

### 3.2.4 最大后验估计（MAP）

定义 3.3（MAP估计） **最大后验估计**定义为：
$$\hat{\theta}_{MAP} = \arg\max_\theta P(\theta|D) = \arg\max_\theta P(D|\theta) P(\theta)$$

**与MLE的比较**
- 最大似然估计（MLE）：$\hat{\theta}_{MLE} = \arg\max_\theta P(D|\theta)$
- MAP 增加了先验正则化

---

## 3.3 贝叶斯网络

### 3.3.1 概率图模型简介

**定义 3.4（贝叶斯网络）** **贝叶斯网络**是有向无环图（DAG），其中：
- 节点表示随机变量
- 边表示条件依赖关系

**因子分解**

若网络结构为 $G$，则联合分布可分解为：
$$P(X_1, X_2, \ldots, X_n) = \prod_{i=1}^n P(X_i|\text{Pa}(X_i))$$

其中 $\text{Pa}(X_i)$ 是 $X_i$ 的父节点。

### 3.3.2 d-分离

定义 3.5（d-分离） 在贝叶斯网络中，节点集 $A$ 和 $B$ 被节点集 $C$ **d-分离**，如果所有 $A$ 到 $B$ 的路径都被 $C$ **阻塞**。

**路径阻塞规则**：
- 链式结构 $A \to C \to B$：若 $C$ 被观测，则阻塞
- 分叉结构 $A \leftarrow C \to B$：若 $C$ 被观测，则阻塞
- 对撞结构 $A \to C \leftarrow B$：若 $C$ 或其子孙被观测，则**不**阻塞

**定理 3.3** 若 $A$ 和 $B$ 被 $C$ d-分离，则 $A \perp\!\!\!\perp B | C$（条件独立）。

### 3.3.3 隐马尔可夫模型（HMM）

**定义 3.6（HMM）** **隐马尔可夫模型**是具有以下结构的贝叶斯网络：
- 隐藏状态序列 $X_1, X_2, \ldots, X_T$
- 观测序列 $Y_1, Y_2, \ldots, Y_T$
- 马尔可夫性：$X_t$ 只依赖于 $X_{t-1}$
- 观测独立性：$Y_t$ 只依赖于 $X_t$

**HMM的三个基本问题**

1. **评估问题**：给定模型 $\lambda = (A, B, \pi)$ 和观测 $O$，求 $P(O|\lambda)$
   - 解法：前向算法

2. **解码问题**：给定模型和观测，求最可能的状态序列
   - 解法：Viterbi算法

3. **学习问题**：给定观测，估计模型参数
   - 解法：Baum-Welch算法（EM算法）

---

## 3.4 贝叶斯滤波

### 3.4.1 贝叶斯滤波框架

考虑动态系统：
- 状态转移：$x_t \sim P(x_t|x_{t-1})$
- 观测模型：$z_t \sim P(z_t|x_t)$

**贝叶斯滤波递推**

**预测步**：
$$P(x_t|z_{1:t-1}) = \int P(x_t|x_{t-1}) P(x_{t-1}|z_{1:t-1}) dx_{t-1}$$

**更新步**：
$$P(x_t|z_{1:t}) = \frac{P(z_t|x_t) P(x_t|z_{1:t-1})}{P(z_t|z_{1:t-1})}$$

### 3.4.2 卡尔曼滤波（KF）

对于线性高斯系统：
- $x_t = A x_{t-1} + B u_t + w_t$，$w_t \sim N(0, Q)$
- $z_t = H x_t + v_t$，$v_t \sim N(0, R)$

**卡尔曼滤波算法**

**预测**：
$$\hat{x}_{t|t-1} = A \hat{x}_{t-1|t-1} + B u_t$$
$$P_{t|t-1} = A P_{t-1|t-1} A^T + Q$$

**更新**：
$$K_t = P_{t|t-1} H^T (H P_{t|t-1} H^T + R)^{-1}$$
$$\hat{x}_{t|t} = \hat{x}_{t|t-1} + K_t (z_t - H \hat{x}_{t|t-1})$$
$$P_{t|t} = (I - K_t H) P_{t|t-1}$$

其中 $K_t$ 是**卡尔曼增益**。

### 3.4.3 扩展卡尔曼滤波（EKF）

对于非线性系统：
- $x_t = f(x_{t-1}, u_t) + w_t$
- $z_t = h(x_t) + v_t$

**线性化**：
$$F_t = \left.\frac{\partial f}{\partial x}\right|_{\hat{x}_{t-1|t-1}}, \quad H_t = \left.\frac{\partial h}{\partial x}\right|_{\hat{x}_{t|t-1}}$$

然后应用标准卡尔曼滤波公式。

### 3.4.4 粒子滤波（PF）

对于非线性非高斯系统，使用蒙特卡洛方法近似后验分布。

**基本思想**

用一组带权粒子 $\{(x_t^{(i)}, w_t^{(i)})\}_{i=1}^N$ 近似后验分布：
$$P(x_t|z_{1:t}) \approx \sum_{i=1}^N w_t^{(i)} \delta(x_t - x_t^{(i)})$$

**序贯重要性采样（SIS）**

1. 从提议分布采样：$x_t^{(i)} \sim q(x_t|x_{t-1}^{(i)}, z_t)$
2. 更新权重：$w_t^{(i)} \propto w_{t-1}^{(i)} \frac{P(z_t|x_t^{(i)}) P(x_t^{(i)}|x_{t-1}^{(i)})}{q(x_t^{(i)}|x_{t-1}^{(i)}, z_t)}$

**重采样**

解决权重退化问题：根据权重重新采样粒子，使权重均匀分布。

---

## 3.5 本章小结

本章深入探讨了贝叶斯理论及其应用：

1. **条件概率**：
   - 链式法则
   - 条件独立性

2. **贝叶斯推断**：
   - 贝叶斯定理
   - 共轭先验
   - MAP估计

3. **贝叶斯网络**：
   - 概率图模型
   - d-分离
   - 隐马尔可夫模型

4. **贝叶斯滤波**：
   - 预测-更新框架
   - 卡尔曼滤波（线性高斯）
   - 扩展卡尔曼滤波（非线性）
   - 粒子滤波（非线性非高斯）

贝叶斯方法为机器人状态估计提供了统一的理论框架。

---

## 习题

### 基础题

3.1 证明：若 $A$ 和 $B$ 独立，则 $P(A|B) = P(A)$。

3.2 设 $\theta \sim \text{Beta}(1, 1)$（均匀先验），观测 $X \sim \text{Binomial}(10, \theta)$ 得到 $X = 3$。求后验分布。

3.3 证明 Beta 分布是二项分布的共轭先验。

3.4 推导卡尔曼滤波中的卡尔曼增益公式。

### 提高题

3.5 证明：在正态-正态模型中，后验均值是先验均值和样本均值的加权平均。

3.6 设计一个贝叶斯网络表示机器人定位问题。

3.7 推导扩展卡尔曼滤波的协方差更新公式。

3.8 分析粒子滤波中重采样的必要性。

### 编程题

3.9 实现卡尔曼滤波器，并在一维运动模型上测试。

3.10 实现粒子滤波器，用于机器人定位。

---

## 参考文献

1. Bishop C M. Pattern Recognition and Machine Learning[M]. Springer, 2006.
2. Murphy K P. Machine Learning: A Probabilistic Perspective[M]. MIT Press, 2012.
3. Thrun S, Burgard W, Fox D. Probabilistic Robotics[M]. MIT Press, 2005.
4. Gelman A, et al. Bayesian Data Analysis[M]. CRC Press, 2013.
