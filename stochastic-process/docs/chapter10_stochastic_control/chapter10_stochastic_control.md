# 第10章 随机控制和决策

## 10.1 随机控制的基本概念

### 10.1.1 随机控制系统的定义

**定义 10.1（随机控制系统）** **随机控制系统**由以下要素组成：
- 状态 $x_t \in \mathcal{X}$
- 控制输入 $u_t \in \mathcal{U}$
- 状态转移概率 $p(x_{t+1} | x_t, u_t)$
- 代价函数 $c(x_t, u_t)$

**目标**：寻找控制策略 $\pi$，最小化期望累积代价。

### 10.1.2 随机最优控制的基本框架

**有限时域问题**：
$$J^\pi(x_0) = E\left[\sum_{t=0}^{T-1} c(x_t, \pi_t(x_t)) + c_T(x_T)\right]$$

**无限时域折扣问题**：
$$J^\pi(x_0) = E\left[\sum_{t=0}^{\infty} \gamma^t c(x_t, \pi(x_t))\right]$$

其中 $\gamma \in (0, 1)$ 是折扣因子。

### 10.1.3 动态规划在随机控制中的应用

**贝尔曼方程**：
$$V(x) = \min_u \left[c(x, u) + \gamma E[V(x') | x, u]\right]$$

最优策略：
$$\pi^*(x) = \arg\min_u \left[c(x, u) + \gamma E[V(x') | x, u]\right]$$

## 10.2 马尔可夫决策过程

### 10.2.1 MDP的定义和要素

**定义 10.2（马尔可夫决策过程）** **马尔可夫决策过程**（MDP）由以下要素组成：
- 状态空间 $\mathcal{S}$
- 动作空间 $\mathcal{A}$
- 转移概率 $P(s' | s, a)$
- 奖励函数 $R(s, a, s')$
- 折扣因子 $\gamma$

**马尔可夫性**：
$$P(s_{t+1} | s_t, a_t, s_{t-1}, a_{t-1}, \ldots) = P(s_{t+1} | s_t, a_t)$$

### 10.2.2 值迭代和策略迭代算法

**值迭代**：

**算法 10.1（值迭代）**

1. 初始化 $V_0(s) = 0$ 对所有 $s$
2. 重复直到收敛：
   $$V_{k+1}(s) = \max_a \sum_{s'} P(s' | s, a) [R(s, a, s') + \gamma V_k(s')]$$
3. 提取策略：
   $$\pi(s) = \arg\max_a \sum_{s'} P(s' | s, a) [R(s, a, s') + \gamma V(s')]$$

**策略迭代**：

**算法 10.2（策略迭代）**

1. 初始化策略 $\pi_0$
2. 重复：
   - **策略评估**：求解线性方程组
     $$V^\pi(s) = \sum_{s'} P(s' | s, \pi(s)) [R(s, \pi(s), s') + \gamma V^\pi(s')]$$
   - **策略改进**：
     $$\pi'(s) = \arg\max_a \sum_{s'} P(s' | s, a) [R(s, a, s') + \gamma V^\pi(s')]$$
   - 如果 $\pi' = \pi$，停止；否则 $\pi \leftarrow \pi'$

### 10.2.3 部分可观测马尔可夫决策过程（POMDP）

**定义 10.3（POMDP）** **部分可观测马尔可夫决策过程**扩展了MDP，增加了：
- 观测空间 $\mathcal{O}$
- 观测概率 $P(o | s, a)$

在POMDP中，智能体不能直接观察状态，只能通过观测推断。

**信念状态**：
$$b_t(s) = P(s_t = s | o_{1:t}, a_{1:t-1})$$

信念更新：
$$b_{t+1}(s') = \eta P(o_{t+1} | s', a_t) \sum_s P(s' | s, a_t) b_t(s)$$

POMDP可以转化为信念空间上的MDP，但信念空间是连续的，求解更加困难。

## 10.3 机器人决策和规划

### 10.3.1 基于随机过程的路径规划

**随机路径规划**考虑环境不确定性：
- 障碍物位置不确定
- 地形 traversability 不确定
- 动态障碍物

**方法**：
- **机会约束规划**：约束以高概率满足
- **鲁棒规划**：考虑最坏情况
- **基于采样的方法**：RRT、PRM等

### 10.3.2 机器人导航中的决策方法

**基于MDP的导航**：
- 状态：机器人位置、方向
- 动作：前进、转向
- 奖励：到达目标、避免碰撞

**信息增益导航**：在探索未知环境时，选择最大化信息增益的动作。

### 10.3.3 多机器人协调的随机决策

**多智能体MDP**（MMDP）：
- 联合状态空间：$\mathcal{S} = \mathcal{S}_1 \times \cdots \times \mathcal{S}_n$
- 联合动作空间：$\mathcal{A} = \mathcal{A}_1 \times \cdots \times \mathcal{A}_n$

**分布式决策**：每个机器人基于局部信息决策，通过通信协调。

**博弈论方法**：将多机器人协调建模为博弈，寻找纳什均衡。

## 10.4 本章小结

本章介绍了随机控制和决策的基本概念：

1. **随机控制**：在不确定性下的最优控制问题
2. **马尔可夫决策过程**：随机决策的数学框架
3. **值迭代和策略迭代**：求解MDP的基本算法
4. **POMDP**：部分可观测情况下的决策
5. **机器人应用**：路径规划、导航、多机器人协调

随机决策理论为机器人在不确定环境中的自主决策提供了理论基础。

## 10.5 参考文献

1. Bertsekas, D. P. (2017). *Dynamic Programming and Optimal Control* (4th ed.). Athena Scientific.
2. Sutton, R. S., & Barto, A. G. (2018). *Reinforcement Learning: An Introduction* (2nd ed.). MIT Press.
3. Kaelbling, L. P., Littman, M. L., & Cassandra, A. R. (1998). Planning and acting in partially observable stochastic domains. *Artificial Intelligence*, 101(1-2), 99-134.

## 10.6 练习题

1. 推导贝尔曼方程。

2. 比较值迭代和策略迭代的优缺点。

3. 解释POMDP中信念状态的作用。

4. 设计一个用于机器人导航的MDP模型。

5. 讨论多机器人协调中的博弈论方法。
