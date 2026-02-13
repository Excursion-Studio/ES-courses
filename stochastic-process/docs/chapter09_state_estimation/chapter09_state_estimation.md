# 第9章 状态估计算法

## 9.1 卡尔曼滤波

### 9.1.1 线性系统的卡尔曼滤波

卡尔曼滤波（Kalman Filter）是线性高斯系统最优的状态估计算法。

**系统模型**：
$$x_t = A x_{t-1} + B u_t + w_t, \quad w_t \sim N(0, Q)$$
$$z_t = H x_t + v_t, \quad v_t \sim N(0, R)$$

**算法 9.1（卡尔曼滤波）**

**预测步**：
$$\bar{\mu}_t = A \mu_{t-1} + B u_t$$
$$\bar{\Sigma}_t = A \Sigma_{t-1} A^T + Q$$

**更新步**：
$$K_t = \bar{\Sigma}_t H^T (H \bar{\Sigma}_t H^T + R)^{-1}$$
$$\mu_t = \bar{\mu}_t + K_t (z_t - H \bar{\mu}_t)$$
$$\Sigma_t = (I - K_t H) \bar{\Sigma}_t$$

其中 $K_t$ 是**卡尔曼增益**。

**定理 9.1（卡尔曼滤波的最优性）** 对于线性高斯系统，卡尔曼滤波器是最小均方误差估计器。

### 9.1.2 卡尔曼滤波的推导和解释

**推导思路**：
1. 预测步：根据运动模型传播均值和协方差
2. 更新步：利用观测信息修正预测

**卡尔曼增益的意义**：
$$K_t = \frac{\text{预测不确定性}}{\text{预测不确定性} + \text{观测噪声}}$$

增益越大，越信任观测；增益越小，越信任预测。

**信息形式**：卡尔曼滤波也可以用信息矩阵（协方差的逆）表示。

### 9.1.3 卡尔曼滤波的扩展：EKF和UKF

**扩展卡尔曼滤波（EKF）**：

对于非线性系统：
$$x_t = f(x_{t-1}, u_t) + w_t$$
$$z_t = h(x_t) + v_t$$

通过一阶泰勒展开线性化：
$$A_t = \left.\frac{\partial f}{\partial x}\right|_{\mu_{t-1}, u_t}, \quad H_t = \left.\frac{\partial h}{\partial x}\right|_{\bar{\mu}_t}$$

**无味卡尔曼滤波（UKF）**：

使用**无味变换**（Unscented Transform）传播均值和协方差，避免了线性化误差。

**无味点**（Sigma Points）：
$$\chi^{(0)} = \mu$$
$$\chi^{(i)} = \mu + \sqrt{(n+\lambda)\Sigma}_i, \quad i = 1, \ldots, n$$
$$\chi^{(i+n)} = \mu - \sqrt{(n+\lambda)\Sigma}_i, \quad i = 1, \ldots, n$$

通过非线性函数传播无味点，然后重构均值和协方差。

## 9.2 粒子滤波

### 9.2.1 粒子滤波的基本思想

粒子滤波（Particle Filter）使用一组随机样本（粒子）表示后验分布，适用于非线性非高斯系统。

**算法 9.2（粒子滤波）**

1. **采样**（预测）：从提议分布采样
   $$x_t^{(i)} \sim p(x_t | x_{t-1}^{(i)}, u_t)$$

2. **重要性权重**：
   $$w_t^{(i)} = p(z_t | x_t^{(i)})$$

3. **归一化**：
   $$\tilde{w}_t^{(i)} = \frac{w_t^{(i)}}{\sum_j w_t^{(j)}}$$

4. **重采样**：根据权重重采样，避免粒子退化

### 9.2.2 重要性采样和重采样

**重要性采样**：当无法直接从目标分布采样时，从提议分布采样并加权。

**粒子退化问题**：随着时间推移，少数粒子占据大部分权重。

**重采样方法**：
- **多项式重采样**：根据权重多项式采样
- **系统重采样**：等间距采样
- **残差重采样**：减少方差

**有效粒子数**：
$$N_{\text{eff}} = \frac{1}{\sum_i (\tilde{w}^{(i)})^2}$$

当 $N_{\text{eff}} < N_{\text{threshold}}$ 时进行重采样。

### 9.2.3 粒子滤波的改进算法

**正则化粒子滤波**：在重采样后添加核平滑，增加粒子多样性。

**Rao-Blackwellized粒子滤波**：对于条件线性高斯部分，使用卡尔曼滤波；对于非线性部分，使用粒子滤波。

**快速SLAM**：使用Rao-Blackwellized粒子滤波同时估计机器人轨迹和地图。

## 9.3 其他滤波方法

### 9.3.1 扩展卡尔曼滤波

EKF是应用最广泛的非线性滤波方法。

**优点**：
- 计算效率高
- 实现简单

**缺点**：
- 线性化引入误差
- 对于强非线性系统可能发散

### 9.3.2 无味卡尔曼滤波

UKF通过无味变换避免了线性化。

**优点**：
- 二阶精度
- 不需要计算雅可比矩阵

**缺点**：
- 计算量大于EKF
- 对于高维系统，无味点数量大

### 9.3.3 信息滤波

**信息滤波**（Information Filter）是卡尔曼滤波的对偶形式，使用信息矩阵（协方差的逆）。

**预测步**：
$$\bar{\Omega}_t = (A \Omega_{t-1}^{-1} A^T + Q)^{-1}$$
$$\bar{\xi}_t = \bar{\Omega}_t (A \Omega_{t-1}^{-1} \xi_{t-1} + B u_t)$$

**更新步**：
$$\Omega_t = \bar{\Omega}_t + H^T R^{-1} H$$
$$\xi_t = \bar{\xi}_t + H^T R^{-1} z_t$$

信息滤波在多传感器融合中具有优势。

## 9.4 本章小结

本章介绍了机器人状态估计的主要算法：

1. **卡尔曼滤波**：线性高斯系统的最优滤波器
2. **EKF和UKF**：非线性系统的近似滤波器
3. **粒子滤波**：非线性非高斯系统的蒙特卡洛方法
4. **信息滤波**：卡尔曼滤波的对偶形式

这些算法在机器人定位、SLAM、目标跟踪等任务中有广泛应用。

## 9.5 参考文献

1. Kalman, R. E. (1960). A new approach to linear filtering and prediction problems. *Journal of Basic Engineering*, 82(1), 35-45.
2. Julier, S. J., & Uhlmann, J. K. (1997). New extension of the Kalman filter to nonlinear systems. *Signal Processing, Sensor Fusion, and Target Recognition VI*, 3068, 182-193.
3. Arulampalam, M. S., et al. (2002). A tutorial on particle filters for online nonlinear/non-Gaussian Bayesian tracking. *IEEE Transactions on Signal Processing*, 50(2), 174-188.

## 9.6 练习题

1. 推导卡尔曼滤波的预测步和更新步。

2. 证明卡尔曼滤波是最小均方误差估计器。

3. 对于非线性系统，比较EKF和UKF的优缺点。

4. 解释粒子滤波中的粒子退化问题及重采样的作用。

5. 实现一个用于机器人定位的粒子滤波器。
