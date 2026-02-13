# 第7章 黎曼几何

## 7.1 黎曼度量

### 7.1.1 黎曼度量的定义

黎曼几何是研究装备了度量结构的微分流形的几何学。黎曼度量为流形上的每一点定义了一个内积，从而可以测量长度、角度和体积。

**定义 7.1（黎曼度量）** 设 $M$ 是n维光滑流形，$M$ 上的一个**黎曼度量** $g$ 是一个光滑的 $(0,2)$ 型张量场，满足：

1. **对称性**：对于任意 $X, Y \in \mathfrak{X}(M)$，有 $g(X, Y) = g(Y, X)$
2. **正定性**：对于任意 $X \in \mathfrak{X}(M)$，$g(X, X) \geq 0$，且 $g(X, X) = 0$ 当且仅当 $X = 0$

装备了黎曼度量的流形 $(M, g)$ 称为**黎曼流形**。

在局部坐标 $(x^1, \ldots, x^n)$ 下，黎曼度量可以表示为：
$$g = g_{ij} dx^i \otimes dx^j$$

其中 $g_{ij} = g\left(\frac{\partial}{\partial x^i}, \frac{\partial}{\partial x^j}\right)$ 是光滑函数，且矩阵 $(g_{ij})$ 是对称正定的。

**定义 7.2（线元）** 黎曼度量诱导的**线元**（Line Element）定义为：
$$ds^2 = g_{ij} dx^i dx^j$$

### 7.1.2 黎曼度量的例子

**例 7.1（欧几里得空间）** 在 $\mathbb{R}^n$ 上，标准黎曼度量为：
$$g_{\text{Euc}} = \delta_{ij} dx^i \otimes dx^j = \sum_{i=1}^n (dx^i)^2$$

对应的线元为 $ds^2 = \sum_{i=1}^n (dx^i)^2$。

**例 7.2（球面 $S^2$）** 使用球坐标 $(\theta, \phi)$，其中 $\theta \in [0, \pi]$ 是极角，$\phi \in [0, 2\pi)$ 是方位角，标准度量为：
$$g_{S^2} = d\theta \otimes d\theta + \sin^2\theta \, d\phi \otimes d\phi$$

即 $g_{\theta\theta} = 1$，$g_{\phi\phi} = \sin^2\theta$，$g_{\theta\phi} = g_{\phi\theta} = 0$。

**例 7.3（双曲平面）** Poincaré圆盘模型中，单位圆盘 $D = \{z \in \mathbb{C} : |z| < 1\}$ 上的双曲度量为：
$$g_{\text{hyp}} = \frac{4}{(1 - |z|^2)^2} (dx \otimes dx + dy \otimes dy)$$

**例 7.7.4（$SO(3)$ 上的生物度量）** 在旋转群 $SO(3)$ 上，常用的**生物度量**（Bi-invariant Metric）定义为：
$$g_R(X, Y) = \frac{1}{2}\text{tr}(X^T Y)$$

其中 $X, Y \in T_R SO(3)$ 被看作矩阵。

### 7.1.3 度量诱导的几何量

**定义 7.3（向量长度）** 切向量 $v \in T_p M$ 的**长度**（范数）定义为：
$$\|v\| = \sqrt{g_p(v, v)} = \sqrt{g_{ij} v^i v^j}$$

**定义 7.4（曲线长度）** 光滑曲线 $\gamma: [a, b] \to M$ 的**长度**为：
$$L(\gamma) = \int_a^b \|\dot{\gamma}(t)\| dt = \int_a^b \sqrt{g_{ij}(\gamma(t)) \dot{\gamma}^i(t) \dot{\gamma}^j(t)} dt$$

**定义 7.5（夹角）** 两个非零切向量 $v, w \in T_p M$ 之间的**夹角** $\theta$ 定义为：
$$\cos\theta = \frac{g_p(v, w)}{\|v\| \|w\|}$$

**定义 7.6（体积形式）** 在定向黎曼流形上，**体积形式**为：
$$dV = \sqrt{|g|} \, dx^1 \wedge \cdots \wedge dx^n$$

其中 $|g| = \det(g_{ij})$。

## 7.2 联络和协变导数

### 7.2.1 仿射联络的定义

在欧几里得空间中，我们可以平行移动向量而不改变其分量。在一般的流形上，需要引入**联络**的概念来定义向量的"平行移动"。

**定义 7.7（仿射联络）** 流形 $M$ 上的**仿射联络**（Affine Connection）是一个映射 $\nabla: \mathfrak{X}(M) \times \mathfrak{X}(M) \to \mathfrak{X}(M)$，记为 $(X, Y) \mapsto \nabla_X Y$，满足：

1. **关于第一个变量的 $C^\infty(M)$-线性性**：
   $$\nabla_{fX + gY} Z = f\nabla_X Z + g\nabla_Y Z$$

2. **关于第二个变量的 $\mathbb{R}$-线性性**：
   $$\nabla_X (aY + bZ) = a\nabla_X Y + b\nabla_X Z$$

3. **莱布尼茨法则**：
   $$\nabla_X (fY) = X(f)Y + f\nabla_X Y$$

$\nabla_X Y$ 称为 $Y$ 沿 $X$ 的**协变导数**。

### 7.2.2 列维-奇维塔联络

一般的仿射联络不具有唯一性。通过要求联络与度量相容且无挠，可以得到唯一的联络。

**定义 7.8（度量相容）** 联络 $\nabla$ 称为**与度量相容**，如果：
$$X(g(Y, Z)) = g(\nabla_X Y, Z) + g(Y, \nabla_X Z)$$

**定义 7.9（无挠）** 联络 $\nabla$ 称为**无挠**（Torsion-free），如果：
$$\nabla_X Y - \nabla_Y X = [X, Y]$$

**定理 7.1（列维-奇维塔联络的基本定理）** 在黎曼流形 $(M, g)$ 上，存在唯一的与度量相容且无挠的仿射联络，称为**列维-奇维塔联络**（Levi-Civita Connection）。

**克里斯托费尔符号**：在坐标基下，列维-奇维塔联络可以表示为：
$$\nabla_{\frac{\partial}{\partial x^i}} \frac{\partial}{\partial x^j} = \Gamma^k_{ij} \frac{\partial}{\partial x^k}$$

其中**克里斯托费尔符号**（Christoffel Symbols）为：
$$\Gamma^k_{ij} = \frac{1}{2} g^{kl} \left(\frac{\partial g_{il}}{\partial x^j} + \frac{\partial g_{jl}}{\partial x^i} - \frac{\partial g_{ij}}{\partial x^l}\right)$$

这里 $g^{kl}$ 是度量矩阵的逆矩阵的元素：$g^{kl}g_{lj} = \delta^k_j$。

### 7.2.3 协变导数的计算

**向量场的协变导数**：设 $Y = Y^j \frac{\partial}{\partial x^j}$，则：
$$\nabla_X Y = X(Y^k) \frac{\partial}{\partial x^k} + X^i Y^j \Gamma^k_{ij} \frac{\partial}{\partial x^k}$$

即 $(\nabla_X Y)^k = X(Y^k) + \Gamma^k_{ij} X^i Y^j$。

**张量场的协变导数**：对于一般的 $(r, s)$ 型张量场 $T$，协变导数 $\nabla T$ 是一个 $(r, s+1)$ 型张量场。

**例 7.5** 在欧几里得空间 $\mathbb{R}^n$ 中，$g_{ij} = \delta_{ij}$ 是常数，因此 $\Gamma^k_{ij} = 0$，协变导数退化为普通偏导数。

**例 7.6** 在球面 $S^2$ 上，非零的克里斯托费尔符号为：
$$\Gamma^\theta_{\phi\phi} = -\sin\theta\cos\theta, \quad \Gamma^\phi_{\theta\phi} = \Gamma^\phi_{\phi\theta} = \cot\theta$$

### 7.2.4 平行移动

**定义 7.10（平行移动）** 设 $\gamma: [a, b] \to M$ 是光滑曲线，$X$ 是沿 $\gamma$ 的向量场。称 $X$ 是**沿 $\gamma$ 平行**的，如果：
$$\nabla_{\dot{\gamma}} X = 0$$

这等价于以下常微分方程组：
$$\frac{dX^k}{dt} + \Gamma^k_{ij} \dot{\gamma}^i X^j = 0$$

**定理 7.2** 给定曲线 $\gamma$ 和初始向量 $X_0 \in T_{\gamma(a)} M$，存在唯一的沿 $\gamma$ 平行的向量场 $X$ 满足 $X(a) = X_0$。

这定义了**平行移动**映射 $P_\gamma: T_{\gamma(a)} M \to T_{\gamma(b)} M$。

**定理 7.3** 列维-奇维塔联络下的平行移动保持内积不变：
$$g_{\gamma(b)}(P_\gamma(v), P_\gamma(w)) = g_{\gamma(a)}(v, w)$$

## 7.3 测地线和指数映射

### 7.3.1 测地线的定义

在欧几里得空间中，直线是"最直"的曲线，也是两点之间距离最短的曲线。在黎曼流形上，测地线推广了这一概念。

**定义 7.11（测地线）** 曲线 $\gamma: I \to M$ 称为**测地线**，如果其切向量沿自身平行：
$$\nabla_{\dot{\gamma}} \dot{\gamma} = 0$$

这称为**测地线方程**。

在坐标下，测地线方程为：
$$\ddot{\gamma}^k + \Gamma^k_{ij} \dot{\gamma}^i \dot{\gamma}^j = 0$$

这是一个二阶常微分方程组。

**定理 7.4（测地线的存在唯一性）** 对于任意 $p \in M$ 和 $v \in T_p M$，存在唯一的测地线 $\gamma_v: (-\epsilon, \epsilon) \to M$ 满足：
$$\gamma_v(0) = p, \quad \dot{\gamma}_v(0) = v$$

**定理 7.5（测地线的极值性质）** 测地线是局部距离最短的曲线。即对于测地线 $\gamma$ 上的充分接近的两点，$\gamma$ 是连接它们的最短曲线。

### 7.3.2 指数映射

**定义 7.12（指数映射）** 设 $p \in M$，定义**指数映射** $\exp_p: \mathcal{E}_p \subset T_p M \to M$ 为：
$$\exp_p(v) = \gamma_v(1)$$

其中 $\gamma_v$ 是以 $p$ 为起点、$v$ 为初始速度的测地线，$\mathcal{E}_p$ 是使得 $\gamma_v(1)$ 有定义的切向量的集合。

**定理 7.6** 指数映射 $\exp_p$ 在 $0 \in T_p M$ 的某个邻域上是良定义的微分同胚。

**定义 7.13（法坐标）** 设 $(e_1, \ldots, e_n)$ 是 $T_p M$ 的一组标准正交基。通过指数映射，可以在 $p$ 附近定义**法坐标**（Normal Coordinates）：
$$(x^1, \ldots, x^n) \mapsto \exp_p(x^i e_i)$$

在法坐标下，有以下简化性质：
- $g_{ij}(p) = \delta_{ij}$
- $\Gamma^k_{ij}(p) = 0$

### 7.3.3 测地线在机器人中的应用

测地线在机器人运动规划中具有重要应用：

**例 7.7（$SO(3)$ 上的测地线）** 在 $SO(3)$ 上装备生物度量，测地线对应于绕固定轴的旋转。对于 $R_0 \in SO(3)$ 和 $A \in \mathfrak{so}(3)$（李代数），测地线为：
$$\gamma(t) = R_0 \exp(tA)$$

其中 $\exp$ 是矩阵指数。

**例 7.8（构型空间中的运动规划）** 对于机械臂，构型空间 $T^n$ 上的测地线对应于各关节以恒定速度运动。这给出了关节空间中的"直线"运动。

**例 7.9（工作空间中的直线运动）** 如果希望末端执行器在工作空间中沿直线运动，需要在构型空间中规划相应的曲线，这通常不是测地线。

## 7.4 本章小结

本章介绍了黎曼几何的核心概念：

1. **黎曼度量**：为流形定义内积结构，允许测量长度、角度和体积
2. **列维-奇维塔联络**：唯一的与度量相容且无挠的联络，定义了协变导数和平行移动
3. **测地线**："最直"的曲线，局部距离最短
4. **指数映射**：将切空间映射到流形，在局部是微分同胚

这些概念是流形优化算法的理论基础，在机器人姿态规划、运动控制等领域有重要应用。

## 7.5 参考文献

1. do Carmo, M. P. (1992). *Riemannian Geometry*. Birkhäuser.
2. Petersen, P. (2016). *Riemannian Geometry* (3rd ed.). Springer.
3. Lee, J. M. (2018). *Introduction to Riemannian Manifolds* (2nd ed.). Springer.
4. Bullo, F., & Lewis, A. D. (2005). *Geometric Control of Mechanical Systems*. Springer.

## 7.6 练习题

1. 证明黎曼度量矩阵 $(g_{ij})$ 是对称正定的。

2. 在球面 $S^2$ 上，计算克里斯托费尔符号 $\Gamma^k_{ij}$。

3. 证明列维-奇维塔联络与度量相容等价于：
   $$\frac{\partial g_{ij}}{\partial x^k} = \Gamma^l_{ki} g_{lj} + \Gamma^l_{kj} g_{il}$$

4. 设 $\gamma$ 是测地线，证明 $\|\dot{\gamma}(t)\|$ 是常数（即测地线以恒定速度参数化）。

5. 在欧几里得空间 $\mathbb{R}^n$ 中，证明测地线就是直线。

6. 设 $(M, g)$ 是黎曼流形，$f: M \to \mathbb{R}$ 是光滑函数。定义**梯度** $\text{grad} f$ 为满足 $g(\text{grad} f, X) = X(f)$ 的向量场。在坐标下，证明：
   $$(\text{grad} f)^i = g^{ij} \frac{\partial f}{\partial x^j}$$

7. 在 $SO(3)$ 上，设 $R(t)$ 是一条曲线。证明 $R^T \dot{R} \in \mathfrak{so}(3)$。

8. 证明在法坐标下，$\Gamma^k_{ij}(p) = 0$。
