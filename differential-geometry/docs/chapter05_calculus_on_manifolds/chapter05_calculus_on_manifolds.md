# 第5章 流形上的微积分

## 本章导读

流形上的微积分将欧几里得空间中的微积分概念推广到流形上，是微分几何的核心内容。本章介绍向量场的李导数、微分形式、外微分和积分等概念。通过本章学习，你将掌握流形上的微积分工具，理解斯托克斯定理的深刻含义，为学习黎曼几何奠定基础。

---

## 5.1 李导数与李括号

### 5.1.1 向量场的流

定义 5.1（向量场的流） 设 $X$ 是光滑向量场。$X$ 的**流**（flow）是映射 $\phi: \mathbb{R} \times M \to M$，满足：
1. $\phi(0, p) = p$
2. $\phi(t, \phi(s, p)) = \phi(t+s, p)$
3. $\frac{d}{dt}\phi(t, p) = X_{\phi(t, p)}$

记 $\phi_t(p) = \phi(t, p)$。

### 5.1.2 李括号

定义 5.2（李括号） 设 $X, Y$ 是光滑向量场。**李括号**定义为：
$$[X, Y]_p(f) = X_p(Yf) - Y_p(Xf)$$

其中 $f \in C^\infty(M)$。

**局部坐标表示**

在坐标 $(x^1, \ldots, x^n)$ 中，设 $X = X^i \partial_i$，$Y = Y^j \partial_j$：
$$[X, Y] = \left(X^i \frac{\partial Y^j}{\partial x^i} - Y^i \frac{\partial X^j}{\partial x^i}\right) \frac{\partial}{\partial x^j}$$

**李括号的性质**

定理 5.1
1. 双线性：$[aX + bY, Z] = a[X, Z] + b[Y, Z]$
2. 反对称：$[X, Y] = -[Y, X]$
3. Jacobi恒等式：$[X, [Y, Z]] + [Y, [Z, X]] + [Z, [X, Y]] = 0$

### 5.1.3 李导数

定义 5.3（李导数） 函数 $f$ 沿向量场 $X$ 的**李导数**：
$$\mathcal{L}_X f = Xf$$

向量场 $Y$ 沿 $X$ 的李导数：
$$\mathcal{L}_X Y = [X, Y]$$

**几何意义**

李导数描述沿向量场 $X$ 的流的变化率。

---

## 5.2 微分形式

### 5.2.1 外代数

**外积（楔积）**

定义 5.4（楔积） 设 $\alpha$ 是 $k$-形式，$\beta$ 是 $l$-形式。**楔积** $\alpha \wedge \beta$ 是 $(k+l)$-形式：
$$(\alpha \wedge \beta)(v_1, \ldots, v_{k+l}) = \frac{1}{k!l!} \sum_{\sigma \in S_{k+l}} \text{sgn}(\sigma) \alpha(v_{\sigma(1)}, \ldots, v_{\sigma(k)}) \beta(v_{\sigma(k+1)}, \ldots, v_{\sigma(k+l)})$$

**性质**

定理 5.2
1. 结合律：$(\alpha \wedge \beta) \wedge \gamma = \alpha \wedge (\beta \wedge \gamma)$
2. 分次交换律：$\alpha \wedge \beta = (-1)^{kl} \beta \wedge \alpha$

### 5.2.2 微分形式的外微分

定义 5.5（外微分） **外微分** $d: \Omega^k(M) \to \Omega^{k+1}(M)$ 满足：
1. $d(\alpha + \beta) = d\alpha + d\beta$
2. $d(\alpha \wedge \beta) = d\alpha \wedge \beta + (-1)^k \alpha \wedge d\beta$
3. $d(df) = 0$ 对任意函数 $f$

**局部坐标表示**

对于 $k$-形式 $\omega = \omega_{i_1 \cdots i_k} dx^{i_1} \wedge \cdots \wedge dx^{i_k}$：
$$d\omega = \frac{\partial \omega_{i_1 \cdots i_k}}{\partial x^j} dx^j \wedge dx^{i_1} \wedge \cdots \wedge dx^{i_k}$$

**重要性质**

定理 5.3（$d^2 = 0$）
$$d(d\omega) = 0$$

### 5.2.3 闭形式与恰当形式

定义 5.6（闭形式） $k$-形式 $\omega$ 称为**闭的**（closed），如果 $d\omega = 0$。

定义 5.7（恰当形式） $k$-形式 $\omega$ 称为**恰当的**（exact），如果存在 $(k-1)$-形式 $\eta$ 使得 $\omega = d\eta$。

**关系**

定理 5.4 恰当形式必是闭形式：$\omega = d\eta \Rightarrow d\omega = 0$。

逆命题一般不成立，与流形的拓扑有关（de Rham上同调）。

---

## 5.3 积分与斯托克斯定理

### 5.3.1 流形的定向

定义 5.8（定向） 流形 $M$ 的**定向**是图册使得所有坐标转换的Jacobian行列式为正。

**体积形式**

定义 5.9（体积形式） $n$ 维定向流形上的**体积形式**是处处非零的 $n$-形式。

### 5.3.2 微分形式的积分

**在坐标卡上的积分**

设 $\omega$ 是 $n$-形式，在坐标卡 $(U, \varphi)$ 中，$\varphi = (x^1, \ldots, x^n)$：
$$\omega = f(x) dx^1 \wedge \cdots \wedge dx^n$$

定义：
$$\int_U \omega = \int_{\varphi(U)} f(x) dx^1 \cdots dx^n$$

**整体积分**

用单位分解将局部积分拼接成整体积分。

### 5.3.3 斯托克斯定理

**带边流形**

定义 5.10（带边流形） **带边流形**是局部类似于上半空间 $\mathbb{H}^n = \{(x^1, \ldots, x^n) : x^n \geq 0\}$ 的拓扑空间。

边界 $\partial M$ 是 $(n-1)$ 维流形。

**斯托克斯定理**

定理 5.5（斯托克斯定理） 设 $M$ 是 $n$ 维紧致定向带边流形，$\omega$ 是 $(n-1)$-形式：
$$\int_M d\omega = \int_{\partial M} \omega$$

**经典特例**

1. **微积分基本定理**：$\int_a^b df = f(b) - f(a)$
2. **格林公式**：$\iint_D \left(\frac{\partial Q}{\partial x} - \frac{\partial P}{\partial y}\right) dxdy = \oint_{\partial D} Pdx + Qdy$
3. **高斯公式**：$\iiint_V \nabla \cdot \mathbf{F} dV = \iint_{\partial V} \mathbf{F} \cdot d\mathbf{S}$
4. **斯托克斯公式**：$\iint_S (\nabla \times \mathbf{F}) \cdot d\mathbf{S} = \oint_{\partial S} \mathbf{F} \cdot d\mathbf{r}$

---

## 5.4 黎曼流形初步

### 5.4.1 黎曼度量

定义 5.11（黎曼度量） **黎曼度量** $g$ 是在每点 $p \in M$ 指定内积 $g_p: T_p M \times T_p M \to \mathbb{R}$ 的光滑赋值。

**局部表示**

在坐标 $(x^1, \ldots, x^n)$ 中：
$$g = g_{ij} dx^i \otimes dx^j$$

其中 $g_{ij} = g\left(\frac{\partial}{\partial x^i}, \frac{\partial}{\partial x^j}\right)$ 是度量张量的分量。

**诱导范数**

向量 $v \in T_p M$ 的长度：
$$\|v\| = \sqrt{g(v, v)} = \sqrt{g_{ij} v^i v^j}$$

### 5.4.2 体积形式

定义 5.12（黎曼体积形式） 定向黎曼流形 $(M, g)$ 上的**体积形式**：
$$dV = \sqrt{\det(g_{ij})} dx^1 \wedge \cdots \wedge dx^n$$

**体积计算**

区域 $U \subseteq M$ 的体积：
$$\text{Vol}(U) = \int_U dV = \int_{\varphi(U)} \sqrt{\det(g_{ij})} dx^1 \cdots dx^n$$

### 5.4.3 梯度、散度与拉普拉斯

**梯度**

定义 5.13（梯度） 函数 $f$ 的**梯度**是向量场 $\text{grad } f$ 满足：
$$g(\text{grad } f, v) = df(v) = vf$$

局部表示：
$$(\text{grad } f)^i = g^{ij} \frac{\partial f}{\partial x^j}$$

**散度**

定义 5.14（散度） 向量场 $X$ 的**散度**：
$$\text{div } X = \frac{1}{\sqrt{\det g}} \frac{\partial}{\partial x^i}(\sqrt{\det g} X^i)$$

**拉普拉斯算子**

定义 5.15（拉普拉斯） **拉普拉斯-贝尔特拉米算子**：
$$\Delta f = \text{div}(\text{grad } f) = \frac{1}{\sqrt{\det g}} \frac{\partial}{\partial x^i}\left(\sqrt{\det g} g^{ij} \frac{\partial f}{\partial x^j}\right)$$

---

## 5.5 本章小结

本章介绍了流形上的微积分：

1. **李导数与李括号**：
   - 向量场的流
   - 李括号：$[X, Y]$
   - Jacobi恒等式

2. **微分形式**：
   - 楔积与外代数
   - 外微分：$d$
   - 闭形式与恰当形式

3. **积分与斯托克斯定理**：
   - 流形的定向
   - 微分形式的积分
   - 斯托克斯定理：$\int_M d\omega = \int_{\partial M} \omega$

4. **黎曼流形初步**：
   - 黎曼度量
   - 体积形式
   - 梯度、散度、拉普拉斯

这些工具为研究流形的几何性质和分析问题提供了强大方法。

---

## 习题

### 基础题

5.1 证明李括号的Jacobi恒等式。

5.2 计算 $d(xdy \wedge dz + ydz \wedge dx + zdx \wedge dy)$。

5.3 验证 $d^2 = 0$ 对1-形式成立。

5.4 在极坐标下写出 $\mathbb{R}^2$ 的体积形式。

### 提高题

5.5 证明恰当形式必是闭形式。

5.6 用斯托克斯定理推导格林公式。

5.7 计算球面 $S^2$ 上标准度量的体积。

5.8 推导球面坐标下拉普拉斯算子的表达式。

### 编程题

5.9 实现微分形式的外微分计算。

5.10 计算给定黎曼度量的体积形式。

---

## 参考文献

1. Lee J M. Introduction to Smooth Manifolds[M]. Springer, 2013.
2. Spivak M. Calculus on Manifolds[M]. Addison-Wesley, 1965.
3. do Carmo M P. Riemannian Geometry[M]. Birkhäuser, 1992.
4. 陈省身, 陈维桓. 微分几何讲义[M]. 北京大学出版社, 2001.
