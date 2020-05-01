# Review of Modern Control Theory

## Chapter 1. State Variable Models

1. 状态空间模型表示为

$$
\begin{split}

\dot{x} &= Ax+Bu \\  

 y &= Cx+Du

\end{split}
$$

2. 求x(t)的响应

$$
\begin{align}
x(t)=\Phi(t)x(0)+\int_0^t{\Phi(t-\tau)Bu(\tau)}  \tag{1}
\end{align}
$$

其中$\Phi(t)=\mathcal{L}^{-1}[(sI-A)^{-1}]=e^{At}$，因此对于零输入响应，也即$u(t)=0$，此时$x(t)=\Phi(t)x(0)$。

3. 可以利用梅森公式将传递函数形式转换成状态空间模型的标准型。

$$
G(s)=\frac{\sum\Delta_k P_k}{\Delta},其中\Delta=1-\sum L_n+\sum{L_m L_q}-\sum{L_rL_sL_t}
$$

$\Delta_k$为1-对应的前向通路不相交的loop。

| 类型           | 节点数 | 特点                            |
| -------------- | ------ | ------------------------------- |
| 相变量标准型   | n+3    | 分子以输出结束                  |
| 输入前馈标准型 | 2n+2   | 分子从输入开始，x单独占一个节点 |



## ==Chapter 2.The Design of State Variable Feedback Systems==

1. 可控可观性

- 若矩阵$P_c=[B \quad AB\quad A^2B \quad\cdots\quad A^{n-1}B]$满足$rank(P_c)=n$,也即$P_c$行列式不为零,则系统可控.

- 若矩阵C$P_o=[C\quad CA\quad \cdots\quad CA^{n-1}]$满足$rank(P_o)=n$,也即$P_o$行列式不为零,则系统可观.

2. 控制器、观测器设计`公式的推导以及框图需要复习`

   - ==ackerman公式== ：$K=[0,0,\cdots,1]P_c^{-1}p(A),L=p(A)P_o^{-1}[0,0,\cdots,1]^T$
   - 或者设K、L矩阵，带入$det[\lambda I-(A-BK)],det[\lambda I-(A-LC)]$后求出关于$\lambda$的多项式，与所要配置的零极点的特征多项式进行对比，确定K和L矩阵。

3. 最优控制

   - Using equation $H^T P+PH=-Q$,where $H=A-BK$.And $Q=I+\lambda K^TK$.
   - Solve the equation,we can get matrix P that is described by element in matrix K.
   - Calculate $J=x(0)^TPx(0)$ and  get K by solveing equation$\frac{\partial J}{\partial K}=0$
   - Get $J_{min}$ and get $u(t)=-Kx$
   - ==Most important==，after we finish four steps above,we need to verify the stability of system which use the k and $J_{min}$.

4. Internal model design(个人理解的内膜设计是让输出跟踪参考输入)

   - Let $e(t)=y(t)-r(t)$,then according to r(t),we can assume $z=x^{(n)},w=u^{(n)}$,同时有
     $$
     \begin{align}
     \left( \begin{array}{c}
     	\overset{\cdot}{e}\\
     	\overset{\cdot \cdot}{e}\\
     	\overset{\cdot}{z}\\
     \end{array} \right) =A\left( \begin{array}{c}
     	e\\
     	\overset{\cdot}{e}\\
     	z\\
     \end{array} \right) +Bw
     \end{align}
     $$
     If it's controllable，we can use ackerman formula to get K martix,then w(t)=-K$A\left( \begin{array}{c}
     	e\\
     	\overset{\cdot}{e}\\
     	z\\
     \end{array} \right) $,then we can get u(t).

## ==Chapter 3.Digital Control Systems==

1. 零阶采样器
   - $G_0(s)=\frac{1-e^{-sT}}{s}$ ,在求系统传递函数中,通常将s乘到后面进行z变换,分子z变换为$1-z^{-1}$
2. ==z变换==  `需要自己推导一遍常见函数的拉普拉斯和Z变换`
3. ==z逆变换== 
   - 留数法:`是求z^{n-1}Z(z)的留数`,故需要讨论分子是否存在变成极点的情况.
   - 长除法 分子除以分母,得到的$z^{-n}$的系数为$y(n)$的值.

4. `根据流程框图求传递函数D(z)`.
   - 自己复习卢京潮课本后总结一遍.
5. Closed-loop feedback sampled-Data systems
   - 判定定理:采样系统所有极点位于z平面的单位圆内,则系统稳定.
   - 对于二阶系统,C.E.满足条件:$|q(0)|<1,q(1)>0,q(-1)>0$,则系统稳定.
   - 对于二阶系统,还可以通过$z=\frac{w+1}{w-1}$变换成p(w),根据劳斯判据判定稳定性.
   - July判据,`卢京潮课本`
6. 稳态误差的计算
   - 利用终值定理:$e(\infin)=\underset{z\rightarrow 1}{\lim}(z-1)E(z)$,where $E(z)=Z[e^*(t)]$.
7. 控制器设计
   - `复习自控的超前滞后校正器`
   - 参数确定:通过得到的连续校正器获得离散的校正器$G_c(s)=K\frac{s+a}{s+b}$得到$D(z)=C\frac{z-A}{z-B}$,其中$A=e^{-aT},B=e^{-bT}$,通过s=0,z=1计算C的值:$C\frac{1-A}{1-B}=K\frac{a}{b}$.
8. 数字控制系统的根轨迹
   - 一切与连续系统根轨迹绘制方法相同.
   - 稳定区域为单位圆内,故临界增益为与单位圆的交点.
     1. 利用参数方程求出交点:$|z|=1,z=e^{j\theta}=cos\theta+jsin\theta$.
     2. 基于双线性变换:$z=\frac{jv+1}{jv-1}$代入根轨迹方程(与z域单位圆的交点等价于s平面的虚轴,故u=0),令实部虚部分别为0解出v,k.满足$v\in R,k\geqslant 0 $.

##  Chapter 4.非线性控制系统分析

- 基本的几种非线性类型
- 相平面法`只适用于二阶系统`
  - 令$\overset{\cdot}x,\overset{\cdot\cdot}x=0$,得到的解$(x_e,0)$为奇点.奇点处的切线斜率不定(故相轨线相交于奇点)
  - $\alpha=\frac{f(\overset{\cdot}x,x)}{\overset{\cdot}x}$,同时还有$\frac{d\overset{\cdot}x}{dx}=\frac{f(\overset{\cdot}x,x)}{\overset{\cdot}x}$,其中$\alpha$为曲线在该点的方向向量. `存在特解情况,方向向量和斜率相同`
  - 在相平面上半部分,方向向量指向倾向右侧,下半部分则倾向左侧.
  - 二阶系统的相轨迹分类(不知道要不要记)
- 描述函数法
  - 应用条件:非线性环节为奇函数且G(s)具有良好的低通性.
  - 计算$N(A)$
    1. 输入为$x(t)=Asinwt$时,对输出$y(t)$进行傅里叶变换并取一次谐波.
    2. 计算$A_0,A_1,B_1$,由于$y(t)$为奇函数,故$A_0=0$,$A_1=\frac{1}{\pi}\int^{2\pi}_{0}y(t)coswtd(wt)=0$,$B_1=\frac{1}{\pi}\int^{2\pi}_{0}y(t)sinwtd(wt)$.
    3. $y(t)=B_1 sinwt$,故$N(A)\triangleq \frac{B_1}{A}$.
  - 稳定性分析
    1. 闭环系统特征方程为$1+N(A)G(s)=0$,得到$G(jw)=-\frac{1}{N(A)}$.
    2. 根据推广的Nyquist稳定判据,得到
       - 当$G(jw)$不包围$-\frac{1}{N(A)}$时,非线性闭环系统稳定.
       - 当$G(jw)$包围$-\frac{1}{N(A)}$时,非线性闭环系统不稳定.
       - 当$G(jw)$与$-\frac{1}{N(A)}$相交时,非线性闭环系统存在周期运动,且方程为$x(t)=A_xsin(w_xt)$`曲线从不稳定区域到稳定区域与G(jw)的交点产生稳定的周期运动`
  - 非线性系统的等效简化
    1. 写出系统的闭环传递函数$\Phi(s)$
    2. 得到特征方程$1+N(A)G(s)=0$
    3. 从特征方程中解出$G(s)$,即为等效的开环线性部分传递函数.

##  Chapter 5. Lyapunov稳定性分析

1. Lyapunov稳定性判据

- 第一法对于线性定常系统$\overset{\cdot}x=Ax,x(0)=0$.

  - 系统每一平衡状态是Lyapunov意义下稳定的充要条件是:A所有特征根均具有非正实部,且零实部的特征值为A的特征多项式的单根.
  - 系统唯一平衡状态$x_e=0$是渐近稳定的充要条件是:A所有的特征值均具有负实部.

- 第二法 对于系统
  $$
  \begin{align}
  	\overset{\cdot}x=&f(x),x\in R^n,t\geq 0 \\
  	f(0)=&0
  \end{align}
  $$
  

  - 若存在具有一阶连续偏导数的标量函数$V(x),V(0)=0$.

    1. $V(x)$正定,$\overset{\cdot}V(x)$负定$\Rightarrow $零解是渐进稳定的.若V(x)径向无界,则系统零解是大范围渐进稳定,径向无界是指满足当$\lVert x \rVert\rightarrow \infin,V(x)\rightarrow\infin$.
    2. $V(X)$正定,$\overset{\cdot}V(X)$半负定,$\overset{\cdot}V(x)\equiv 0\Rightarrow x \equiv 0$,则零解是渐进稳定的,若V(x)径向无界,则系统零解是大范围渐进稳定.
    3. $V(X)$正定,$\overset{\cdot}V(X)$半负定,$\overset{\cdot}V(x)$在$x\ne 0$时恒等于0,则零解是稳定的.
    4. $V(X)$和$\overset{\cdot}V(X)$正定,零解不稳定.

    - 若零解不在原点处,则可以通过坐标变换使其在原点,从而使用上述定理进行判断.
    - 二次型函数V(x)正定:$\Delta_k>0$,负定:$(-1)^k\Delta_k>0,k=1,2,\cdots,n$.半正(负)定即为正(负)定判断准则中将$>$改为$\geq$.

2. 线性定常连续系统的稳定性
   - 对于任意的正定对称矩阵Q,方程$A^TP+PA=-Q$存在正定对称矩阵解P.`一般取Q为单位阵`
3. 线性定常离散系统的稳定性(貌似不考)
   - 对于任意的正定对称矩阵Q,方程$G^TPG-P=-Q$存在正定对称矩阵解P.`一般取Q为单位阵`



