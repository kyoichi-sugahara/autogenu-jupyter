# 離散時間システムの最適制御

## ラグランジュ関数の定義

各時刻における等式拘束条件$f(x(k), u(k), k) - x(k+1) = 0$ に対するラグランジュ乗数のベクトルを$\lambda(k+1) \in \mathbb{R}^n$として、ラグランジュ関数を以下のように定義する。

$\begin{aligned}
\mathcal{L}(x,u,\lambda) &= \varphi(x(N)) + \sum_{k=0}^{N-1} \left( H(x(k), u(k), \lambda(k+1), k) - \lambda(k+1)^\top x(k+1) \right)
\end{aligned}$

ここで、$H(x,u,\lambda,k) = L(x,u,k) + \lambda^\top (f(x,u,k))$はスカラー値関数であり、$x(k)$は状態ベクトル、$u(k)$は制御入力、$\lambda(k+1)$はラグランジュ乗数のベクトル、$f(x(k), u(k), k)$は状態方程式、$N$は最終時刻を表します。また、$\varphi(x(N))$は終端コスト、$L(x(k), u(k),k)$はステージコストを示しています。

## ラグランジュ関数の変形

ハミルトニアン関数は以下のように定義します。
$$H(x,u,\lambda,t) = L(x,u,t) + \lambda^\top (f(x,u,t))$$

ラグランジュ関数の範囲を$\sum_{k=1}^{N-1}$に変更すると、$k=0$の項を明示的に追加する必要があります。この場合、ラグランジュ関数$\mathcal{L}(x,u,\lambda)$は以下のように書き換えることができます。

$\begin{aligned}
\mathcal{L}(x,u,\lambda) &= \varphi(x(N)) + H(x(0), u(0), \lambda(1), 0) - \lambda(N)^\top x(N) + \sum_{k=1}^{N-1} \left( H(x(k), u(k), \lambda(k+1), k) - \lambda(k)^\top x(k) \right)
\end{aligned}$

## 停留条件

状態の微小変化${ dx(k) }_{k=0}^N$と入力の微小変化${ du(k) }_{k=0}^{N-1}$を考慮したラグランジュ関数$\mathcal{L}(x,u,\lambda)$の微小変化は以下のように導出できます。

$ dL(x,u, \lambda) = d\varphi(x(N)) + dH(0) - \lambda(N)^T dx(N) + \sum_{k=1}^{N-1}(dH(k) - \lambda(k)^T dx(k)) $

ここで、各項について展開し、$dx(k)$および$du(k)$を共通の項として持つ場合は結合していきます。
まず、$d\varphi(x(N))$および$dH(k)$について1階のテイラー展開を行います。

$\begin{aligned}
\frac{\partial \varphi(x(N))}{\partial H(k)} = \frac{\partial x(N)}{\partial \varphi} dx(N) = \frac{\partial x(k)}{\partial H(k)} dx(k) + \frac{\partial u(k)}{\partial H(k)} du(k)
\end{aligned}$

これらの式を$d\mathcal{L}(x,u,\lambda)$に代入し、シグマの中で$dx(k)$および$du(k)$を共通の項として持つ場合は結合し、$dx(N)$、$dx(0)$、および$du(0)$を共通の項として持つ場合は結合していきます。

$\begin{aligned}
dL(x,u,\lambda) = \frac{\partial x(N)}{\partial \varphi} dx(N) + \frac{\partial x(0)}{\partial H(0)} dx(0) + \frac{\partial u(0)}{\partial H(0)} du(0) - \lambda(N)^\top dx(N) + \sum_{k=1}^{N-1} \left( \frac{\partial x(k)}{\partial H(k)} dx(k) + \frac{\partial u(k)}{\partial H(k)} du(k) - \lambda(k)^\top dx(k) \right)
\end{aligned}$

ラグランジュ関数$\mathcal{L}(x,u,\lambda)$の微小変化$d\mathcal{L}(x,u,\lambda)$から停留条件を導出するために、それぞれの変数に関する偏導関数が0になる条件を求めます。

以上の条件と元の拘束条件$f(x(k), u(k), k) - x(k+1) = 0$を合わせて、停留条件は以下のようになります。

$\frac{\partial H(k)}{\partial x(k)} - \lambda(k)^\top = 0$ for $k=1,\dots,N-1$
$\frac{\partial H(k)}{\partial u(k)} = 0$ for $k=0,\dots,N-1$
$\frac{\partial \varphi}{\partial x(N)} - \lambda(N)^\top = 0$
$f(x(k), u(k), k) - x(k+1) = 0$ for $k=0,\dots,N-1$

## 最適性条件の導出

### 離散時間2点境界値問題の導出

評価区間を$N$ステップに離散化し、時間刻みを$\Delta \tau = \frac{T}{N}$と定義します。評価区間上$i$番目の時間ステップにおける状態を${x_i^*(t)}$、入力を${u_i^*(t)}$、ラグランジュ乗数を${\lambda_i^*(t)}$と表します。

このとき、停留条件$f(x(k), u(k), k) - x(k+1) = 0$を差分近似することによって得られる離散時間2点境界値問題は以下のようになります。

$x_{i+1}^*(t) = x_i^*(t) + \Delta \tau f(x_i^*(t), u_i^*(t), i)$
また、$\frac{\partial H(k)}{\partial x(k)} - \lambda(k)^\top = 0$を差分近似することによって得られる離散時間2点境界値問題は以下のようになります。

$\lambda_{i}^*(t) = \lambda_{i+1}^*(t) + \Delta \tau \frac{\partial H(i)}{\partial x_i^*(t)}$ for $i=1,\dots,N-1$

$x_{0}^*(t)=x(0)$
$\lambda_N^*(t) = \frac{\partial \varphi}{\partial x(N)}^\top$
$\frac{\partial H(k)}{\partial u(k)} = 0$

状態遷移方程式: $x_{i+1}^* = x_i^* + \Delta t f(x_i^*, u_i^*, i)$ （$i=0,\dots,N-1$）
逆方向のアジョイント方程式: $\lambda_i^*(t) = \frac{\partial J}{\partial x_i}(x_i^*,u_i^*,i) + \frac{\partial H}{\partial x_i}(x_i^*,u_i^*,\lambda_{i+1}^*,i)$ （$i=N-1,\dots,0$）
境界条件: $x_0^* = x_0$ および $\lambda_N^*= \frac{\partial \varphi}{\partial x_N}(x_N^*)$
最適性条件: $\frac{\partial H}{\partial u_k}(x_k^*,u_k^*,\lambda_{k+1}^*,k) = 0$ （$k=0,\dots,N-1$）

ここで、$x_i^*$ および $u_i^*$ は最適解、$\lambda_i^**$ はアジョイント変数、$\Delta t$ は離散時間ステップサイズ、$f$ は状態方程式、$H$ はラグランジュ関数、$J$ は目的関数、$\varphi$ は終端コスト、$x_0$ は初期状態を表します。逆方向のアジョイント方程式では、$\frac{\partial J}{\partial x_i}$ は目的関数 $J$ を $x_i$ で偏微分したものを表します。

未知の制御入力の系列${u_i^*}_{k=0}^{N-1}$からなるベクトル$U(t)$を以下のように定義する。
$$
U = \begin{bmatrix} u_0^* \\ u_1^*\\ \vdots \\ u_{N-1}^* \end{bmatrix}
$$
状態の系列${x_i^*}_{k=0}^{N}$が$x(t)$と$U(t)$および$t$によってきまることが分かる。

そして、それらに依存して、随伴係数の系列${\lambda_i^*(t)}$が決まる。

離散時間2点境界値問題の最適性条件は以下のようになります。
$$
F(U(t),x(t),t)=\begin{bmatrix}
\frac{\partial H(x_0^*, u_0^*, \lambda_1^*, 0)}{\partial u_0} & \\
\frac{\partial H(x_1^*, u_1^*, \lambda_2^*, 1)}{\partial u_1} & \\
\vdots & \\
\frac{\partial H(x_{N-1}^*, u_{N-1}^*, \lambda_N^*, N-1)}{\partial u_{N-1}} &
\end{bmatrix}=0
$$

$\frac{\partial F}{\partial U} \dot{U}-ζF-\frac{\partial F}{\partial x} \dot{x}$

ここで、$\frac{dx}{dt} = f(t,x,u)$を用いたことに注意してください。

次に、$\frac{\partial F}{\partial x}$を$N \times 4$行列$A$、$\frac{\partial F}{\partial U}$を$N \times N$行列$B$として、$B\dot{U} = - \zeta F - A f(t,x,u)$とします。両辺に$B^{-1}$を左から掛けると、$\dot{U} = - B^{-1} \zeta F - B^{-1} A f(t,x,u)$となります。

したがって、$\frac{\partial F}{\partial U} \dot{U} - \zeta F - \frac{\partial F}{\partial x} \dot{x} = B \dot{U} - \zeta F + A f(t,x,u)$を解くと、

$\dot{U} = -B^{-1} \zeta F - B^{-1} A f(t,x,u)$


## 問題設定

状態ベクトルは以下のように定義します。
$$ x = \begin{bmatrix} y \\ \theta \\ \dot{y} \\ \dot{\theta} \end{bmatrix} .$$
状態方程式を以下のように定義します。
$$\dot{x} = f(t, x, u) = \begin{bmatrix} \dot{y} \\ \dot{\theta} \\ \frac{1}{m_c + m_p \sin ^2{\theta}} \left\{ u + m_p \sin{\theta} (l \dot{\theta}^2 + g \cos{\theta}) \right\} \\ \frac{1}{l(m_c + m_p \sin ^2{\theta})} \left\{ - u \cos{\theta} - m_p l {\dot{\theta}}^2 \cos{\theta} \sin{\theta} - (m_c + m_p) g \sin{\theta} \right\} \end{bmatrix} .$$
目的関数は以下のように定義します。
$$ J = \varphi(t, x) + \int_{t}^{t+T} L(t, x, u) .$$
目標状態は以下のように定義します。
$$x_{\rm ref} := \begin{bmatrix} 0 \\ \pi \\ 0 \\ 0 \end{bmatrix}.$$
終端コストは以下のように定義します。
$$\varphi(t,x) = \frac{1}{2} (x - x_{\rm ref})^{\rm T} Q_{\rm terminal} (x - x_{\rm ref}) ,$$
ステージコストは以下のように定義します。
$$L(t, x, u) = \frac{1}{2} (x - x_{\rm ref})^{\rm T} Q (x - x_{\rm ref}) + \frac{1}{2} r u^2,$$

ここで、使用された記号は以下の通りです。

$y$: カートの水平位置（m）
$\theta$: 振子の角度（rad）
$\dot{y}$: カートの水平速度（m/s）
$\dot{\theta}$: 振子の角速度（rad/s）
$m_c$: カートの質量（kg）
$m_p$: 振子の質量（kg）
$l$: 振子の長さ（m）
$g$: 重力加速度（m/s^2）
$u$: カートに与える力（N）
$Q_{\rm terminal}$: 終端コストの重み行列
$Q$: ステージコストの重み行列
$r$: 入力に対するコストの重み
