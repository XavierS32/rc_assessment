# MeArm 运动学正逆解推导

## 建系方式

## Forward Kinematics

## Inverse Kinematics

抽出……
$$
\left\{
\begin{aligned}
x &= c_1(u+x_2)-s_1y_2
y &= s_1(u+x_2)+c_1y_2
\end{aligned}
\right.
$$
此时，可以研究一般情况
$$
\left\{
\begin{aligned}
x &= ac-bs
\tag{1}
y &= as+bc
\tag{2}
\end{aligned}
\right.
$$
这个方程可以写成矩阵的形式
$$
\begin{bmatrix}
x\\
y
\end{bmatrix}
=
\begin{bmatrix}
c_1 & -s_1\\
s_1 & c_1
\end{bmatrix}
\begin{bmatrix}
a\\
b
\end{bmatrix}
$$
由此，可以从几何意义的角度分析：

// ..........

$$
d = \sqrt{(u-l_4)^2+v^2}\\
\varphi = \mathrm{atan2}{v, u-l_4}\\
\varphi = atan2{v, u-l_4}\\
\cos{\pi-\theta2-\theta3}=\frac{l2_2^2+l_3^2-d^2}{2l_2l_3}=A
$$