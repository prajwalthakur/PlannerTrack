# Cubic Spline Interpolation

Cubic spline interpolation is a method used to approximate data by dividing the interpolation domain into intervals \([x_i, x_{i+1}]\) and fitting low-degree polynomials using nearby values. The key idea is to ensure that derivatives are continuous at the boundaries between intervals. Since cubic polynomials are used in each interval, this method is called **cubic spline interpolation** (quadratic splines also exist).

Given \(N+1\) data points:
\[
(x_0, y_0), (x_1, y_1), \dots, (x_N, y_N)
\]
let \(S_i(x)\) denote the interpolation function on the interval \([x_i, x_{i+1}]\). :contentReference[oaicite:0]{index=0}

---

## 1. Piecewise Polynomial

Each spline segment is defined as:
\[
S_i(x) = a_i (x - x_i)^3 + b_i (x - x_i)^2 + c_i (x - x_i) + d_i
\quad (i = 0, 1, \dots, N-1)
\]

To minimize the overall curvature, we impose the **natural spline condition**:
\[
S_0''(x_0) = S_{N-1}''(x_N) = 0
\]

Let:
\[
v_i = S''(x_i)
\]

---

## 2. Expression for \(b_i\)

From the second derivative:
\[
S_i''(x) = 6a_i(x - x_i) + 2b_i
\]

At \(x = x_i\):
\[
v_i = S_i''(x_i) = 2b_i \Rightarrow b_i = \frac{v_i}{2}
\]

---

## 3. Expression for \(a_i\)

Using continuity of second derivatives:
\[
v_{i+1} = S_i''(x_{i+1}) = 6a_i(x_{i+1} - x_i) + 2b_i
\]

Substituting \(b_i\):
\[
a_i = \frac{v_{i+1} - v_i}{6(x_{i+1} - x_i)}
\]

---

## 4. Expression for \(d_i\)

Since the spline passes through the data points:
\[
S_i(x_i) = y_i \Rightarrow d_i = y_i
\]

---

## 5. Expression for \(c_i\)

Using:
\[
S_i(x_{i+1}) = y_{i+1}
\]

After substitution:
\[
c_i =
\frac{y_{i+1} - y_i}{x_{i+1} - x_i}
- \frac{1}{6}(x_{i+1} - x_i)(2v_i + v_{i+1})
\]

---

## 6. System of Equations

From continuity of first derivatives:
\[
S_i'(x_{i+1}) = S_{i+1}'(x_{i+1})
\]

This leads to:
\[
(x_{i+1} - x_i)v_i + 2(x_{i+2} - x_i)v_{i+1} + (x_{i+2} - x_{i+1})v_{i+2}
=
6\left(
\frac{y_{i+2} - y_{i+1}}{x_{i+2} - x_{i+1}}
-
\frac{y_{i+1} - y_i}{x_{i+1} - x_i}
\right)
\]

With natural spline boundary conditions:
\[
v_0 = v_N = 0
\]

---

## 7. Matrix Form

The system can be written as:

\[
\begin{bmatrix}
2(h_0 + h_1) & h_1 & 0 & \cdots \\
h_1 & 2(h_1 + h_2) & h_2 & \cdots \\
0 & h_2 & 2(h_2 + h_3) & \cdots \\
\vdots & & & \ddots
\end{bmatrix}
\begin{bmatrix}
v_1 \\ v_2 \\ v_3 \\ \vdots \\ v_{N-1}
\end{bmatrix}
=
\begin{bmatrix}
w_1 \\ w_2 \\ w_3 \\ \vdots \\ w_{N-1}
\end{bmatrix}
\]

where:
\[
h_i = x_{i+1} - x_i
\]
\[
w_i = 6\left(
\frac{y_{i+1} - y_i}{h_i}
-
\frac{y_i - y_{i-1}}{h_{i-1}}
\right)
\]

---

## 8. Final Procedure

1. Solve the linear system to obtain \(v_i\)
2. Compute:
   - \(a_i\) from equation (5)
   - \(b_i\) from equation (4)
   - \(c_i\) from equation (7)
   - \(d_i\) from equation (6)
3. Use each \(S_i(x)\) to compute interpolated values in its interval

---

## Summary

Cubic spline interpolation constructs a smooth curve through all data points by:
- Using cubic polynomials in each interval
- Enforcing continuity of function, first derivative, and second derivative
- Applying natural boundary conditions

This results in a smooth and stable interpolation suitable for numerical and control applications.