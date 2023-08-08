Improved Cubic Spline 
------
This the file explaining calculations of the improved versions of the cubic-spline method.

![antoine-dautry-05A-kdOH6Hw-unsplash](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/assets/69509720/c542bfd2-395a-4449-a1b9-7c200ccce1b5)
Photo by <a href="https://unsplash.com/@antoine1003?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Antoine Dautry</a> on <a href="https://unsplash.com/photos/05A-kdOH6Hw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a>

## First Method: Set Initial And Final Acceleration To Zero
------

## Second Method: Continuous Jerk Profile
------
Let's start from the very beginning. There are $n+1$ points given to us and we need $n$ polynomials of degree $p$ to interpolate them. To get a continuous jerk profile we shall have $p=5 for \quad i=0,\dots,n-1$

Where $i$ is the number of the polynomial.

$$ q(t) = a_0 + a_1t + a_2t^2 + a_3t^3 + a_4t^4 + a_5t^5 $$

So the overall function is given by: 

$$ s(t) = \lbrace q_k(t), \quad t\in [t_k, t_{k+1}], \quad k=0, \dots, n-1\rbrace $$

$$ q_k(t) = a_{k0} + a_{k1}(t-t_k) + a_{k2}(t-t_k)^2 + a_{k3}(t - t_k)^3 + a_{k4}(t - t_k)^4 + a_{k5}(t - t_k)^5 \quad for \quad k=0,\dots, n-1 $$

If we calculate the total number of coefficients we will reach: $6n$
So we will need that exact number of constraints. We consider the following conditions:

- $2n$ conditions for: Interpolation of the given positions at the start and finish of each segment
- $n-1$ conditions for: continuity of the velocity
- $n-1$ conditions for: continuity of the acceleration
- $n-1$ conditions for: continuity of the jerk
- $n-1$ conditions for: continuity of the jerk differential curve
- $4$ conditions for: initial and final velocity and acceleration set to zero

So the conditions will totally add up to $6n$ as needed. Re-writing the conditions in mathematic form we'll have: 

$$
\begin{cases}
  q_k(t_k) = q_k, \quad q_k(t_{k+1}) = q_{k+1}, &k=0, \dots, n-1 \\
  \dot{q_k}(t_{k+1}) = \dot{q_{k+1}}(t_{k+1}) = V_{k+1},  &k=0, \dots, n-2 \\
  \ddot{q_k}(t_{k+1}) = \ddot{q_{k+1}}(t_{k+1}) = A_{k+1}, &k=0, \dots, n-2 \\ 
  \dddot{q_k}(t_{k+1}) = \dddot{q_{k+1}}(t_{k+1}), &k=0, \dots, n-2 \\
  \ddot{\ddot{q_k}}(t_{k+1}) = \ddot{\ddot{q_{k+1}}}(t_{k+1}), &k=0, \dots, n-2 \\
  \dot{q_0}(t_0) = 0, \quad \dot{q_{n-1}}(t_n) = 0 & \\ 
  \ddot{q_0}(t_0) = 0, \quad \ddot{q_{n-1}}(t_n) = 0 & \\ 
\end{cases}
$$

The coefficients $a_{k, i}$ can be computed with the following alogrithm.

If the velocities $v_k, \quad k=1, \dots, n-1$, in the intermediate points were known for each cubic polynomial it would be possible to write

$$
\begin{cases}
  q_k(t_k) = a_{k0} &= q_k \\ 
  \dot{q_k}(t_k) = a_{k1} &= V_k \\
  \ddot{q_k}(t_k) = 2a_{k2} &= A_k \\ 
  q_k(t_{k+1}) = a_{k0} + a_{k1}T_k + a_{k2}T_k^2 + a_{k3}T_k^3 + a_{k4}T_k^4 + a_{k5}T_k^5 &= q_{k+1} \\ 
  \dot{q_k}(t_{k+1}) = a_{k1} + 2a_{k2}T_k + 3a_{k3}T_k^2 + 4a_{k4}T_k^3 + 5a_{k5}T_k^4 &= V_{k+1} \\ 
  \ddot{q_k}(t_{k+1}) = 2a_{k2} + 6a_{k3}T_k + 12a_{k4}T_k^2 + 20a_{k5}T_k^3 &= A_{k+1} \\ 
\end{cases}
$$

Now if we solve for $a_{k0}, a_{k1}, a_{k2}, a_{k3}, a_{k4}$ in MATLAB:

```
clear; clc;
syms a0 a1 a2 a3 a4 a5 T q_k v_k A_k q_kp1 v_kp1 A_kp1

eqn1 = a0 == q_k;
eqn2 = a1 == v_k;
eqn3 = 2*a2 == A_k;
eqn4 = a0 + a1*T + a2*T^2 + a3*T^3 + a4*T^4 + a5*T^5 == q_kp1;
eqn5 = a1 + 2*a2*T + 3*a3*T^2 + 4*a4*T^3 + 5*a5*T^4 == v_kp1;
eqn6 = 2*a2 + 6*a3*T + 12*a4*T^2 + 20*a5*T^3 == A_kp1;

[A, B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4, eqn5, eqn6], [a0, a1, a2, a3, a4, a5]) 

final_answer = simplify(linsolve(A, B))
```

$$
\begin{array}{l}
\left(\begin{array}{c}
q_k \\
v_k \\
\frac{A_k }{2}\\
-\frac{20\,q_k -20\,q_{\textrm{kp1}} +12\,T\,v_k +8\,T\,v_{\textrm{kp1}} +\sigma_1 -A_{\textrm{kp1}} \,T^2 }{2\,T^3 }\\
\frac{30\,q_k -30\,q_{\textrm{kp1}} +16\,T\,v_k +14\,T\,v_{\textrm{kp1}} +\sigma_1 -2\,A_{\textrm{kp1}} \,T^2 }{2\,T^4 }\\
-\frac{12\,q_k -12\,q_{\textrm{kp1}} +6\,T\,v_k +6\,T\,v_{\textrm{kp1}} +A_k \,T^2 -A_{\textrm{kp1}} \,T^2 }{2\,T^5 }
\end{array}\right)\\
\mathrm{}\\
\textrm{where}\\
\mathrm{}\\
\sigma_1 =3\,A_k \,T^2 
\end{array}
$$ 

Re-writing the final answer we'll reach the following: 

$$
\begin{cases}
  a_{k0} = q_k \\
  a_{k1} = v_k \\
  a_{k2} = 0.5A_k \\
  a_{k3} = -\frac{1}{2T_k^3} \left( 20(q_k - q_{k+1}) + (12v_k + 8v_{k+1})T_k + (3A_k - A_{k+1})T_k^2 \right) \\
  a_{k4} = \frac{1}{2T_k^2} \left( 30(q_k - q_{k+1}) + (16v_k + 14v_{k+1})T_k + (3A_k - 2A_{k+1})T_k^2 \right) \\
  a_{k5} =  -\frac{1}{2T_k^5} \left( 12(q_k - q_{k+1}) + 6(v_k + v_{k+1})T_k + (A_k - A_{k+1})T_k^2 \right) \\
\end{cases}
$$

Where $T_k = t_{k+1} - t_k$

On the other hand, velocities ($v_1, \dots, v_{n-1}$) and Accelerations ($A_1, \dots, A_{n-1}$) in intermediate points are not known, and therefore they must be computed. 

$$
\begin{cases}
  \dddot{q_k}(t_{k+1}) = 6a_{k3} + 24a_{k4}T_k + 60a_{k5}T_k^2 = 6a_{k+1,3} = \dddot{q_{k+1}}(t_{k+1}), &k=0, \dots, n-2 \\ 
  \ddot{\ddot{q_k}} = 24a_{k4} + 120a_{k5}T_k = 24a_{k+1,4} = \ddot{\ddot{q_{k+1}}} , &k=0, \dots, n-2 \\ 
\end{cases}
$$





