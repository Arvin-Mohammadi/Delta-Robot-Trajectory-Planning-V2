# Improved Cubic Spline 
------
This the file explaining calculations of the improved versions of the cubic-spline method.

![antoine-dautry-05A-kdOH6Hw-unsplash](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/assets/69509720/c542bfd2-395a-4449-a1b9-7c200ccce1b5)
Photo by <a href="https://unsplash.com/@antoine1003?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Antoine Dautry</a> on <a href="https://unsplash.com/photos/05A-kdOH6Hw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a>

## First Method: Set Initial And Final Acceleration To Zero
------

## Second Method: Continuous Jerk Profile
------
Let's start from the very beginning. There are $n+1$ points given to us and we need $n$ polynomials of degree $p$ to interpolate them. To get a continuous jerk profile we shall have:

$$
\begin{cases}
  p=5 & for \quad i=0 \\
  p=4 & for \quad i=1, \dots, n-1
\end{cases}
$$

Where $i$ is the number of the polynomial.

$$ q(t) = a_0 + a_1t + a_2t^2 + a_3t^3 + a_4t^4 (+ a_5t^5) $$

So the overall function is given by: 

$$ s(t) = \lbrace q_k(t), \quad t\in [t_k, t_{k+1}], \quad k=0, \dots, n-1\rbrace $$

$$
\begin{cases}
  q_k(t) = a_{k0} + a_{k1}(t-t_k) + a_{k2}(t-t_k)^2 + a_{k3}(t - t_k)^3 + a_{k4}(t - t_k)^4 + a_{k5}(t - t_k)^5 & for \quad k=0 \\ 
  q_k(t) = a_{k0} + a_{k1}(t-t_k) + a_{k2}(t-t_k)^2 + a_{k3}(t - t_k)^3 + a_{k4}(t - t_k)^4 & for \quad k=1, \dots, n-1 \\ 
\end{cases}
$$

If we calculate the total number of coefficients we will reach: $5n+1$
So we will need that exact number of constraints. We consider the following conditions:

- $2n$ conditions for: Interpolation of the given positions at the start and finish of each segment
- $n-1$ conditions for: continuity of the velocity
- $n-1$ conditions for: continuity of the acceleration
- $n-1$ conditions for: continuity of the jerk
- $4$ conditions for: initial and final velocity and acceleration set to zero

So the conditions will totally add up to $5n+1$ as needed. Re-writing the conditions in mathematic form we'll have: 

$$
\begin{cases}
  q_k = t_k = q_k, \quad q_k(t_{k+1}) = q_{k+1}, &k=0, \dots, n-1 \\
  {\dot{q}}_k (t_{k+1}) &k=0, \dots, n-2
\end{cases}
$$


