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
  p=5 & for i=0 \\
  p=4 & for i=1, \dots, n-1
\end{cases}
$$

Where $i$ is the number of the polynomial.
