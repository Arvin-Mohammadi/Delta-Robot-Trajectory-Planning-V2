
#### 2.2.1 - Higher Order Polynomials
------
<ins>**Math**</ins>

Remember how in the 4-5-6-7 interpolating polynomial we used a 7th order polynomial to constraint the jerk, acceleration, velocity and position of two points? In theory we can do that with any number of points. say we have $n+1$ points to interlpolate, and we also have the following constraints:
- initial and final velocity equals zero
- initial and final acceleration equals zero
- initial and final jerk equals zero

this means we'll be having $n+7$ overall conditions ($n+1$ points and 6 above conditions) that can be interpolated through a $n+6$th-order polynomial. Let's solve an example for a 3-point polynomial (which will have us solving an 8th-order polynomial to solve) 

let's say we have the polynomial as: 

$$ q(\tau) = a_8\tau^8 + \dots + a_1\tau + a_0 $$ 

and the conditions are for the polynomial $q$ to hit $q_0$, $q_1$ and $q_2$ and have initial and final velocity, acceleration and jerk equal to zero. 

$$
\begin{cases}
    q(0) = q_0, \quad q(0.5) = q_1, \quad q(1) = q_2 \\ 
    \dot{q}(0) = 0, \quad \dot{q}(1) = 0 \\
    \ddot{q}(0) = 0, \quad \ddot{q}(1) = 0 \\
    \dddot{q}(0) = 0 , \quad \dddot{q}(1) = 0 
\end{cases}
$$

These conditions will give us 9 equations as well as 9 coefficients to calculate which make up a system of linear equations. the equation solution is uploaded in [this file](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/Research/Higher-order%20Polynomial/higher_order_poly_3pt.mat) and the final answers are:

$$
\begin{cases}
   a_0 = q_0 \\ 
   a_1 = 0 \\ 
   a_2 = 0 \\ 
   a_3 = 0 \\ 
   a_4 = 256q_1 - 163q_0 - 93q_2 \\ 
   a_5 = 596q_0 - 1024q_1 + 428q_2 \\
   a_6 = 1536q_1 - 838q_0 - 698q_2 \\
   a_7 = 532q_0 - 1024q_1 + 492q_2 \\
   a_8 = 256q_1 - 128q_0 - 128q_2
\end{cases}
$$


<ins>**Discussion**</ins>

Using this method isn't all that appreciated anyways because for the larger numder of points we'll have some problems. Here are some of those problems listed: 
* **Less Sensitivity to Data Perturbations:** High-degree polynomials are highly sensitive to changes in data points. Even small adjustments in the input data can significantly affect the resulting polynomial.
* **Avoiding Overfitting:** High-degree polynomials can lead to overfitting the data, capturing noise rather than the underlying trend.
* **Numerical Efficiency:** Solving systems of equations involving high-degree polynomials can be computationally expensive and may lead to numerical issues. In contrast, solving cubic splines is relatively efficient and numerically stable.
* **Local Control:** adding or removing a point in this method of high-order polynomial means recalculating the whole path instead of just one segment. 
