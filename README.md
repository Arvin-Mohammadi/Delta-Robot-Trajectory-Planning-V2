# A Full guide to Delta Parallel Robot Trajectory Planning
------
<ins>**A word from me:**</ins> I was really excited about creating a controller for DPR, and I must say, it was quite a challenging task. However, I became pretty obsessive about finding a solution and felt driven to share it with others as an open-source framework. So, I decided to fully immerse myself in this pursuit until I discovered a satisfying answer. I hope you'll find it helpful. I've actually worked on a similar repository in the past, but this new version is going to be even better and more comprehensive. I'm set to try out as many different methods as I can. the best way to reach me is via email: 
arvin1844m@gmail.com

![delta_robot_urdf](https://raw.githubusercontent.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/main/raw_images/delta_robot_urdf.PNG)

Overview: 
- Introduction
- Trajectory Planning 
    - point-to-point trajectory planning  
        - 3-4-5 interpolating polynomial
        - 4-5-6-7 interpolating polynomial
        - Trapezoidal method
    - multi-point trajectory planning
        - Higher Order Polynomials
        - Cubic-Spline
        - Improved Cubic-Spline 
        - Multi-Point Trapezoidal
        - PSO

## 1 - INTRODUCTION
------
#### Here I just tell you some history of the Delta Parallel Robots (DPRs), some of its applications and the edge it has over other rivals, the distadvantages, what ways have been used for its trajectory planning, etc. You know ... an introduction


<ins>**Some History:**</ins> Delta Parallel Robots (DPRs) are widely used in industrial
automation, offering significant enhancements in productivity
and work quality for a variety of complex tasks. With their
parallel kinematic structure and impressive speed capabilities,
they excel in applications that require precise and coordinated
robot movements. Since the advent of industrial robotics in the
1950s, significant advancements have led to the evolution of
four generations of industrial robots. Amongst them, DPRs
belong to the third generation. Remarkable capabilities of
DPRs for high-speed pick-and-place operations (PPO) make
them particularly well-suited for applications that demand
rapid and precise motion. Moreover, their parallel design
ensures excellent load-carrying capabilities and overall me-
chanical stability, thereby enhancing their overall performance
[1]. Understanding the characteristics of trajectory planning
for DPRs is crucial for their effective utilization, enabling
seamless robot motion. This article aims to explore the specific
aspects of trajectory planning for DPRs, with a focus on the
challenges of generating a smooth path for the End-Effector
(EE) while minimizing deviation throughout the trajectory.

</br>

<ins>**Main Types of Controlable Robots:**</ins> Industrial robots can be categorized into four main types
based on their control methods: remote controlled, sequence
controlled, controllable track, and adaptive control. While
remote-controlled robots are manually operated by an operator
and sequence-controlled robots follow predefined programs,
controllable track robots execute tasks along predetermined
paths with adjustable parameters, offering flexibility in specific
applications. Adaptive control empowers robots to respond
to real-time feedback and adapt their actions accordingly,
enabling dynamic and adaptive behavior in complex envi-
ronments. This article focuses on studying the third type of
controller, specifically controllable track robots [2].

</br>

<ins>**Some Algorithms:**</ins> The trajectory-planning problem of a DPR can be tackled
using various algorithms. Some notable approaches include the
Identify applicable funding agency here. If none, delete this.
utilization of 5th and 7th order polynomials, which provide
mathematical representations for smooth and continuous tra-
jectories. Similarly, 4th, 6th, and 7th order B-Spline algorithms
offer a flexible and precise representation of robot trajectories,
allowing for efficient path planning. Lame’s Curve, another
method employed in trajectory planning, offers a parameter-
ized curve that ensures smooth and continuous motion profiles
while accommodating dynamic constraints [3].

</br>

<ins>**Some More Algorithms:**</ins> In addition to these methods, Pythagorean-Hodograph
Curves are worth mentioning as they provide a specialized
class of curves that simplify the computation of differential
properties, such as velocity and acceleration, for smoother and
more efficient motion planning [4]. Variations of the Particle
Swarm Optimization (PSO) algorithm have also been applied
to trajectory planning for DPRs. The PSO algorithm, inspired
by social behavior, optimizes trajectories by iteratively adjust-
ing particle positions in a search space to find optimal solutions
[5]. Furthermore, variations of the butterfly optimization algo-
rithm have been explored, which is a metaheuristic algorithm
inspired by the foraging behavior of butterflies. [6]. Lastly,
the Trapezoidal Algorithm, a well-known method, divides
the trajectory into multiple segments with constant velocity
profiles. This approach ensures smooth transitions between
different segments while satisfying dynamic constraints, of-
fering a simple yet effective solution for trajectory planning
[7].

</br>

<ins>**Importance of This Article-ish Page:**</ins> Efficient trajectory planning is a crucial aspect in maxi-
mizing the performance and productivity of Delta robots in
industrial automation. However, the generation of a smooth
and accurate path for the end-effector while minimizing devi-
ation throughout the trajectory poses a significant challenge.
One of the key difficulties encountered in this process is the
constraint of bounding the jerk, which refers to the rate of
change of acceleration. Achieving smooth and jerk-limited
trajectories is essential to ensure precise and coordinated robot
movements, preventing abrupt changes in velocity that can
lead to mechanical stress and instability. Therefore, addressing
the challenge of creating a good trajectory with controlled
jerk profiles becomes paramount for the effective utilization
of Delta robots in various complex tasks, ultimately enhancing
productivity and work quality in industrial automation.

</br>

<ins>**Goal And Overview:**</ins> The primary goal of this research is to investigate and
implement various trajectory planning algorithms for Delta
robots, including the multi-point and single-point trapezoidal
method, cubic spline, and higher-order polynomials such as
the 5th and 7th order. By exploring these different algorithms,
we aim to assess their effectiveness in generating smooth
and accurate trajectories while considering the constraint of
bounded jerk. Furthermore, we seek to contribute to the
existing body of knowledge by focusing on the utilization of
the multi-point trapezoidal method, which has received limited
attention in the context of Delta robots. The importance of this
research lies in two key aspects. Firstly, the investigation of
the multi-point trapezoidal method for Delta robots presents a
novel contribution to the field, potentially offering improved
trajectory planning capabilities. Secondly, the comprehensive
comparison of multiple trajectory planning algorithms pro-
vides valuable insights into their relative performance and
assists in identifying the most suitable approaches for Delta
robot applications.

</br>

<ins>**Organization of Paper:**</ins> To achieve these research objectives, this paper is organized
as follows: we first begin by presenting a detailed description
and implementation of each trajectory planning algorithm.
Subsequently, we evaluate and compare their performance
using simulated scenarios. Based on the results, we select
the most promising algorithms and proceed to validate their
effectiveness through experimental tests on a real Delta robot.
The experimental results provide practical insights into the
applicability and performance of these algorithms, thereby
enhancing our understanding of trajectory planning for Delta
robots in industrial automation settings.

The code to Inverse and Forward Kinematics can be found in the [Delta Robot file](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/python/delta_robot.py).

## 2 - TRAJECTORY PLANNING
------
#### Here are the different methods of controlling the trajectory of the robot


<ins>**What is Trajectory Planning:**</ins> Trajectory planning is a crucial step in achieving the desired
movement for a manipulator control system, ensuring the
generation of reference inputs that conform to the given
geometric path and comply with the kinematic and dynamic
constraints of the manipulator [8]. Trajectory planning plays
a vital role in optimizing the performance and efficiency of
Delta robots in industrial automation. By generating smooth
and accurate paths for robot movements, trajectory planning
algorithms ensure precise and coordinated motion from the
starting point to the target locations. The goal is to determine
a trajectory for the joints or the EE, comprising a sequence
of position, velocity, and acceleration values. The trajectory
planning algorithm generates a time-based sequence of values,
respecting the imposed constraints, to specify the position and
orientation of the EE.

</br>

<ins>**What Trajectory Plannings We Look Into:**</ins> This section focuses on two key aspects of trajectory plan-
ning: point-to-point and multi-point movements. For point-to-
point trajectory planning, we explore three distinct algorithms:
the 5th order polynomial, the 7th order polynomial, and the
trapezoidal algorithm. These algorithms offer different math-
ematical representations and methodologies for generating
smooth paths. Additionally, we delve into multi-point trajec-
tory planning and investigate the adaptation of the trapezoidal
algorithm, the utilization of Pythagorean-Hodograph Curves,
and the application of the Particle Swarm Optimization (PSO)
algorithm. By examining these trajectory planning approaches,
we aim to provide a comprehensive understanding of their
mathematical foundations, implementation considerations, and
their effectiveness in optimizing the motion of Delta robots in
industrial automation settings.

### 2.1 - Point-to-Point Trajectory Planning
------
#### This entire sub-section is for when you want to move EE from point A to B


<ins>**What is Point-to-Point Trajectory Planning:**</ins> Point-to-Point Trajectory Planning refers to the process of
generating smooth and coordinated paths for delta robots that
involve moving from a starting point to a single target location.
It focuses on determining the optimal trajectory that ensures
precise and controlled movements of the robot’s EE or joints.
By considering the geometric path, kinematic and dynamic
constraints, and other factors, point-to-point trajectory plan-
ning algorithms enable the seamless execution of movements
from one specific location to another, enhancing the overall
performance and accuracy of delta robots in industrial automa-
tion settings

#### 2.1.1 - 3-4-5 Interpolating Polynomial
------
#### This one has a problem of unbounded jerk


<ins>**Math:**</ins>
When interpolating between given initial and final values of the joint variable $\theta^I$ and $\theta^F$ respectively, the following can be employed:

$$\theta(t) = \theta^I + (\theta^F - \theta^I)s(\tau)$$

Here, $\tau$ represents the normalized time, where $\tau = \frac{t}{T} and $T$ denotes the overall time period. The function $s(\tau)$ is a fifth-order polynomial defined as: 

$$s(\tau) = a\tau^5 + b\tau^4 + c\tau^3 + d\tau^2 + e\tau + f$$

In this context, it is important to note that $s(\tau)$ lies within the range of 0 to 1, and $\tau$ ranges from 0 to 1 as well. 

To establish desired constraints on the generated path, initial and final positions, velocities, and accelerations can be set. By applying the following conditions:

$$s(0) = 0, s^\prime(0) = 0, s^{\prime\prime}(0) = 0$$

$$s(1) = 1, s^\prime(1)=0, s^{\prime\prime}(1)=0$$

a system of six equations with six unknowns can be solved. The resulting values are:

$$a = 6, b = -15, c = 10, d = 0, e = 0, f = 0$$

Thus, the polynomial takes the form: 

$$s(\tau) = 6\tau^5 - 15\tau^4 + 10\tau^3$$

This representation allows for smooth and controlled joint variable interpolation, satisfying the prescribed constraints [9].

![345 method](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/raw_images/345%20method.png)

<ins>**Discussion:**</ins>
The utilization of a fifth-order polynomial, such as the 3-4-5 interpolating polynomial, enables the generation of smooth and continuous trajectories for Delta robots. By incorporating higher-order terms, this polynomial minimizes abrupt changes in position, velocity, and acceleration during the interpolation process. While the 3-4-5 interpolating polynomial offers desirable smoothness and continuity, it is important to note its limitations. One significant drawback is the lack of explicit constraints on jerk, which refers to the rate of change of acceleration. The absence of jerk constraints can result in undesirable mechanical stress and instability, particularly at the start and end points of the trajectory where jerk values may be unbounded. As a result, caution must be exercised when applying the 3-4-5 interpolating polynomial in DPR applications, as uncontrolled jerk may affect the overall performance and quality of robot operations.

The code can be found in the [path planning file](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/python/path_planning_ptp.py) in the function `point_to_point_345`

#### 2.1.2 - 4-5-6-7 Interpolating Polynomial
------
#### For point to point this is a good one


<ins>**Math:**</ins> 
If we consider $\theta^I$ and $\theta^F$ to be the given initial and final values of the joint variable, and w ewant to interpolate the values in between, the 4-5-6-7 interpolating polynomial can be employed. The formula below represents the interpolation: 

$$\theta(t) = \theta^I + (\theta^F - \theta^I)s(\tau)$$

In this formula, $\tau$ represents the normalized time $(\tau = \frac{t}{T})$, where $T$ is the overall time period), and $s(\tau)$ is a fourth-order polynomial defined as: 

$$s(\tau) = a\tau^7 + b\tau^6 + c\tau^5 + d\tau^4 + e\tau^3 + f\tau^2 + g\tau + h$$

The constraints for the path generated using this method include setting the initial and final position, velocity, acceleration, and jerk. By incorporating the following conditions:


$$s(0) = 0, s^\prime(0) = 0, s^{\prime\prime}(0) = 0, s^{\prime\prime\prime}(0) = 0$$

$$s(1) = 1, s^\prime(1)=0, s^{\prime\prime}(1)=0, s^{\prime\prime\prime}(1)=0$$

By solving this system of eight equations with eight unknowns, we can determine the values of the coefficients:

$$a=-20, b = 70, c = -84, d = 35, e = 0, f = 0, g = 0, h=0$$

As a result, the polynomial will take the form [9]: 

$$s(\tau) = -20\tau^7 + 70\tau^6 - 84\tau^5 + 35\tau^4$$

The result of this method is shown in the figure below. As explained, the advantage of this method compared to 3-4-5 interpolating polynomial is that the jerk is bounded at the initial and final points.

![4567 method](https://raw.githubusercontent.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/main/raw_images/4567%20method.png)

<ins>**Discussion:**</ins> 
The 4-5-6-7 interpolating polynomial offers an improvement over the 3-4-5 interpolating polynomial by incorporating higher-order terms. This enables a more precise representation of the desired trajectory, leading to enhanced accuracy and control. Additionally, the inclusion of jerk constraints in the interpolation process ensures smoother robot movements, reducing mechanical stress and instability. From all the point-to-point methods, this one proves to be the most reliable.

The code can be found in the [path planning file](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/python/path_planning_ptp.py) in the function `point_to_point_4567`

#### 2.1.3 - Trapezoidal method
------
#### In concept this isn't a bad method, but if you want to use it in practice i'd recommend S-curve which is an improved version of this


<ins>**Math:**</ins>:
Like the previous methods, the goal here is to basically use a trapezoidal diagram as a way to interpolate the velocity profile between the values of $\theta^I$ and $\theta^F$. In this instance we call them $p$ and $p_0$. The trapezoidal diagram is defined as the following formula:

$$
\dot{p} = v = 
\begin{cases}
  at & t_0 \leq t < t_1 \\
  V_{max} & t_1 \leq t < t_2  \\
  -at & t_2 \leq t \leq t_3
\end{cases}
$$

For the sake of simplicity we say that $t_0 = 0, t_1 = T/3, t_2 = 2T/3, t_3 = T$. Here's the result: 

$$
\ddot{p} = a = 
\begin{cases}
    a & 0 \leq t < \frac{1}{3} T \\
    0 & \frac{2}{3} T \leq t < \frac{1}{3} T \\
    -a & \frac{2}{3} T \leq t < T \\
\end{cases} 
\quad \quad 
\dot{p} = v = 
\begin{cases} 
    at & 0 \leq t < \frac{1}{3} T \\
    V_{max} & \frac{2}{3} T \leq t < \frac{1}{3} T \\
    -at & \frac{2}{3} T \leq t < T \\
\end{cases}
$$

Since $v_{max}$ is given to us as a limitation of our DPR, we use that value to calculate acceleration. 


$$v_{max} = a.t_{t=\frac{1}{3} T} \rightarrow a = \frac{3v_{max}}{T}$$

Implementing this sequence with a Python script, we can get the results show as below [10]: 

![trapezoidal_ptp](https://raw.githubusercontent.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/main/raw_images/trapezoidal_ptp.png)

<ins>**Discussion:**</ins>:
In this method We've taken the maximum difference in $\theta^I$ and $\theta^F$ since there are three motors, and generated a path according to that maximum amount of movement (meaning we set the overall time period according to the longest path that is to be taken). As for the other actuators, we have two options, either minimize the time or use the same time period. According to our need both options are good enough, the diagram is the result of the longest path. This method however, has the same problem of 3-4-5 method, but instead of start and finishing point, the problem is at $T/3$ and $2T/3$.

The code can be found in the [path planning file](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/python/path_planning_ptp.py) in the function `trapezoidal_ptp`


### 2.2 - Multi-Point Trajectory Planning
------
#### This entire sub-section is for when you want to move EE through points P1 to Pn

Multi-Point Trajectory Planning involves generating smooth and coordinated paths for Delta Parallel Robots (DPRs) that include multiple target locations. Unlike point-to-point trajectory planning, which focuses on movements between a single starting point and target location, multi-point trajectory planning considers the optimization of trajectories with multiple intermediate points. This type of trajectory planning is particularly useful for applications where DPRs need to navigate complex paths or perform tasks that require precise motion through multiple waypoints. The goal of multi-point trajectory planning is to determine an optimal trajectory that passes through the specified intermediate points while ensuring smooth and accurate motion of the robot's End-Effector (EE) or joints. This process involves considering various factors, such as geometric path constraints, kinematic and dynamic limitations, and any additional requirements specific to the application.

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

and the conditions are: 
- $ q(0) = q_0 $
- $ q(0.5) = q_1 $
- $ q(1) = q_2 $
- $ \dot{q} (0) = 0 $
- $ \dot{q} (1) = 0 $
- $ \ddot{q} (0) = 0 $
- $ \ddot{q} (1) = 0 $
- $ \dddot{q} (0) = 0 $
- $ \dddot{q} (1) = 0 $

<ins>**Discussion**</ins>

Using this method isn't all that appreciated anyways because for the larger numder of points we'll have some problems. Here are some of those problems listed: 
* **Less Sensitivity to Data Perturbations:** High-degree polynomials are highly sensitive to changes in data points. Even small adjustments in the input data can significantly affect the resulting polynomial.
* **Avoiding Overfitting:** High-degree polynomials can lead to overfitting the data, capturing noise rather than the underlying trend.
* **Numerical Efficiency:** Solving systems of equations involving high-degree polynomials can be computationally expensive and may lead to numerical issues. In contrast, solving cubic splines is relatively efficient and numerically stable.
* **Local Control:** adding or removing a point in this method of high-order polynomial means recalculating the whole path instead of just one segment. 

#### 2.2.2 - Cubic-Spline
------
#### taken from ref[10] chapter 4.4 | It would be a good choice if it weren't for the jerk at the initial and final points, how does one solve that problem? 

When provided with $n+1$ points, it is feasible to construct a unique interpolating polynomial of degree $n$. However, as the number of points increases, the computational burden becomes heavier. To address this, an alternative approach is to utilize n polynomials of degree $p$ instead. The selection of $p$ is based on the desired level of continuity for the spline. For instance, if one aims to achieve continuity of velocities and accelerations at the time instances $t_k$, where the transition between two consecutive segments takes place, a cubic polynomial with degree $p=3$ can be assumed.

$$q(t) = a_0 + a_1t + a_2t^2 + a_3t^3$$

The overall function is given by 

$$s(t)    = \lbrace q_k(t), t\in [t_k, t_{k+1}], k=0, ..., n-1 \rbrace $$

$$ q_k(t) = a_{k0} + a_{k1}t + a_{k2}t^2 + a_{k3}t^3$$

By adopting this approach, it becomes essential to calculate four coefficients for each polynomial. Given that $n$ polynomials are required to define a trajectory using $n+1$ points, the total number of coefficients to be determined amounts to $4n$. To address this challenge, the following conditions need to be taken into account. Adding up the conditions that we have, $2n$ conditions for following the points, $n -1$ conditions for the continuity of velocities, and $n-1$ conditions for the continuity of accelerations, it can be seen that there are $4n-2$ conditions, in comparison to the $4n$ constants. The two degrees of freedom can be used for extra conditions such as: 

* The initial and final velocities
* The initial and final accelerations
* The periodic conditions for velocity and acceleration
* The continuity of jerk 

#### We assume assigned initial and final velocities

For given the points of $(t_k, q_k)$ for $k=0, ..., n$ we'll want to calculate: 
$$s(t) = \lbrace q_k(t), t\in[t_k, t_{k+1}], k=0, ..., n-1 \rbrace$$
$$q_k(t) = a_{k0} + a_{k1}(t-t_k) + a_{k2}(t-t_k)^2 + a_{k3}(t-t_k)^3$$

The conditions will be: 

$$
\begin{cases}
    q_k(t_k) = q_k, \quad q_k(t_{k+1}) = q_{k+1}, & k=0, ..., n-1 \\
    \dot{q_k} (t_{k+1}) = \dot{q_{k+1}}(t_{k+1})=v_{k+1}, & k=0, ..., n-2\\
    \ddot{q_k} (t_{k+1}) = \ddot{q_{k+1}} (t_{k+1}), & k=0, ..., n-2\\
    \dot{q_0} (t_0) = v_0, \quad \dot{q_{n-1}} (t_n) = v_n & \\
\end{cases} 
$$

#### solution 

The coefficient $a_{k,i}$ can be computed with the following steps: 

if we consider each velocity at time $t_k$ to be known:

$$
\begin{cases}
    q_k(t_k) = a_{k0}, & = q_k \\
    \dot{q_k} (t_k) = a_{k1}, & = v_k \\ 
    q_k(t_{k+1}) = a_{k0} + a_{k1} T_k + a_{k2} T^2_k + a_{k3}T^3_k, & = q_{k+1} \\ 
    \dot{q_k} (t_{k+1}) = a_{k1} + 2a_{k2} T_k + 3 a_{k3} T^2_k, & = v_{k+1} \\ 
\end{cases}
$$

Where $T_k = t_{k+1} - t_k$. Solving the above equations we have: 

$$
\begin{cases}
    a_{k,0} = q_k\\ 
    a_{k,1} = v_k\\ 
    a_{k,2} = \frac{1}{T_k}   [\frac{3(q_{k+1} - q_k)}{T_k} - 2v_k - v_{k+1}] \\ 
    a_{k,3} = \frac{1}{T^2_k} [\frac{2(q_k - q_{k+1})}{T_k} + v_k + v_{k+1}] \\
\end{cases}
$$

But this is for when the velocities of the points are known, which they are not (except the initial and final points). So the velocities have to be calculated, in this instance we use the continuity conditions of acceleration: 

Velocities can be found with a matrix of $v = A^{-1}c$. Where: 

$$
A = 
\begin{bmatrix}
    2(T_0+T_1)     & T_0         & 0       & ...                             &     & 0 \\
    T_2            & 2(T_1+T_2)  & T_1     & 0                               &     & \vdots \\
    0              &             & \ddots  &                                 &     & 0 \\
    \vdots         &             &         & T_{n-2}  & 2(T_{n-3}+T_{n-2})   & T_{n-3} \\ 
    0              & \dots       &         & 0        & T_{n-1}              & 2(T_{n-2} + T_{n-1}) \\  
\end{bmatrix}
$$

$$
c = 
\begin{bmatrix}
    \frac{3}{T_0T_1} \left[ T^2_0(q_2 - q_1) + T^2_1(q_1 - q_0) \right] - T_1 v_0 \\
    \frac{3}{T_1T_2} \left[ T^2_1(q_3 - q_2) + T^2_2(q_2 - q_1) \right] \\
    \vdots \\ 
    \frac{3}{T_{n-3}T_{n-2}} \left[ T^2_{n-3}(q_{n-1} - q_{n-2}) + T^2_{n-2}(q_{n-2} - q_{n-3}) \right] \\
    \frac{3}{T_{n-2}T_{n-1}} \left[ T^2_{n-2}(q_n - q_{n-1}) + T^2_{n-1}(q_{n-1} - q_{n-2}) \right] - T_{n-2}v_n \\
\end{bmatrix}
$$

$$
v = 
\begin{bmatrix}
    v_1 \\ 
    v_2 \\ 
    \vdots \\ 
    v_{n-2} \\ 
    v_{n-1} \\ 
\end{bmatrix}
$$

Thus we have the velocities and the problem is solved. (for more details go to reference [10] Chapter 4.4). The implementation of this problem is coded in [this python file](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/python/path_planning_mltp.py) and the results are: 

![Cubic spline method](https://raw.githubusercontent.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/main/raw_images/cubic%20spline%20method.png)

#### note from me

There was a problem with this algorithm in my previous repository that I'm going to explain now. My previous employer defined the problem to be something like: we want to go through a lot of different points in space meaning we want to <ins> be precise while being fast </ins>. not taking into account that this problem definition is not valid. this is a trade-off. you either get high speed and low precision, or you get high precision and low speed. if you want both high speed and high precision there will be an unnaturally high jerk at the start. what he was telling me to do was to define a circle with like 1000 points and then make the interpolation between 1000 points. but that's not the true goal of the robot is it? (unless you're making a painter robot or a 3D printer in which case the process is gonna be slow so that the jerk won't cause a problem) 

the goal is to hit certain points. like {(x1, y1, z1), (x2, y2, z2), ... (x10, y10, z10)} something like that. because in each operation we want our robot to move towards the target, pick the target up, move the target, place the target, get back into first position. that's what? 5 or 6 points to hit? we don't need 1000 constraints.

#### 2.2.3 - Improved Cubic spline 
------
#### This idea came to me from the ref [5]

**first method: intial and final zero acceleration**

for more control over the start and finishing points we can use 4-th order polynomial for the start and finishing points. so instead of using n polynomials with an order of 3, we'll use n-2 polynomials with order of 3 and two polynmials with an order of 4. the first and final polynomials so to speak. This will give us two more constants hence, we can apply two more constraints. 

**second method: smooth acceleration curve**

Another way of imporvemnt is to use p=4 altogether. this requires that the calculations be re-done in a similar manner to the previous section. of course one of the polynomials have to be p=5. either first or last polynomial so to speak. because the number of constrains we want for $n+1$ points are: 
- $2n$ constraints for the positions
- $n-1$ constraints for the velocity continuity
- $n-1$ constraints for the acceleration continuity
- $n-1$ constraints for the jerk continuity
- $2$ constraints for initial and final velocity
- $2$ constraints for initial and final acceleration

that adds up to $5n+1$ constants. and since if we have n polynomials with the order of 4, we're going to get $5n$ constants. in order to add a constant, we can just put one of the polynomials a 5-th order. 

#### 2.2.4 - Particle Swarm Optimization (PSO)
------

Here are some of my [researchs on PSO](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/Research/PSO/PSO.README.md)

#### 2.2.5 - Multi-Point Trapezoidal
------

#### the idea for this is from ref [10] - Part 3.2.4

![Trapezoidal through a sequence of points](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/assets/69509720/725a87c9-7c0f-4fbd-94d2-5bf5696a4dca)

We talked about the trapezoidal method in one of the point-to-point methods, but now we want to use it as a multi-point method. we already know about the 3 phases in trapezoidal. Assume that we want to use point-to-point interpolation on multiple points. What's the problem with that? it's the fact that the end effector will stop at all of the points that we want to hit. meaning if we define our points as $P_0, ..., P_n$, and we use point-to-point trajectory planning to go from $P_0$ to $P_1$ and from $P_1$ to $P_2$ and so on and so forth, the end effector will stop at each of the points (in some cases that might be what we want to do but in most cases that highly inefficient). but for now let's implement this for point-to-point trapezoidal. our first goal is to implement something like the figure (a) from the two diagrams about (Reference for picture is ref [10] - Part 3.2.4) - Hence the trapezoidal will reduce to a trianlge for us to hit the max velocity and then immediately enter the deceleration phase. the calculation will look like: 

$$
\ddot{p} = a(t) = 
\begin{cases}
  a & 0 \leq t < T/2 \\
  -a & T/2 \leq t \leq T
\end{cases}
$$


$$
\dot{p} = v(t) = 
\begin{cases}
  at & 0 \leq t < T/2 \\
  -at & T/2 \leq t \leq T
\end{cases}
$$

$$
p(t) = 
\begin{cases}
  a\frac{t^2}{2} + p_0 & 0 \leq t < T/2 \\
  \left[v_{max} \frac{t}{2} - 0.5v(t)(T-t)\right] + p_0 & T/2 \leq t \leq T
\end{cases}
$$

But since $v_{max} = \frac{T}{2}a$ and $p_{final} = v_{max}\times \frac{T}{2} + p_0$ and we're given the values for $a, p_{fina}, p_0$ with these two equations values for $T$ and $v_{max}$ can be found

$$
\begin{cases}
  T = \sqrt{(p_{final} - p_0)\frac{4}{a}} \\
  v_{max} = a \frac{T}{2}
\end{cases}
$$

I wish it was as easy as this though. since we have three motors we have to synchronize them first and then we can generate the velocity profile.

------

**Motor Synchronization**




### 2.3 - Results
------ 
3D Animation for results of the sampled data of generated trajectories.

#### 2.3.1 - 4-5-6-7 interpolating polynomial 

https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/assets/69509720/ec3256be-25d6-479c-86f5-d363633df996

#### 2.3.2 - cubic spline results

https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/assets/69509720/b941c628-c7fa-4237-bbe9-e208a2699d5c

## 3 - SIMULATION
------
The platform I want to use for simulating the DPR is ROS. Of course as always there are a couple of challenges along the way. So let's deal with them one by one. Here's [my research](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/Research/ROS/README.md) about this topic. Since simulation with ROS seems really difficult because delta robot is a parallel robot, we're going to switch to simulink. But first, let's calculate the safe workspace.

### 3.1 - Safe Workspace
------
In the inverse kinematics, if you go beyond a certain angle in each of the motors, then the robot is going to be morphed and broken, so we can't go beyond those points. Whether or not the the robot can't go to a certain point, is determined through a calculation in the [delta robot inverse kinematics python file](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/python/delta_robot.py). if the given point isn't reachable then we get the error "non existing point". So the reachable workspace will be calculated with the use of that error in [this file](https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/python/workspace.py)

![reachable wokrspace](https://raw.githubusercontent.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/main/raw_images/reachable_workspace.png)

As can be seen from the generated picture, this is not the workspace we want to calculate, since it includes dangerous points in 3D space for the robot to be in. joint limits define the range within which a robot's joints can safely move without causing damage or compromising its operation. These limits are typically set based on the mechanical design and capabilities of the robot, taking into account factors such as joint structure, motor capabilities, and the intended range of motion. Seemingly the criteria isn't simply "if the robot can reach that point or not". 


## References: 
------
[1] doi: /10.1007/978-3-030-03538-9 23 </br>
[2] doi: 10.32629/jai.v5i1.505 </br>
[3] doi: 10.1109/CRC.2017.38 </br>
[4] doi: 10.3390/app9214491 </br>
[5] doi: 10.1007/s00170-019-04421-7 </br>
[6] doi: 10.3390/app12168145 </br>
[7]Research of Trajectory Planning for Delta Parallel Robots, 2013 International Conference on Mechatronic Sciences, Electric Engineering and Computer (MEC) </br>
[8] doi: 10.1007/s11786-012-0123-8 </br>
[9] Fundamentals of Robotic Mechanical Systems Theory, Methods, and Algorithms, Fourth Edition by Jorge Angeles </br>
[10] Trajectory Planning for Automatic Machines and Robots by Luigi Biagiotti, Claudio Melchiorri


