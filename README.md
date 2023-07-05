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
        - Multi-Point Trapezoidal
        - Cubic-Spline
        - PSO
        - Butterfly Optimization

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

#### 2.1.4 - Results
------
#### Here is the simulation of EE path for point-to-point (4-5-6-7 interpolating polynomial is used)
https://github.com/ArthasMenethil-A/Delta-Robot-Trajectory-Planning/blob/main/raw_images/4567.mp4


### 2.2 - Multi-Point Trajectory Planning
#### 2.2.1 - Higher Order Polynomials
#### 2.2.2 - Multi-Point Trapezoidal
#### 2.2.3 - Cubic-Spline
#### 2.2.4 - PSO
Here are some of my {researchs on PSO]()
#### 2.2.5 - Butterfly Optimization

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


