# A Full guide to Delta Parallel Robot Trajectory Planning

Delta Parallel Robots (DPRs) are widely used in industrial
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
Industrial robots can be categorized into four main types
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
controller, specifically controllable track robots. [2]
The trajectory-planning problem of a DPR can be tackled
using various algorithms. Some notable approaches include the
Identify applicable funding agency here. If none, delete this.
utilization of 5th and 7th order polynomials, which provide
mathematical representations for smooth and continuous tra-
jectories. Similarly, 4th, 6th, and 7th order B-Spline algorithms
offer a flexible and precise representation of robot trajectories,
allowing for efficient path planning. Lame’s Curve, another
method employed in trajectory planning, offers a parameter-
ized curve that ensures smooth and continuous motion profiles
while accommodating dynamic constraints. [3]
In addition to these methods, Pythagorean-Hodograph
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
Efficient trajectory planning is a crucial aspect in maxi-
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
The primary goal of this research is to investigate and
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
To achieve these research objectives, this paper is organized
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
II. TRAJECTORY PLANNING
Trajectory planning is a crucial step in achieving the desired
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
This section focuses on two key aspects of trajectory plan-
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
A. Point-to-Point Trajectory Planning
Point-to-Point Trajectory Planning refers to the process of
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


References: 
[1] doi: /10.1007/978-3-030-03538-9 23
[2] doi: 10.32629/jai.v5i1.505
[3] doi: 10.1109/CRC.2017.38
[4] doi: 10.3390/app9214491
[5] doi: 10.1007/s00170-019-04421-7
[6] doi: 10.3390/app12168145
[7] Research of Trajectory Planning for Delta Parallel Robots, 2013 International Confere
[8] doi: 10.1007/s11786-012-0123-8

