# Particle Swarm Optimization (PSO)
------
This readme file is dedicated to represent some of my research on PSO. The end game is to apply PSO in trajectory planning of the delta robot for a time optimal solution. 

## 1 - INTRODUCTION
------
<ins>**Question: What is PSO?**</ins> 

PSO is one of the bio-inspired algorithms. PSO was proposed by Kennedy and Eberhart in 1995. They believed that a school of fish or a flock of birds that moves in a group “can profit from the experience of all other members”. We can imagine each bird is to help us find the optimal solution in a high-dimensional solution space and the best solution found by the flock is the best solution in the space.

<ins> **Example**</ins> 

Assume a function that has two independant variables such as $F(x, y) = z$. Our task is to find the minimum value of z. The way to use PSO in this instance is to randomly initialize a set of particles such as: ${(x_1, y_1), ..., (x_n, y_n)}$. We think of each point as a **bird** or a **particle** and we let them search in the space for the minimum value. After a few iterations we take the minimum value that is found by all the particles to be the global minimum of the function. [1]

## 2 - PATH PLANNING
------

## References: 
------
[1] [A Gentle Introduction to PSO](https://machinelearningmastery.com/a-gentle-introduction-to-particle-swarm-optimization/) </br>
