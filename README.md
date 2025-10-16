**Overview**

This repository simulates a robot moving around a landmark in a 2D world, which is a 3x3 grid. 
For simplicity, the robot can only move in the x,y planes. The 2 main classes are the ekf and
matrix classes. To implement, the main.cpp file calls the "simulate()" ekf class method.

To run simply use a compiler like g++:

g++ main.cpp matrix/matrix.cpp ekf/ekf.cpp -o myprogram
