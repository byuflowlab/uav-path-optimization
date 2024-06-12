# UAV_Path_Optimization
This code was developed in conjunction with our paper 

Ingersoll, B., Ingersoll, K., DeFranco, P., and Ning, A., “UAV Path-Planning Using Bézier Curves and a Receding Horizon Approach,” AIAA Modeling and Simulation Technologies Conference, Washington, DC, Jun. 2016. doi:10.2514/6.2016-3675 

Given a flight domain of static and/or dynamic
obstacles, the method attempts to find a viable flight path while minimizing some criteria, such as path length, time elapsed, or energy use.

To use the path-planning algorithm, open main.m. Add the subfolders in \src and then run main.m. To preview possible obstacle fields, use test.m.

In main.m, there are several different algorithm options, such as which objective function will be used, how the gradients will be calculated, what type of obstacle field will be 
used, etc. Once the algorithm is complete, plots are generated which show the UAV's planned path.  
