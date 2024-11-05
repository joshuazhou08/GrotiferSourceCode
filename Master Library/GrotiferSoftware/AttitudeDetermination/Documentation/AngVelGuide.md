# Guide 
## To include in a project
Add the folder AngVelLib to your project.

### Include statement:
> #include AngVelLibAngVelClass.hpp

This also has the backward solution class in it. This one include statement covers Eigen, Backward Solution, Forward Solution, and Angular Velocity.
<br> </br>

You only need to include this header as the rest of the dependences are included by it.

## Function
Only one function is needed:
> AngularVel::GetAngularVelVec(double thxIncl1, double thzIncl1, double thySun1, double thzSun1,
            double thxIncl2, double thzIncl2, double thySun2, double thzSun2)
            
<br> </br>
It takes in 6 sensor angles (initial angles and final angles) and computes the angular velocity vector according to a time step 

To set the time step, access it with AngularVel::time_interval
- It is recommended get this by getting the time difference between the initial angle readings and the final angle readings.
