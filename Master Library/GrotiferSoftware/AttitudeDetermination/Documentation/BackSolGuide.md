# Guide

ALL ANGLE UNITS ARE IN RADIANS, THE SOLVERS DO NOT WORK FOR ANGLES GREATER THAN A CERTAIN POINT (DUE TO NATURE OF TRIG FUNCTIONS). But, for the purposes of the demo and the actual project, Grotifer is not planned to rotate greater than these angles or else the sun will be out of the sun sensor and solar panel frames (in which other light sensors take over to reorient).


## Files
**Attitude Determination Documentation.pdf** - Details the methods we use to determine the attitude matrix (rotation matrix to get from the global reference frame to the body-fixed frame) given four angle measurements.  
<br></br>
**BackwardSolClass.cpp** and **BackwardSolClass.hpp** - The code implementation of the method detailed above.  
<br></br>
**asa047.cpp** and **asa047.hpp** - The library files for the old Nelder-Mead implementation.  
<br></br>
**Libraries/** - A folder containing all dependences. To use this model, simply copy and paste the Library folder into your project and include all the files in it.


## Backward Solution Class Functions (**IMPORTANT**)

### Solver Functions (The Ones That Perform The Actual Solution)
#### `BackwardSol::AlgebraicSolutionMatrix(double thxIncl, double thzIncl, double thySun, double thzSun)`
  - Returns the rotation matrix given the sensor readings using the closed-form (algebraic method) solution.
  - Much faster than Nelder-Mead as it is not iterative.
<br></br>

#### `BackwardSol::NelminSolutionMatrix(double thxIncl, double thzIncl, double thySun, double thzSun)`
  - Returns the rotation matrix given the sensor readings using the Nelder-Mead method (iterative method).
  - A lot slower, but can be used as a backup/checker for the algebraic method.

### Helper Functions 
#### `BackwardSol::GetEulerFromRot`
  - Takes in a rotation matrix and stores the euler angles (in order of global x, y and z) into the given pointer variables
<br></br>

