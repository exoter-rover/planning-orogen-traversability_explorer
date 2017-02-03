# Traversability Explorer

## A component for simulated exploring of traversability cost map.

This component loads a traversability cost map from a file. 
It assumes that each cell contains a SBPL-compatible cost. 
The values are converted to drivability and reprsented using the Envire library.
The whole map is loaded at the beginning, but only the path in the field of view of the robot is outputted. This is to simulate the behaviour of the path planning subsystem during exploration without complete knowledge of the environment.

### Usage
An examle of the data format can be found in `data/`.  
Details on input, output and triggering can be found in [`.orogen`](https://github.com/exoter-rover/planning-orogen-traversability_explorer/blob/master/traversability_explorer.orogen) file.  
The field of view is trapezoidal:
  * ``robot_fov_a``  
  the width of FOV in front of the rover
  * ``robot_fov_b``  
  the width of FOV at the extend.
  * ``robot_fov_l``  
  height of the trapezoid   
  
Other configuration values are map grid-cell size (resolution) and filename, as described in [`.orogen`](https://github.com/exoter-rover/planning-orogen-traversability_explorer/blob/master/traversability_explorer.orogen) file.

**Author: [Jan Filip](mailto:jan.filip2@gmail.com "Contact the author"),  
Contact: [Martin Azkarate](mailto:Martin.Azkarate@esa.int "Contact the maintainer"),  
Affiliation: Automation and Robotics Laboratories, ESTEC, ESA**

