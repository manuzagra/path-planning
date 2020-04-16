# **Path Planning**


In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

---
### Files Submitted & Usage

##### Files

My project includes the following source files:
* **src/json.hpp** helpful to work with json data
* **src/spline.h** contains a library to work with splines
* **src/helpers.h** various functions, from converting units to coordinates transforms
* **src/main.cpp** contains the execution flow
* **src/Eigen-3.3** folder containing Eigen library

And some extra files:
* **data** folder containing the waypoints of the map
* **CMakeLists.txt** to build the project using cmake
* **README.md** the readme of the original project from Udacity
* **model_documentation.md** this file with the results

##### Usage

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


---
### Strategy

##### Previous points
I decided to use some of the previous points, in the chosen configuration 20 of them. The reasons to take some of the previous points are:
* Ensure continuity between old trajectory and new trajectory. Taking some old points the transition between trajectories is smooth using splines to joint the waypoints.
* Not taking all the points from the previous path makes the car more reactive to incidents.


##### Cars detection
To use the information from the sensor fusion is essential to ensure a safe driving. Two questions must be solved with this information:
1. Is there a car in front of the ego car with danger of collision? This is answered checking if the car is in the same lane, if it is in front of the ego car and if the distance between cars is not enough.
2. Are there in other lanes gaps where ego car could go in case it needs to change lane? We need to check if there are no cars in a range of distances from where the ego car would finish a lane transition.


##### Trajectory generation
1. Take the last 2 points from the previous path as the first two waypoints.
2. Append to those points some extra waypoints equally spaced. The space between waypoints will determine how aggressive the conduction of the car is, in the final solution the distance is 40m.
3. Transform the waypoints to car coordinates. This is needed because we are using one variable polynomial splines where we the independent variable must be monotonic. We can see this in the code of "spline.h" in lines 293-295:
```c++
for(int i=0; i<n-1; i++) {
    assert(m_x[i]<m_x[i+1]);
}
```
This has some limitations, for example, it is not possible to do an U turn, but in a highway most of the trajectories will have a very straight trajectories if you see them from the point of view of the car, so we should not have any problem.
4. Calculate the distance between points having into account the *dt* of the simulator and the desired speed.
5. Generate the points using the spline.
6. Transform the points back to map coordinates.
7. Final trajectory is the part of the previous trajectory that we are reusing + the new generated points.


##### Video

<a href="http://www.youtube.com/watch?feature=player_embedded&v=y4uSOT5Tzc8
"><img src="http://img.youtube.com/vi/y4uSOT5Tzc8/0.jpg"
alt="Path Planning in highway" border="10" /></a>


---
### Reflection

I did a very simple implementation, and mainly using what was provided by Udacity team. The behavioural planner is a couple of *if* statements so there is no complex behaviour implemented. The car has a bias to overtake by the left lane, this is what driving rules say (at least in Spain), but sometimes the right lane is more adequate, I should implement a cost function to choose the best lane to overtake.
