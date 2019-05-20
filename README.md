# path_planning_highway_driving
Udacity self driving car nano degree project #6
## Reference: Udacity's path planning walkthrough session: https://www.youtube.com/watch?v=7sI3VHFPP0w&feature=youtu.be
## The above reference walked me through the following points:
- how to make the car goes straight
- how to make the car follow the path 
- how to reduce jerk at the cold start
- how to reduce the vehicle speed when facing the front car to avoid collision. 
## Project rubrics:
CRITERIA | MEETS SPECIFICATIONS 


|CRITERIA                                                        |MEETS SPECIFICATIONS|
|:---                                                            |:-                  |
|The car is able to drive at least 4.32 miles without incident.. |The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.|
|The car is able to drive at least 4.32 miles without incident.. |The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.|

|The car drives according to the speed limit.                    |The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.|

|Max Acceleration and Jerk are not Exceeded.                     |The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.|

|Car does not have collisions.                                   |The car must not come into contact with any of the other cars on the road.|

|The car stays in its lane, except for the time between changing lanes.|The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.|

|The car is able to change lanes|The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.|
