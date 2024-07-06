# Turtlebot Spaceship Navigation Simulation Project

This project was contributed to and designed by:

  Rejus Bulevicius (https://github.com/rejus-b)
  
  Devesh Kanal     (https://github.com/dk0123)
  
  Dexter Lardner   (https://github.com/DexterL42)
  
  Leandro Russo    (https://github.com/Sandpaperr)

## Project Outline

The project aim, over the course of a week, was to create a turtlebot that could explore a spaceship. This spaceship would have 'broken' rooms and obstacles, our goal was to create a robust solution regardless of obstacles and map dimensionality, which would navigate and identify planets within windows inside rooms and calculate the distance between planets.

We used ROS2 for programming the turtlebot, Gazebo/Rviz for visualising the robot and for localisation display. The robot itself had access to LiDAR and the Nav2Goal stack to help facilitate movement, while relying on openCV for object detection, and a convultional neural network (CNN) for planet classification. At the end of the week, we took our robot from the simulated environment to a real world testing environment where we saw mixed results, with further documentation of the entire project in the pdf listed at the bottom. 

The key tasks can be broken down into:

- **Finding the correct room**: Using openCV for object detection, identifying the state of a room.
- **Navigating the room**: Achieved using lidar and wall following algorithms to have the best chance of finding windows.
- **Classifying planets**: A CNN was trained and used on planet identification.
- **Identifying windows**: OpenCV was used for object detection of windows, and then a brute-force algorithm for image stitching. 

## Documentation

For detailed documentation, please refer to the [Robotic Spaceship Report.pdf](./Robotic%20Spaceship%20Report.pdf).
