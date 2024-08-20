# Line Navigator

A proof of concept for, parking lane marking robot. The robot uses camera image to detect lane markings and initates painting process by navigating the lane waypoints. Once navigated, the painted lines are marked as obstacles along the costmap so that the robot doesn't drive on them again.

## Launch

Build the workspace with provided packages and source it. Afterwards, run the following command.

```
roslaunch line_navigator navigation.launch
```

# Demo

![](https://github.com/MahirGulzar/line_navigator/blob/main/demo.gif)