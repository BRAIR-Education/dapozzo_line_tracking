# Line Tracking Race
Simulated line tracking robot developed for the final exam of the Robotics course at UniPi.

## Nodes

### Camera Node
The camera node extracts track information from raw camera images.
It can be launched as follows:

``` 
rosrun dapozzo_line_tracking camera_node.py
```

### Control Node
TODO

### Manual Control Node
The manual control node allows keyboard control of the car through the arrow keys. It can be launched as follows:

```
rosrun dapozzo_line_tracking manual_control_node.py
```