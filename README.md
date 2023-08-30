# Line Tracking Race
Simulated line tracking robot developed for the final exam of the Robotics course at UniPi.

## Usage
You can use the provided `full.launch` launch file to launch the simulation along with all nodes:

```
roslaunch dapozzo_line_tracking full.launch
```

Remember that you can pass launch arguments with the `name:=value` syntax. Consult the section below to learn about available arguments.

## Nodes

### Planner Node
The planner node extracts track information from raw camera images. A launch file (`planner.launch`) is provided.
Available parameters:
- `viz` (True | <u>False</u>): whether to enable live visualization of planning data with OpenCV;

- `strategy` (centerline | <u>centroid</u>): the first planning strategy chooses a point from the track's centerline as waypoint, while the second strategy uses the centroid computed on the detected track shape;

- `error_type` (offset | <u>angle</u>): the first approach uses the x-axis offset between the waypoint and the center of the image as error, while the second one uses the angle between the line connecting the bottom-center pixel and the waypoint and the vertical line passing through the image's center. Both errors are remapped into the `[-1, 1]` range.

### Control Node
The control node uses a PID controller in order to accelerate and steer the car based on the provided waypoint error. A launch file (`control.launch`) is provided.
Available parameters (PID parameters are read from `config/pid_params.yaml`):
- k_p: PID proportional gain;
- k_i: PID integral gain;
- k_d: PID derivative gain.
- duration: how long this node should run for (in seconds). Useful for performing multiple runs of the same length. The car automatically stops when the duration reaches 0.


### Manual Control Node
The manual control node allows for basic keyboard control of the car using the arrow keys.

## Logs
Logs in `.csv` format can be found in the `logs` directory.
