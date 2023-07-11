# Line Tracking Race
Simulated line tracking robot developed for the final exam of the Robotics course at UniPi.

## Usage
You can use the provided `full.launch` launch file to launch the simulation along with all nodes:

```
roslaunch dapozzo_line_tracking full.launch
```

Remember that you can pass launch arguments with the `name:=value` syntax. Consult the section below to learn about available arguments.

## Nodes

### Camera Node
The camera node extracts track information from raw camera images. A launch file (`camera.launch`) is provided.
Parameters:
- viz (True | <u>False</u>).

### Control Node
The control node uses a PID controller in order to accelerate and steer the car based on the provided waypoint x-axis offset. A launch file (`control.launch`) is provided.
Parameters (PID parameters are read from `config/pid_params.yaml`):
- k_p: PID proportional gain;
- k_i: PID integral gain;
- k_d: PID derivative gain.
- duration: how long this node should run for (in seconds). Useful for performing multiple runs of the same length.


### Manual Control Node
The manual control node allows keyboard control of the car through the arrow keys.

## Logs
Logs in `.csv` format can be found in the `logs` directory.

## Plots
Plots in `.png` format can be found in the `plots` directory.