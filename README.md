# Line Tracking Race
Simulated line tracking robot developed for the final exam of the Robotics course at UniPi.

## Usage
You can use the provided `full.launch` launch file to launch all nodes with one command:

```
roslaunch dapozzo_line_tracking full.launch
```

The only available command line argument is `debug`, a boolean which decides whether to display debug data. Its default value is `false`, but it can be set to `true` as such:
```
roslaunch dapozzo_line_tracking full.launch debug:=True
```

## Nodes

### Camera Node
The camera node extracts track information from raw camera images. A launch file (`camera.launch`) is provided.
Parameters:
- debug (true | <u>false</u>).

### Control Node
The control node uses a PID controller in order to accelerate and steer the car based on the provided waypoint x-axis offset. A launch file (`control.launch`) is provided.
Parameters (read from `config/pid_params.yaml`):
- k_p: PID proportional gain;
- k_i: PID integral gain;
- k_d: PID derivative gain.


### Manual Control Node
The manual control node allows keyboard control of the car through the arrow keys.