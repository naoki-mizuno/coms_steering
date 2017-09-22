# COMS Steering

## Description

Package for controlling the steering wheel for the COMS EV project.

## Dependencies

- [serial](http://wiki.ros.org/serial)

## Building and Running

```
$ cd ~/ros/catkin_ws/src
$ git clone https://github.com/naoki-mizuno/coms_steering
$ cd ~/ros/catkin_ws
$ catkin_make
$ rosrun coms_steering coms_steering_node
```

## Parameters

- `port`: port that the actuator is connected to
- `baudrate`: baud rate of the actuator
- `frequency`: frequency at which to publish the angle message
- `origin_offset`: pulse count between the mechanical and electrical origin`
- `limit_ccw`: [maximum CCW rotation angle in radians, pulse count at that
  point]
- `limit_cw`: [maximum CW rotation angle in radians, pulse count at that
  point]

## Subscribed topics

- `cmd_steer`: commands the steering wheel angle in radians

## Published topics

- `angle`: current angle of the steering wheel in radians

## License

MIT

## Author

Naoki Mizuno
