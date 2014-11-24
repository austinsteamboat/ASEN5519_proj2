# vidro #

An API combing pymavlink and Vicon streaming data to close the feedback control loop

###Resources###

* [pymavlink](https://github.com/mavlink/pymavlink)

* [Vicon Streaming](https://github.com/cfinucane/pyvicon)

* [MavProxy](https://github.com/tridge/MAVProxy)


## Contents ##

### vidro.py ###

This is the class for connecting to the APM and vicon. It is able to handle SITL of the APM which is useful for testing.

### position_controller.py ###

Example PID controller using rc overrides.

### flight_test.py ###

Implementation of vidro and rc override controller to close the loop.

### demo.py ###

Demonstration using vidro and position controller to follow where a wand points.


