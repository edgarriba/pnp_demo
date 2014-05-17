# Google Summer of Code 2014 - OpenCV - PnP Demo

Some explanations of the project: [Real Time Pose Estimation Tutorial and DLS implementation](http://www.google-melange.com/gsoc/project/details/google/gsoc2014/edgarriba/5757334940811264).

## Table of Contents

- [Updates](#updates)
- [Installation](#installation)
	- [Pre-requirements](#pre-requirements)
	- [Installation Steps](#installation-steps)
- [How to Run](#how-to-run)
- [Contributors](#contributors)

## Updates

- *May 17 2014*: Readme update.

## Installation

### Pre-requirements

This application is developed and tested on Linux machine running Ubuntu 12.04 LTS (64 bit) and OpenCV 2.4.6. However it should also work on other mainstream Linux and OpenCV distributions.

### Installation Steps

The installation follows the same steps needed usually to compile using CMake.

* Get the code: Clone (or download and unpack) to your workspace.
* Compile the pnp_demo: use the CMakeLists.txt to compile the code.

      ```bash
      $ cd ~/my_workspace
      $ git clone https://github.com/edgarriba/pnp_demo.git
      $ mkdir build && cd build
      $ cmake .. && make
      ```
      
## How to Run
The application executable is `pnp_demo` located in `~/my_workspace/pnp_demo/build`.


<pre>
# Default Setting - 50Hz non-realtime update, the drone transmission rate is 200Hz
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=False  _navdata_demo:=0

# 200Hz real-time update
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=0

# 15Hz real-rime update
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=1
</pre>

## Contributors

- [Edgar Riba](https://github.com/edgarriba) 
- [Alexander Shishkov](https://github.com/alekcac)
