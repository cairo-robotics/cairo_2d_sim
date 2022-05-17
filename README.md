CAIRO Lab's 2D Simulation Environment for running 2D LfD experiments.

export SETUPTOOLS_USE_DISTUTILS=stdlib


# Cairo Lab's PyGame-based 2D Sandbox

## Description and Purpose

This is a repository containts interfaces to quickly build 2D interactive environments within the limits of PyGame. It integrates with ROS. 

## Table of Contents

- [Important Features](#important-features)
- [Installation](#installation)
- [FAQs](#faqs)

---
## Important Features <a name="important-features"></a>

### src/cairo_2d_sim
Location of all the source code.

#### src/cairo_2d_sim

### /launch
**/setupgame.launch** --> A basic laumch file that 



### /setup_scripts
**focal_noetic_basic_setup.bash** --> this script will prompt you to install basic software for Ubuntu 20.04 / ROS Noetic for new machine installations.

**sawyer_install.sh** --> this script will help you to install all the required packages for Sawyer (and some others needed to get sawyer for noetic working properly w/ moveit and gazebo etc)

### /rosinstall
.rosinstall files for use by wstools and scripts.

---
## Installation <a name="installation"></a>


```
---
## FAQ and Common Issues <a name="faqs"></a>

Please contact Carl Mueller - carl.mueller@colorado.edu for any help or if anything does not work properly.

If you see an obvious fix, want to add anything, please feel free to do a pull request.
