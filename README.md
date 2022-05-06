CAIRO Lab's 2D Simulation Environment for running 2D LfD experiments.



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

### /launch
**/record.launch** --> A basic launch file that starts the game node and recording node in order to record 2D demonstrations.

---
## Installation <a name="installation"></a>

Install locally into your src/ directory of your ROS1 workspace and build your workspace.

Note: This package was built for python 3.8 or greater. As such, distutils is deprecated in favor of setuputils. As such,
ROS will attempt to build the python package using distutils which can cause a build failure. One remedy is to prepend the 
following on your build command in your terminal:

```
$ export SETUPTOOLS_USE_DISTUTILS=stdlib caktin_make
```

---
## FAQ and Common Issues <a name="faqs"></a>

Please contact Carl Mueller - carl.mueller@colorado.edu for any help or if anything does not work properly.

If you see an obvious fix, want to add anything, please feel free to do a pull request.
