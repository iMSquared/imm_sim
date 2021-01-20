# IMM-SIM

Repository for the simulation environment for the [KAIST Intelligent Mobile Manipulation Lab](http://imsquared.kaist.ac.kr/).

## Installation

### Requirements

Currently, not all dependencies have been documented other than [pybullet](https://github.com/bulletphysics/bullet3) which is our underyling simulator.

See [setup.cfg](setup.cfg)

For development, the currently recommended way to setup is as follows:

```
pip3 install -e . --user
```

## Run

The package is still in development, and currently the only app available is in `sim.py`, i.e.

```
python3 sim.py
```

And the display should show something like the following:

![2021-01-21-sample-gibson-box-2d-env](img/2021-01-21-sample-gibson-box-2d-env.gif)
