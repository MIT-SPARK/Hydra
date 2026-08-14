# Hydra-Evaluation

This directory contains various evaluation code for Hydra, split between c++ executables and a python package

To build the c++ code, build hydra with the option `-DHYDRA_ENABLE_EVAL=ON` (which is on by default).

To use the python code, setup a virtual environment and pip install this directory. You can then run

```
hydra-eval --help
```
or
```
python -m hydra_eval --help
```
to see possible commands.

### Timing

After running hydra, you can evaluate the timing performance of various components by
```
# -t will plot the elapsed time against dataset time, omitting it will show distributions
hydra-eval timing plot /path/to/results [-t]
```
or
```
hydra-eval timing show /path/to/results
```
will show (a very coarse) breakdown of timing by layer.
