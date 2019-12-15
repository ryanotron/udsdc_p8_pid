# PID Controller

For a self-driving car.

Forked from the starter repo [here](https://github.com/udacity/CarND-PID-Control-Project)

## Building and running

This project uses cmake. You can build it with

```bash
$ mkdir build && cd build
$ cmake ..
$ make
```

You will get a `pid` executable you can run. You would need the [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45).

## Configuration

There are two PID controllers inside, one to control steering and another to control throttle. You can specify their gains, along with other parameters, with a parameter file. The parameter file must be named `params.txt` and placed in the root folder of the source code (one directory above `build`).

For project submission purpose, the default values in `params.txt` is the one submitted.

`steer` line contains gains for steering and `throttle` line contains gains for throttle, both in order: proportional, integral, differential.

`vmin` is the minimum speed for the car (in mph) and `vmax` for the maximum speed. The reference speed used by the throttle PID is a linear interpolation between `vmin` and `vmax`, linear in absolute cross-track-error. To set a fixed speed, just set them to the same value (this submission does that).

## Gain tuning

Gains were tuned by hand. I started by setting `K_p` and `K_d` to the same small value (0.01) and increase them together in 0.01 increment until the car could pass the bridge in the track (and usually hit the block in the first left turn after). At the end of this stage, I had `K_p, K_d = 0.05`

After, I gradually increased `K_d` until the car managed to miss the block mentioned earlier. At `K_d = 0.7` the car could miss the block and the navigate the next two sharp turns (narrowly!). If left alone, the car would be able to indefinitely navigate the course, though not very well.

Next, I add integral gain `K_i`. I started with 1/10 of `K_p` (so, 0.005) and decrease it slowly until the sharp turn performance stopped getting better. Sharp turn performance being how narrow the car misses the block after the bridge and the next two sharp turns.

Finally, I made minute adjustments to the gains to arrive at the values in the `params.txt` file.

Throttle gains were easier. Almost any small `K_p` value would work. I tuned them for 50 mph on the beginning stretch of the track and left them alone. They work well enough for slightly higher and lower speed references.