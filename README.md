# Prerequisite

[`drake_cmake_installed` examples](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed)
can be built and run

```
cd drake-external-examples/drake_cmake_installed
mkdir build && cd build
cmake ..
make
/opt/drake/bin/drake-visualizer&
cd src/particles
./uniformly_accelerated_particle
```

# Generate resources

    erb res/tribot.rsdf > res/tribot.sdf

# Build & Run

```
mkdir build && cd build
cmake ..
make
/opt/drake/bin/drake-visualizer&
./tribot -- [-simulation_time +inf] [-target_realtime_rate 1.0] [-time_step 0]
```
