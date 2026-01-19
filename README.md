# ridesense

real-time perception and telemetry for motorcycles

## idea

a live program that ingests sensors, runs perception, fuses data, and exposes results
nothing more magical than that

## goals

- real-time camera, imu, gps ingestion
- time synchronization across sensors
- on-device perception
  1. slam
  2. object detection
  3. lane detection
- data fusion for vehicle state
- analytics on riding behavior
- live streaming to a ui
- replay

### architecture

```bash
capture -> sync -> perception -> fusion -> analytics -> ui
```

### build

i don't know how you build it for windows system :/

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)

./ridersense # to run the project

```

### dependencies:

- opencv
- spdlog
- yaml-cpp
- pthreads

### current status

stuff in the repo, basically placeholders for now (no real data)

### license

mit, do whatever you want
