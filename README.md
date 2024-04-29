
# ROS2 porting of Fast-Planner

## Host build
<!-- Please update steps here... -->

## Docker build  

### Clone the repo

```bash
git clone -b  basavaraj/fix-compilation https://github.com/RohitPawar2406/Fast-Planner-ROS2.git
```

### Build

```bash
cd  Fast-Planner-ROS2/src
./build.sh # This will build the docker image
```

### Run

```bash
./run.sh # This will build the docker image
colcon build 
```
