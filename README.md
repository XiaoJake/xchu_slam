# XCHU_SLAM

## Introduction

xchu_odom添加回环

![image-20201124154817082](README/image-20201124154817082.png)

## Dependency

- [GTSAM](https://github.com/borglab/gtsam/releases)(Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)

## Usage

### Run the package

1. Run the launch file:

```shell
roslaunch xchu_slam  mapping.launch 
```

2. Play existing bag files kitti, bag包播放时请0.1倍速，因为目前性能上还未优化，在bag包播放完成后，建图也将结束，后续将处理成离线的，逐帧处理。ctrl+c关闭终端则自动保存地图。

```shell
rosbag play kitti_2011_10_03_drive_0027_synced.bag --clock -r 0.1
```

## Issues

- 线程安全
- 优化点云配准的初始估计(目前imu和编码器不可用，请设置为false)
- pitch 的累计误差导致高度漂移问题
- 目前位姿有抖动情况，尤其是 z 轴，还未检查问题（与定位时的抖动比较像，应该可以一起解决）

## TODOs

- 以`均值-协方差`格式存储/加载 NDT 地图
- 基于特征点的回环检测
- 借鉴 Apollo NDT 定位
