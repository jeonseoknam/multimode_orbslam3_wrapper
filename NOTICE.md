# Project Notice

This repository (`multimode_orbslam3_warpper`) is a **standalone modified
copy** of [`zang09/ORB_SLAM3_ROS2`](https://github.com/zang09/ORB_SLAM3_ROS2)
adapted for the fault-tolerant LiDAR-Visual-GNSS multimodal localization
pipeline ([`fault-tolerant-localization-pipeline`](https://github.com/jeonseoknam/fault-tolerant-localization-pipeline)).

## Upstream

- Source: https://github.com/zang09/ORB_SLAM3_ROS2
- Branch: `humble`
- License: MIT (declared in `package.xml`)

This wrapper depends on the **ORB-SLAM3 C++ library**
([`UZ-SLAMLab/ORB_SLAM3`](https://github.com/UZ-SLAMLab/ORB_SLAM3) / GPLv3),
which must be installed separately at `~/ORB_SLAM3`. The wrapper code in this
repository remains MIT; the GPLv3 obligations apply only to the underlying
library binary.

## Modifications in this fork

- MORAI camera intrinsics (`config/monocular/MORAI_*.yaml`)
- `monocular_align.yaml` for online relay against piecewise Sim(3) atlas
- Camera fault-injection topic plumbing (`/camera/image_faulty`)
- Launch argument `use_faulty`
- Various build / config fixes for ROS 2 Humble + CUDA toolchain
