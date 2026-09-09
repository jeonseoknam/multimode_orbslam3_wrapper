# ORB vocabulary

`ORBvoc.txt` (~139 MB) is **not tracked in git** — it ships with ORB-SLAM3.
`CMakeLists.txt` installs this whole directory, so the directory itself must
exist for the build to succeed; this file keeps it in the repository.

Copy the vocabulary here once, after installing ORB-SLAM3:

```bash
cd ~/ORB_SLAM3/Vocabulary
tar -xf ORBvoc.txt.tar.gz            # only needed the first time
cp ORBvoc.txt <workspace>/src/orbslam3_ros2/vocabulary/
```

`launch/monocular.launch.py` resolves it as
`share/orbslam3/vocabulary/ORBvoc.txt`, so a rebuild is required after
copying if you did not build with `--symlink-install`.
