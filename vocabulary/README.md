# ORB vocabulary

`ORBvoc.txt.tar.gz` is tracked here (42 MB compressed, 139 MB extracted).
**Extract it once before building:**

```bash
cd <workspace>/src/orbslam3_ros2/vocabulary
tar -xzf ORBvoc.txt.tar.gz
```

That is all — `ORBvoc.txt` lands next to the archive, which is where the build
expects it. The extracted file stays untracked (`.gitignore`), so it will not
be committed back.

`CMakeLists.txt` installs this directory to `share/orbslam3/vocabulary/`, and
`launch/monocular.launch.py` resolves the vocabulary as
`share/orbslam3/vocabulary/ORBvoc.txt`. If you did not build with
`--symlink-install`, rebuild after extracting. The archive itself is excluded
from the install tree.

## Where this file comes from

It is the vocabulary distributed with
[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (`Vocabulary/ORBvoc.txt.tar.gz`),
byte-identical to upstream, and it carries ORB-SLAM3's GPLv3 license. It is
mirrored here only so that a clone of this repository builds without a separate
download.
