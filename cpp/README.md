# C++ Jupyter ROS Utils

Check out the usage of the scripts in the notebooks folder:
- [OpenCV](notebooks/cpp_opencv4.ipynb)
- [PCL](notebooks/cpp_pcl-1.10.ipynb)
- [ROS publisher](notebooks/cpp_ros_publisher.ipynb)
- [ROS subscriber](notebooks/cpp_ros_subscriber.ipynb)

## Generate cling 3rd party C++ library
The command below pretty much cover all 3rd party library given that has `pkg-config` configuration
```
python3 generate_cling_3rd_party.py libname [--target-dir .]
```

## Generate cling from boost git
```
python3 generate_cling_boost.py /path/to/boost_git [--target-dir .]
```