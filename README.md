# Jupyter ROS Utils

This repo contains tools to ease the pain of setting up C++ libraries such as ROS into Jupyter Notebook. We also provided examples notebook to load 3rd party C++ libraries and ROS.

If you're interested to know how things work behind the scene, please check out [our blog post](https://www.rapyuta-robotics.com/2021/07/09/running-c-ros-in-jupyter-using-xeus-cling/)

# Setup

You need to install conda to start the setup.
Link to install [conda](https://docs.anaconda.com/anaconda/install/linux/)

```sh
git clone git@github.com:rapyuta-robotics/jupyter_ros_utils.git
cd jupyter_ros_utils

# Use requirements.txt to create env
conda create --name <env> --file requirements.txt
conda activate --name <env>
#                OR
# Use environment.yml to create env
conda env create -f environment.yml
conda activate jupyter_ros

cd cpp/notebooks
jupyter-notebook
```

After finishing the installation, you can check the instruction inside of our notebook example [here](cpp/notebooks).

# Status
- [x] Automatic flags generation for for 3rd party library
- [x] Example notebook for 3rd party library([OpenCV](cpp/notebooks/cpp_opencv4.ipynb), [PCL](cpp/notebooks/cpp_pcl-1.10.ipynb))
- [x] Example notebook: ROS1 [Publisher](cpp/notebooks/cpp_ros_publisher.ipynb), and [Subscriber](cpp/notebooks/cpp_ros_subscriber.ipynb)
- [x] Example notebook: Widget ROS1 [Publisher](cpp/notebooks/cpp_ros_async_callback_publisher.ipynb), [Subscriber](cpp/notebooks/cpp_ros_async_subscriber.ipynb), and [Param](cpp/notebooks/cpp_ros_param_widget.ipynb)
- [ ] Example notebook: ROS2 Subscriber and Publisher
- [x] Example notebook: Widget ROS2 [Subscriber](cpp/notebooks/ros2_subscriber.ipynb)
- [ ] Example notebook: Widget ROS2 Publisher
