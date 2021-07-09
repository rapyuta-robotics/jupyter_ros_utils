# Description

Repository contains some of the basic ROS examples using C++ libraries implemeted in Jupyter Notebook.

# Setup

You need to install conda to start the setup.
Link to install [conda](https://docs.anaconda.com/anaconda/install/linux/)

```
git clone git@github.com:rapyuta-robotics/jupyter_ros_utils.git
cd jupyter_ros_utils

// Use requirements.txt to create env
conda create --name <env> --file requirements.txt
conda activate --name <env>
                OR
// Use environment.yml to create env
conda env create -f environment.yml
conda activate jupyter_ros

cd cpp/notebooks
jupyter-notebook
```

Follow the instruction in the cells of each notebook.
