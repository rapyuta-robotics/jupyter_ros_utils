FROM condaforge/mambaforge:4.10.3-7

# Setup required user for running binder
# https://mybinder.readthedocs.io/en/latest/tutorials/dockerfile.html
ARG NB_USER=jovyan
ARG NB_UID=1000
ENV USER ${NB_USER}
ENV NB_UID ${NB_UID}
ENV HOME /home/${NB_USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${NB_UID} \
    ${NB_USER}

RUN apt-get update > /dev/null && \
    apt-get install --no-install-recommends -y libgl1-mesa-glx libglu1-mesa && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    conda config --env --add channels conda-forge && \
    conda config --env --add channels robostack && \
    conda config --env --set channel_priority strict && \
    mamba update -y mamba && \
    mamba install -y python=3.8 xeus-cling xwidgets widgetsnbextension jupyterlab ros-noetic-desktop mesa-libgl-devel-cos6-x86_64 mesa-dri-drivers-cos6-x86_64 libselinux-cos6-x86_64 libxdamage-cos6-x86_64 libxxf86vm-cos6-x86_64 libxext-cos6-x86_64 xorg-libxfixes && \
    conda clean -tipsy && \
    conda clean -afy

COPY . ${HOME}
USER root
RUN chown -R ${NB_UID} ${HOME}
USER ${NB_USER}
WORKDIR ${HOME}
RUN jupyter-notebook --generate-config && \
    echo 'c.NotebookApp.terminado_settings = { "shell_command": ["/bin/bash"] }' >> /home/jovyan/.jupyter/jupyter_notebook_config.py
