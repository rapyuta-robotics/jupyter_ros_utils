### Ardiya Note: mostly stolen from
### https://github.com/conda-forge/miniforge-images/blob/master/ubuntu/Dockerfile
FROM ros:noetic-perception-focal

# install bootstrap tools
RUN apt-get update > /dev/null
RUN apt-get install --no-install-recommends -y \
    wget bzip2 ca-certificates \
    git > /dev/null

# cleanup apt
RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/*

### install conda miniforge ###
ARG MINIFORGE_NAME=Miniforge3
ARG MINIFORGE_VERSION=4.10.3-1
ARG TINI_VERSION=v0.18.0
ARG TARGETPLATFORM=amd64

ENV CONDA_DIR=/opt/conda
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH=${CONDA_DIR}/bin:${PATH}

RUN TARGETARCH="$(echo ${TARGETPLATFORM} | cut -d / -f 2)"; case ${TARGETARCH} in "ppc64le") TARGETARCH="ppc64el" ;; *) ;; esac ; \
    wget --no-hsts --quiet https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini-${TARGETARCH} -O /usr/local/bin/tini && \
    chmod +x /usr/local/bin/tini && \
    wget --no-hsts --quiet https://github.com/conda-forge/miniforge/releases/download/${MINIFORGE_VERSION}/${MINIFORGE_NAME}-${MINIFORGE_VERSION}-Linux-$(uname -m).sh -O /tmp/miniforge.sh && \
    /bin/bash /tmp/miniforge.sh -b -p ${CONDA_DIR}
# TODO: figure out miniforge version that uses python 3.8 so we don't waste internet bandwith
RUN /bin/bash -c "conda install -c conda-forge python=3.8 xeus-cling xwidgets widgetsnbextension jupyterlab boost=1.71.0 boost-cpp=1.71.0"

# cleanup conda installation
RUN rm /tmp/miniforge.sh
RUN conda clean -tipsy
RUN find ${CONDA_DIR} -follow -type f -name '*.a' -delete
RUN find ${CONDA_DIR} -follow -type f -name '*.pyc' -delete
RUN conda clean -afy

RUN echo ". ${CONDA_DIR}/etc/profile.d/conda.sh && conda activate base" >> /etc/skel/.bashrc

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

RUN echo ". ${CONDA_DIR}/etc/profile.d/conda.sh && conda activate base" >> ${HOME}/.bashrc
COPY . ${HOME}
USER root
RUN chown -R ${NB_UID} ${HOME}

# Configure container startup
USER ${NB_USER}
WORKDIR ${HOME}
