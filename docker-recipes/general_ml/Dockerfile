# General machine learning
# Requires nvidia gpu with driver 396.37 or higher


FROM nvidia/cudagl:10.1-devel-ubuntu18.04


# Install cudnn
#=============================================================
ENV CUDNN_VERSION 7.6.4.38
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=$CUDNN_VERSION-1+cuda10.1 \
libcudnn7-dev=$CUDNN_VERSION-1+cuda10.1 \
&& \
    apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*




#(sometimes EGL and stuff is helpful)
#=============================================================
# Install a few libraries to support both EGL and OSMESA options
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y wget doxygen curl libjsoncpp-dev libepoxy-dev libglm-dev libosmesa6 libosmesa6-dev libglew-dev libopencv-dev python-opencv python3-setuptools python3-dev python3-pip git



RUN pip3 install opencv-python==4.1.0.25 torch==1.1.0 torchvision==0.3.0 numpy==1.13.3 pandas==0.24.1 networkx==2.2 h5py tqdm vocab revtok Pillow tensorboardX==1.8 

RUN pip3 install git+https://github.com/Joeclinton1/google-images-download.git

#(some problems installing these unless upgrade pip)
RUN pip3 install --upgrade pip
RUN pip3 install img2vec_pytorch seqeval==0.0.12 transformers==2.7.0 pytorch-crf==0.7.2 dgl-cu101==0.4.3post2 webcolors gensim

#(make sure the ai2thor room data has been downloaded)
RUN apt-get install unzip

#add additional libraries as you develop below ..