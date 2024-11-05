# docker build --rm -f Dockerfile -t openexr-ht:latest .
# docker run -it --rm -v C:\\temp:/tmp/ openexr-ht:latest
FROM ubuntu:jammy

RUN apt-get update

# disable interactive install 
ENV DEBIAN_FRONTEND noninteractive

# install developement tools
RUN apt-get -y install cmake
RUN apt-get -y install g++
RUN apt-get -y install git
RUN apt-get -y install unzip
RUN apt-get -y install libnuma-dev
RUN apt-get -y install python3

# install developement debugging tools
RUN apt-get -y install valgrind

# build OpenEXR 
WORKDIR /usr/src/OpenEXR
COPY . .
WORKDIR /usr/src/OpenEXR/build
RUN cmake .. 
RUN make
RUN make install

# finalize docker environment



