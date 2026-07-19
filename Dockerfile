FROM ubuntu:20.04
LABEL maintainer "Chris Ohk <utilforever@gmail.com>"

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
    build-essential=12.8ubuntu1.1 \
    autoconf autoconf-archive automake curl git libtool tar unzip zip \
    python3-dev=3.8.2-0ubuntu2 \
    python3-pip=20.0.2-5ubuntu1.11 \
    --no-install-recommends \
    && pip3 install --no-cache-dir cmake==3.31.6 \
    && rm -rf /var/lib/apt/lists/*

ENV VCPKG_ROOT=/opt/vcpkg
RUN git clone https://github.com/microsoft/vcpkg.git "$VCPKG_ROOT" \
    && git -C "$VCPKG_ROOT" checkout 9432c416e5c543eded0b4df35a4d347b2e669208 \
    && "$VCPKG_ROOT/bootstrap-vcpkg.sh" -disableMetrics

COPY . /app

WORKDIR /app/build
RUN cmake .. && \
    make -j "$(nproc)" && \
    make install && \
    bin/UnitTests

WORKDIR /app
RUN pip3 install --no-cache-dir -r requirements.txt . && \
    python3 -m pytest Tests/PythonTests

WORKDIR /
