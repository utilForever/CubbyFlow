FROM ubuntu:20.04
LABEL maintainer "Chris Ohk <utilforever@gmail.com>"

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
    build-essential=12.8ubuntu1.1 \
    python3-dev=3.8.2-0ubuntu2 \
    python3-pip=20.0.2-5ubuntu1.11 \
    --no-install-recommends \
    && pip3 install --no-cache-dir cmake==3.31.6 \
    && rm -rf /var/lib/apt/lists/*

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
