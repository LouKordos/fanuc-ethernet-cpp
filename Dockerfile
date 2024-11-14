FROM ubuntu:24.04

RUN apt-get update -y && \
	apt-get install -y g++-14 gcc-14 cmake build-essential gdb git vim libtbb-dev && \
	rm -rf /var/lib/apt/lists/*

WORKDIR /app/src

COPY ./ /app/
# For "dubious ownership" error inside dev container
RUN git config --global --add safe.directory /workspaces/${localWorkspaceFolderBasename}

ENV CC=gcc-14
ENV CXX=g++-14
ENV DOCKER_FLAG_FOR_RUN_SCRIPT=1

RUN cmake -DCMAKE_BUILD_TYPE=Release -S . -B ./build
RUN cmake --build ./build -j $(nproc) -v

WORKDIR /app

# Using CMD instead of ENTRYPOINT here so that docker-compose.yml for vs code can override it.
CMD build-and-run.sh
