FROM ubuntu:24.04

RUN apt-get update -y && \
	apt-get install -y g++-14 gcc-14 cmake build-essential gdb git vim libtbb-dev && \
	rm -rf /var/lib/apt/lists/*

# For color test on shell startup
RUN echo 'function color_test() {' >> ~/.bashrc && \
    echo '    for i in {0..255} ; do' >> ~/.bashrc && \
    echo '        printf "\\x1b[48;5;%sm%3d\\e[0m " "$i" "$i"' >> ~/.bashrc && \
    echo '        if (( i == 15 )) || (( i > 15 )) && (( (i-15) % 6 == 0 )); then' >> ~/.bashrc && \
    echo '            printf "\\n";' >> ~/.bashrc && \
    echo '        fi' >> ~/.bashrc && \
    echo '    done' >> ~/.bashrc && \
    echo '}' >> ~/.bashrc && \
    echo 'color_test' >> ~/.bashrc

WORKDIR /app/src
COPY ./ /app/

ENV CC=gcc-14
ENV CXX=g++-14
ENV DOCKER_FLAG_FOR_RUN_SCRIPT=1

RUN cmake -DCMAKE_BUILD_TYPE=Release -S . -B ./build
RUN cmake --build ./build -j $(nproc) -v

WORKDIR /app

# Using CMD instead of ENTRYPOINT here so that docker-compose.yml for vs code can override it.
CMD build-and-run.sh
