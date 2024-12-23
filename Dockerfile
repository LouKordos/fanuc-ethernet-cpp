FROM ubuntu:24.04

RUN apt-get update -y && \
	apt-get install -y g++-14 gcc-14 cmake build-essential gdb git curl vim libtbb-dev && \
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

ENV CC=gcc-14
ENV CXX=g++-14
ENV DOCKER_FLAG_FOR_RUN_SCRIPT=1

# RUN git clone https://github.com/nimbuscontrols/EIPScanner.git /tmp/EIPScanner
# RUN cd /tmp/EIPScanner && git reset --hard d20ef61
# RUN mkdir build
# WORKDIR /tmp/EIPScanner/build
# RUN cmake ..
# RUN cmake --build . --target install --parallel

# NEEDS TO RUN AFTER INSTALLING ALL DEPENDENCIES
RUN ldconfig

COPY ./ /app/
WORKDIR /app

RUN cmake -DCMAKE_BUILD_TYPE=Release -S . -B ./build
RUN cmake --build ./build -j $(nproc) -v

WORKDIR /app

# Using CMD instead of ENTRYPOINT here so that docker-compose.yml for vs code can override it.
CMD sleep infinity
