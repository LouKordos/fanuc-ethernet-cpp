#!/bin/bash
set -exo pipefail

if [[ -z "${DOCKER_FLAG_FOR_RUN_SCRIPT}" ]]; then
    echo "Host (no docker) detected."
    docker compose --progress plain up -d
    echo "Docker container is now running, starting interactive shell. Run /app/build-and-run.sh inside the shell to proceed."
    docker exec -it fanuc-ethernet-cpp-fanuc-ethernet-cpp-1 /bin/bash
else
    echo "Docker detected."
    cd /app/src 
    cmake -DCMAKE_BUILD_TYPE=Release -S . -B /app/src/build
    cmake --build /app/src/build -j $(nproc) -v
    /app/src/build/tests/move_test
fi
