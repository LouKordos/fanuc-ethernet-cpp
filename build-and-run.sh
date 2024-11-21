#!/bin/bash
set -exo pipefail

# Default build type
BUILD_TYPE="Release"

# Parse command line arguments

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --build-type=*)
            BUILD_TYPE="${1#*=}"
            shift
            ;;
        --build-type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        *)
            echo "Unknown parameter passed: $1, exiting."
            exit 1
            ;;
    esac
done
        

if [[ -z "${DOCKER_FLAG_FOR_RUN_SCRIPT}" ]]; then
    echo "Host (no docker) detected."
    docker compose --progress plain up -d
    echo "Docker container is now running, starting interactive shell. Run /app/build-and-run.sh inside the shell to proceed."
    docker exec -it fanuc-ethernet-cpp-fanuc-ethernet-cpp-1 /bin/bash
else
    echo "Docker detected."
    cd /app/src
    cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -S . -B /app/src/build
    cmake --build /app/src/build -j $(nproc) -v
    if [[ "${BUILD_TYPE}" = "Debug" ]]; then
        gdb /app/src/build/tests/move_test
    else
        /app/src/build/tests/move_test
    fi
fi