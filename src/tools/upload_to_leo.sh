#!/usr/bin/env bash

TARGET=all
if [[ $# -eq 1 ]]; then
    TARGET=$1
fi

# Descend into base folder
pushd ../../../

# Compress the sources and files
tar cjvf argos3-leo.tar.bz2 --exclude build --exclude .git argos3-leo

# Copy the archive
if [[ "x${TARGET}" = "xall" ]]; then
    for LEO in leo1 leo2 leo3; do
        scp argos3-leo.tar.bz2 ${LEO}:~/ARGoS
    done
else
    scp argos3-leo.tar.bz2 ${TARGET}:~/ARGoS
fi

popd
