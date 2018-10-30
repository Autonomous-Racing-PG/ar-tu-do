#!/bin/bash

set -e

# Remove image if present
docker rm -f $2 &>/dev/null || true
docker pull $1
docker run -d -it --name $2 --privileged $1
docker exec $2 mkdir $2
docker cp . $2:/$2
