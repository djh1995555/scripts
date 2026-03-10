#!/bin/bash -ex

if [ "$#" -eq 0 ]; then
  echo "USAGE: $0 <target> ..."
  echo "  e.g. $0 //release:whole"
  exit 1
fi


bazelisk build "$@" \
  --config=aarch64-linux-cuda-driveos701 \
  --config=select-fetch \
  --config=release \
  --define deploy_platform=dd3_thor \
  --config=cartype_add3 \
  --config=carmodel_mx11_1_5 \
  -k \


bazelisk build "$@" \
  --config=aarch64-linux-cuda-driveos701 \
  --config=select-farm \
  --config=release \
  --define deploy_platform=dd3_thor \
  --config=cartype_add3 \
  --config=carmodel_mx11_1_5 \
  --jobs=24 \
  -k \