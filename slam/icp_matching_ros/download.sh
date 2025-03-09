#!/bin/sh

if [ ! -d pcd ]; then
    mkdir -v pcd
fi
wget -O pcd/bun000.pcd https://github.com/3d-point-cloud-processing/3dpcp_book_data/raw/master/bun000.pcd
wget -O pcd/bun045.pcd https://github.com/3d-point-cloud-processing/3dpcp_book_data/raw/master/bun045.pcd
