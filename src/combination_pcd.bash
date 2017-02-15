#!/bin/bash -e

mkdir -p ~/pcd/making_envir_cloud/laserscan_data ~/pcd/making_envir_cloud/filtered_data ~/pcd/making_envir_cloud/raw_data

cd ~/pcd/making_envir_cloud/filtered_data
ls -1 | sort -n | grep [0-9].pcd$ | while read f;
do
    cat $f | awk '{if(NR>11)print}' >> combination.txt
done

wc=`wc -l combination.txt | awk '{print $1}'`
head=`head -n11 1.pcd | sed -e "s/WIDTH .../WIDTH $wc/" -e "s/POINTS .../POINTS $wc/"`

echo "$head" | cat - combination.txt > combination.pcd
cd -
