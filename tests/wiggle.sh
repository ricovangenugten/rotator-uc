#!/bin/bash

for i in `seq 1 1000000`;
do
echo $1 > /dev/ttyUSB0
sleep 5
echo $2 > /dev/ttyUSB0
sleep 5
done

