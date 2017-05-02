#! /bin/bash
dt=$(date '+%d-%m-%Y_%H:%M:%S');
raspistill -t 1 -o photos/"$dt".jpg
