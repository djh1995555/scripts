#!/bin/bash

pids=($(pgrep -f "controller"))

for pid in "${pids[@]}"
do
    echo "Killing process with PID: $pid"
    kill $pid
done

echo "All processes with 'controller' in their name have been killed."