#!/bin/bash
 
# Function to handle the interrupt signal
cleanup() {
  echo "Script interrupted"
  exit 1
}
 
# Trap SIGINT (Ctrl+C) and call the cleanup function
trap cleanup SIGINT
 
for i in {1..100}
do
  rosrun mppi_planner ros_jackalsimulator.py
done