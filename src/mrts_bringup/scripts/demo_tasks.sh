#!/usr/bin/env bash

submit() {
  echo "Task: pickup ($1, $2) -> dropoff ($3, $4)"
  ros2 service call /fleet_manager/submit_task mrts_interfaces/srv/SubmitTask \
    "{pickup_x: $1, pickup_y: $2, dropoff_x: $3, dropoff_y: $4}" | grep -A1 "response:"
}

submit 2.0  1.0   3.81 -0.24
submit 1.34 2.53  2.61 -1.52
# Tasks 3 and 4 wait in the queue until a robot is Idle again
submit 3.81 -0.24 1.34  2.53
submit 2.61 -1.52 2.0   1.0
