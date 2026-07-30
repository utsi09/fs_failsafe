#!/bin/bash
# Run the leaderboard evaluator. KEY=VALUE args anywhere become env vars.
# usage: ./run_leaderboard.sh              (normal run, no fault injection)
#        ./run_leaderboard.sh cf=20        (destroy camera 20 s in — frozen image)
#        ./run_leaderboard.sh cn=20:15     (camera gaussian noise sigma=15 from 20 s)
#        ./run_leaderboard.sh ln=20:0.1    (BOTH lidars, noise sigma=0.1 m from 20 s)
#        ./run_leaderboard.sh lnf=20:0.3   (front lidar only)
#        ./run_leaderboard.sh lnr=20:0.3   (rear lidar only)
#        ./run_leaderboard.sh imn=20:0.05  (IMU: gyro sigma rad/s, accel 10x m/s^2)
#        ./run_leaderboard.sh gn=20:2.0    (EKF input pose noise, sigma meters)
cd "$(dirname "$0")"
for a in "$@"; do
  case "$a" in
    [A-Za-z_]*=*) export "$a" ;;
    *) echo "unknown arg: $a" >&2; exit 1 ;;
  esac
done
exec python3 src/external/leaderboard/leaderboard/leaderboard_evaluator.py --conf_file_path=config/config.yaml
