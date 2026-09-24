#!/usr/bin/env bash

set -u

PROJECT_DIR="$HOME/22SmartCar/M3/M3-1"
START_RUN=3
END_RUN=10

cd "$PROJECT_DIR" || exit 1

# ROS 2 / colcon 的 setup 脚本可能访问尚未定义的环境变量。
# source 时暂时关闭 nounset，完成后再恢复。
set +u
source install/setup.bash
set -u

cleanup_gazebo() {
    echo "Stopping Gazebo..."
    killall -9 gzserver 2>/dev/null || true
    killall -9 gzclient 2>/dev/null || true
    sleep 2
}

cleanup_all() {
    echo
    echo "Experiment script interrupted."
    cleanup_gazebo
}

trap cleanup_all INT TERM

for RUN in $(seq "$START_RUN" "$END_RUN"); do
    printf -v RUN_PADDED "%02d" "$RUN"

    echo
    echo "========================================"
    echo "Starting run ${RUN_PADDED}"
    echo "========================================"

    # 每轮开始前确保没有旧 Gazebo 残留。
    cleanup_gazebo

    echo "Launching Gazebo..."
    ros2 launch smart_car_description sim.launch.py \
        > "/tmp/m3_1_gazebo_run_${RUN_PADDED}.log" 2>&1 &
    GAZEBO_LAUNCH_PID=$!

    echo "Waiting for Gazebo and controllers..."

    READY=0

    for _ in $(seq 1 60); do
        if ! kill -0 "$GAZEBO_LAUNCH_PID" 2>/dev/null; then
            echo "ERROR: Gazebo launch exited unexpectedly."
            echo "Check /tmp/m3_1_gazebo_run_${RUN_PADDED}.log"
            cleanup_gazebo
            exit 1
        fi

        if ros2 topic info /odom 2>/dev/null | grep -q "Publisher count: 1" &&
           ros2 topic info /gazebo/model_states 2>/dev/null | grep -q "Publisher count: 1"; then
            READY=1
            break
        fi

        sleep 1
    done

    if [ "$READY" -ne 1 ]; then
        echo "ERROR: Gazebo/controllers were not ready within 60 seconds."
        echo "Check /tmp/m3_1_gazebo_run_${RUN_PADDED}.log"
        cleanup_gazebo
        exit 1
    fi

    # 再留一点时间，让车辆落地并稳定。
    sleep 2

    echo "Starting recorder for run ${RUN_PADDED}..."

    python3 scripts/record_traj.py \
        --out logs \
        --run "$RUN" &
    RECORDER_PID=$!

    # 让 recorder 完成订阅初始化。
    sleep 2

    echo "Starting square driver..."

    python3 scripts/square_driver.py \
        --side 2.0 \
        --laps 1 &
    DRIVER_PID=$!

    echo "Waiting for recorder to finish..."

    wait "$RECORDER_PID"
    RECORDER_STATUS=$?

    if [ "$RECORDER_STATUS" -ne 0 ]; then
        echo "ERROR: Recorder failed during run ${RUN_PADDED}."
        kill "$DRIVER_PID" 2>/dev/null || true
        cleanup_gazebo
        exit 1
    fi

    # recorder 收到 STOP 并退出后，driver 已经完成实验，
    # 但 driver 本身仍停留在 STOP 状态，所以主动结束它。
    kill -INT "$DRIVER_PID" 2>/dev/null || true
    wait "$DRIVER_PID" 2>/dev/null || true

    ODOM_FILE="logs/run_${RUN_PADDED}_odom.csv"
    TRUTH_FILE="logs/run_${RUN_PADDED}_truth.csv"

    if [ ! -s "$ODOM_FILE" ] || [ ! -s "$TRUTH_FILE" ]; then
        echo "ERROR: Output CSV missing for run ${RUN_PADDED}."
        cleanup_gazebo
        exit 1
    fi

    ODOM_LINES=$(wc -l < "$ODOM_FILE")
    TRUTH_LINES=$(wc -l < "$TRUTH_FILE")

    echo "Run ${RUN_PADDED} complete."
    echo "  odom : ${ODOM_LINES} lines"
    echo "  truth: ${TRUTH_LINES} lines"

    cleanup_gazebo
done

trap - INT TERM

echo
echo "========================================"
echo "Runs 03-10 completed."
echo "========================================"

echo
echo "Generated files:"
ls -lh logs/run_{03..10}_odom.csv logs/run_{03..10}_truth.csv
