#!/bin/bash
# Run all 7 butler delivery scenarios sequentially for screen recording
# Usage: bash run_all_scenarios.sh

set -e
source /opt/ros/humble/setup.bash
cd /home/praju/bumperbot_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle

echo "============================================"
echo "  BUTLER ROBOT - ALL SCENARIOS RUNNER"
echo "============================================"
echo ""

for SCENARIO in 1 2 3 4 5 6 7; do
    echo ""
    echo "============================================"
    echo "  STARTING SCENARIO $SCENARIO"
    echo "============================================"
    echo ""
    sleep 3

    ros2 run butler_delivery scenario_runner "$SCENARIO" 2>&1

    echo ""
    echo "============================================"
    echo "  SCENARIO $SCENARIO FINISHED"
    echo "============================================"
    echo ""
    echo "Next scenario starts in 5 seconds..."
    sleep 5
done

echo ""
echo "============================================"
echo "  ALL 7 SCENARIOS COMPLETE!"
echo "============================================"
