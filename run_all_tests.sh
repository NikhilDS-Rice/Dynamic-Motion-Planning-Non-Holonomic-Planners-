#!/bin/bash
# Comprehensive test script for all planners and configurations

echo "========================================="
echo "COMPREHENSIVE PROJECT 4 TEST"
echo "========================================="
echo ""

# Clean output directory
mkdir -p output
rm -f output/*.txt

echo "========================================="
echo "PENDULUM TESTS"
echo "========================================="

echo ""
echo "1. Pendulum with RRT, torque=3"
printf "1\n1\n1\n" | timeout 60 ./Project4Pendulum

echo ""
echo "2. Pendulum with KPIECE1, torque=3"
printf "1\n1\n2\n" | timeout 60 ./Project4Pendulum

echo ""
echo "3. Pendulum with RRT, torque=5"
printf "1\n2\n1\n" | timeout 60 ./Project4Pendulum

echo ""
echo "4. Pendulum with RRT, torque=10"
printf "1\n3\n1\n" | timeout 60 ./Project4Pendulum

echo ""
echo "========================================="
echo "CAR TESTS"
echo "========================================="

echo ""
echo "5. Car with RRT"
printf "1\n1\n" | timeout 90 ./Project4Car

echo ""
echo "6. Car with KPIECE1"
printf "1\n2\n" | timeout 90 ./Project4Car

echo ""
echo "========================================="
echo "TEST COMPLETE - Generated Files:"
echo "========================================="
ls -lh output/*.txt 2>/dev/null || echo "No output files generated"
