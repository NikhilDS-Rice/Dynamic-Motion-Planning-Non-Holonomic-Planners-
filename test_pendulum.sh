#!/bin/bash
# Test script for Pendulum

echo "========================================="
echo "Testing Pendulum with RRT, torque=3"
echo "========================================="
printf "1\n1\n1\n" | timeout 60 ./Project4Pendulum

echo ""
echo "========================================="
echo "Testing Pendulum with KPIECE1, torque=3"
echo "========================================="
printf "1\n1\n2\n" | timeout 60 ./Project4Pendulum

echo ""
echo "========================================="
echo "Testing Pendulum with RRT, torque=5"
echo "========================================="
printf "1\n2\n1\n" | timeout 60 ./Project4Pendulum

echo ""
echo "Files created:"
ls -lh *.txt 2>/dev/null || echo "No output files yet"
