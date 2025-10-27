#!/bin/bash
# Test script for Car

echo "========================================="
echo "Testing Car with RRT"
echo "========================================="
printf "1\n1\n" | timeout 90 ./Project4Car

echo ""
echo "========================================="
echo "Testing Car with KPIECE1"
echo "========================================="
printf "1\n2\n" | timeout 90 ./Project4Car

echo ""
echo "Files created:"
ls -lh *.txt 2>/dev/null || echo "No output files yet"
