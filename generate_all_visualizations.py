#!/usr/bin/env python3
"""
Generate all visualizations for Project 4 Checkpoint
"""
import subprocess
import os

print("=" * 60)
print("Project 4 - Visualization Generator")
print("=" * 60)

files_to_visualize = [
    ("pendulum_path.txt", "visualize_pendulum.py"),
    ("car_path.txt", "visualize_car.py")
]

for data_file, viz_script in files_to_visualize:
    if os.path.exists(data_file):
        print(f"\nGenerating visualizations for {data_file}...")
        try:
            subprocess.run(["python", viz_script, data_file], check=True)
            print(f"✓ Successfully generated visualizations for {data_file}")
        except Exception as e:
            print(f"✗ Error generating visualizations: {e}")
    else:
        print(f"✗ File not found: {data_file}")

print("\n" + "=" * 60)
print("Generated Visualizations:")
print("=" * 60)

# List all generated PNG files
png_files = [f for f in os.listdir('.') if f.endswith('.png')]
if png_files:
    for png in sorted(png_files):
        size = os.path.getsize(png) / 1024  # KB
        print(f"  ✓ {png:40s} ({size:8.1f} KB)")
else:
    print("  No PNG files found")

print("\n" + "=" * 60)
print("Visualization complete!")
print("=" * 60)
