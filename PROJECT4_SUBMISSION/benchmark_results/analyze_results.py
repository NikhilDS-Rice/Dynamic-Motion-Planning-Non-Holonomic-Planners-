import re
import sys

if len(sys.argv) < 2:
    print("Usage: python analyze_results.py <log_file>")
    sys.exit(1)

log_file = sys.argv[1]

with open(log_file, 'r') as f:
    content = f.read()

# Find all planner sections
planner_sections = re.findall(r'control_(\w+)\n.*?\n(\d+) runs\n(.*?)\n\.', content, re.DOTALL)

import os
log_filename = os.path.basename(log_file)

print("\n" + "="*70)
print(f"BENCHMARK RESULTS ANALYSIS: {log_filename}")
print("="*70 + "\n")

for planner_name, num_runs, data_section in planner_sections:
    lines = [line.strip() for line in data_section.split('\n') if line.strip() and ';' in line]
    
    total_runs = len(lines)
    successful_runs = []
    failed_runs = []
    
    for line in lines:
        parts = line.split(';')
        solved = int(parts[9].strip())
        time = float(parts[11].strip())
        states = int(parts[3].strip())
        path_length = float(parts[7].strip()) if solved else 0
        
        if solved == 1:
            successful_runs.append({
                'time': time,
                'states': states,
                'path': path_length
            })
        else:
            failed_runs.append({
                'time': time,
                'states': states
            })
    
    success_rate = len(successful_runs) / total_runs * 100
    
    print(f"{planner_name}:")
    print(f"  Success Rate: {len(successful_runs)}/{total_runs} = {success_rate:.1f}%")
    
    if successful_runs:
        avg_time = sum(r['time'] for r in successful_runs) / len(successful_runs)
        avg_states = sum(r['states'] for r in successful_runs) / len(successful_runs)
        avg_path = sum(r['path'] for r in successful_runs) / len(successful_runs)
        min_time = min(r['time'] for r in successful_runs)
        max_time = max(r['time'] for r in successful_runs)
        
        print(f"  Solve Time: {avg_time:.4f} ± {(max_time - min_time)/2:.4f} s")
        print(f"              (min: {min_time:.4f}, max: {max_time:.4f})")
        print(f"  Path Length: {avg_path:.2f}")
        print(f"  Graph States: {avg_states:.0f}")
    
    if failed_runs:
        avg_fail_time = sum(r['time'] for r in failed_runs) / len(failed_runs)
        avg_fail_states = sum(r['states'] for r in failed_runs) / len(failed_runs)
        print(f"  Failed runs: {len(failed_runs)}")
        print(f"    Avg timeout: {avg_fail_time:.2f}s")
        print(f"    Avg states explored: {avg_fail_states:.0f}")
    
    print()

print("="*70)
