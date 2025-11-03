#!/usr/bin/env python3
"""
Analyze Car benchmark results
"""
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import os

def parse_benchmark_log(filename):
    """Parse OMPL benchmark log file and extract statistics"""
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    results = defaultdict(lambda: defaultdict(list))
    
    # Find planner sections
    planner_names = {
        'control_RRT': 'RRT',
        'control_KPIECE1': 'KPIECE1', 
        'control_RGRRT': 'RGRRT'
    }
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # Check if this is a planner section
        if line in planner_names:
            planner_name = planner_names[line]
            
            # Skip to data rows (after the run count line)
            i += 1
            while i < len(lines) and not lines[i].strip().endswith('runs'):
                i += 1
            i += 1  # Move past "X runs" line
            
            # Read data rows until we hit a '.' or next planner
            while i < len(lines):
                data_line = lines[i].strip()
                if data_line == '.' or data_line in planner_names:
                    break
                
                if data_line and ';' in data_line:
                    # Parse CSV format
                    parts = [p.strip() for p in data_line.split(';')]
                    if len(parts) >= 12:
                        try:
                            solved = int(parts[9])
                            time = float(parts[11])
                            states = int(parts[3])
                            path_length = float(parts[7])
                            
                            results[planner_name]['solved'].append(solved)
                            results[planner_name]['times'].append(time)
                            results[planner_name]['states'].append(states)
                            results[planner_name]['paths'].append(path_length)
                        except (ValueError, IndexError):
                            pass
                
                i += 1
            
            # Calculate success rate
            if results[planner_name]['solved']:
                results[planner_name]['success_rate'] = sum(results[planner_name]['solved']) / len(results[planner_name]['solved'])
            else:
                results[planner_name]['success_rate'] = 0
        
        i += 1
    
    return results

def print_statistics(results):
    """Print summary statistics"""
    print(f"\n{'='*70}")
    print(f"CAR BENCHMARK RESULTS")
    print(f"{'='*70}\n")
    
    for planner in sorted(results.keys()):
        data = results[planner]
        print(f"{planner}:")
        print(f"  Success Rate: {data['success_rate']*100:.1f}%")
        
        if data['times']:
            times = np.array(data['times'])
            print(f"  Solve Time: {np.mean(times):.4f} ± {np.std(times):.4f} s")
            print(f"              (min: {np.min(times):.4f}, max: {np.max(times):.4f})")
        
        if data['paths']:
            paths = np.array(data['paths'])
            print(f"  Path Length: {np.mean(paths):.2f} ± {np.std(paths):.2f}")
        
        if data['states']:
            states = np.array(data['states'])
            print(f"  Graph States: {np.mean(states):.0f} ± {np.std(states):.0f}")
        print()

def create_car_plots(results):
    """Create visualization plots for car benchmark"""
    
    planners = ['RRT', 'KPIECE1', 'RGRRT']
    colors = {'RRT': '#1f77b4', 'KPIECE1': '#ff7f0e', 'RGRRT': '#2ca02c'}
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Car Benchmark Results', fontsize=16, fontweight='bold')
    
    # 1. Box plot of solve times
    ax = axes[0, 0]
    data_to_plot = []
    labels = []
    for planner in planners:
        if planner in results and results[planner]['times']:
            data_to_plot.append(results[planner]['times'])
            labels.append(planner)
    
    if data_to_plot:
        bp = ax.boxplot(data_to_plot, labels=labels, patch_artist=True)
        for patch, planner in zip(bp['boxes'], labels):
            patch.set_facecolor(colors[planner])
            patch.set_alpha(0.7)
    
    ax.set_ylabel('Solve Time (s)', fontsize=11)
    ax.set_title('Solve Time Distribution', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # 2. Bar chart of success rates
    ax = axes[0, 1]
    success_rates = []
    planner_labels = []
    for planner in planners:
        if planner in results:
            success_rates.append(results[planner]['success_rate'] * 100)
            planner_labels.append(planner)
    
    bars = ax.bar(planner_labels, success_rates, color=[colors[p] for p in planner_labels], alpha=0.8)
    ax.set_ylabel('Success Rate (%)', fontsize=11)
    ax.set_title('Success Rate', fontsize=12, fontweight='bold')
    ax.set_ylim([0, 105])
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add percentage labels on bars
    for bar, rate in zip(bars, success_rates):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{rate:.1f}%', ha='center', va='bottom', fontsize=10)
    
    # 3. Box plot of graph states
    ax = axes[1, 0]
    data_to_plot = []
    labels = []
    for planner in planners:
        if planner in results and results[planner]['states']:
            data_to_plot.append(results[planner]['states'])
            labels.append(planner)
    
    if data_to_plot:
        bp = ax.boxplot(data_to_plot, labels=labels, patch_artist=True)
        for patch, planner in zip(bp['boxes'], labels):
            patch.set_facecolor(colors[planner])
            patch.set_alpha(0.7)
    
    ax.set_ylabel('Graph States', fontsize=11)
    ax.set_title('Graph States Distribution', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # 4. Box plot of path lengths
    ax = axes[1, 1]
    data_to_plot = []
    labels = []
    for planner in planners:
        if planner in results and results[planner]['paths']:
            data_to_plot.append(results[planner]['paths'])
            labels.append(planner)
    
    if data_to_plot:
        bp = ax.boxplot(data_to_plot, labels=labels, patch_artist=True)
        for patch, planner in zip(bp['boxes'], labels):
            patch.set_facecolor(colors[planner])
            patch.set_alpha(0.7)
    
    ax.set_ylabel('Path Length', fontsize=11)
    ax.set_title('Path Length Distribution', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    output_file = 'benchmark_results/car_benchmark_results.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\nSaved plot: {output_file}")
    plt.close()

def main():
    """Main analysis function"""
    
    filename = 'benchmark_results/car_benchmark.log'
    if os.path.exists(filename):
        print(f"Parsing {filename}...")
        results = parse_benchmark_log(filename)
        print_statistics(results)
        create_car_plots(results)
    else:
        print(f"Error: {filename} not found")
    
    print("\n" + "="*70)
    print("Car benchmark analysis complete!")
    print("="*70)

if __name__ == '__main__':
    main()
