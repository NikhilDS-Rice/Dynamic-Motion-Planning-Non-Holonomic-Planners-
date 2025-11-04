#!/usr/bin/env python3
"""
Analyze Car benchmark results
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
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
                            tree_nodes = int(parts[12])
                            
                            results[planner_name]['solved'].append(solved)
                            results[planner_name]['times'].append(time)
                            results[planner_name]['states'].append(states)
                            results[planner_name]['paths'].append(path_length)
                            results[planner_name]['tree_nodes'].append(tree_nodes)

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
    
    fig = plt.figure(figsize=(15, 8))
    fig.suptitle('Car Benchmark Results', fontsize=16, fontweight='bold')
    gs = GridSpec(2, 3, figure=fig, height_ratios=[1, 1])
    
    ax_time   = fig.add_subplot(gs[0, 0])
    ax_succ   = fig.add_subplot(gs[0, 1])
    ax_states = fig.add_subplot(gs[0, 2])
    ax_path   = fig.add_subplot(gs[1, 0:2])
    ax_nodes  = fig.add_subplot(gs[1, 2])
    
    # 1. Box plot of solve times
    data_to_plot, labels = [], []
    for p in planners:
        if p in results and results[p].get('times'):
            data_to_plot.append(results[p]['times'])
            labels.append(p)
    if data_to_plot:
        bp = ax_time.boxplot(data_to_plot, tick_labels=labels, patch_artist=True)
        for patch, p in zip(bp['boxes'], labels):
            patch.set_facecolor(colors[p]); patch.set_alpha(0.7)
    ax_time.set_ylabel('Solve Time (s)', fontsize=11)
    ax_time.set_title('Solve Time Distribution', fontsize=12, fontweight='bold')
    ax_time.set_yscale('log')
    ax_time.grid(True, alpha=0.3, axis='y')
    
    # 2. Bar chart of success rates
    success_rates = []
    for p in planners:
        sr = results.get(p, {}).get('success_rate', None)
        success_rates.append(100.0 * float(sr) if sr is not None else 0.0)
    bars = ax_succ.bar(planners, success_rates, color=[colors[p] for p in planners], alpha=0.85)
    ax_succ.set_ylabel('Success Rate (%)', fontsize=11)
    ax_succ.set_title('Success Rate', fontsize=12, fontweight='bold')
    ax_succ.set_ylim([0, 105])
    ax_succ.grid(True, alpha=0.3, axis='y')
    for bar, rate in zip(bars, success_rates):
        ax_succ.text(bar.get_x() + bar.get_width()/2., bar.get_height(),
                     f'{rate:.1f}%', ha='center', va='bottom', fontsize=10)
    
    
    # 3. Box plot of graph states
    data_to_plot, labels = [], []
    for p in planners:
        if p in results and results[p].get('states'):
            data_to_plot.append(results[p]['states'])
            labels.append(p)
    if data_to_plot:
        bp = ax_states.boxplot(data_to_plot, tick_labels=labels, patch_artist=True)
        for patch, p in zip(bp['boxes'], labels):
            patch.set_facecolor(colors[p]); patch.set_alpha(0.7)
    ax_states.set_ylabel('Graph States', fontsize=11)
    ax_states.set_yscale('log')
    ax_states.set_title('Graph States Distribution', fontsize=12, fontweight='bold')
    ax_states.grid(True, alpha=0.3, axis='y')
    
    # 4. Box plot of path lengths
    data_to_plot, labels = [], []
    for p in planners:
        if p in results and results[p].get('paths'):
            data_to_plot.append(results[p]['paths'])
            labels.append(p)
    if data_to_plot:
        bp = ax_path.boxplot(data_to_plot, tick_labels=labels, patch_artist=True)
        for patch, p in zip(bp['boxes'], labels):
            patch.set_facecolor(colors[p]); patch.set_alpha(0.7)
    ax_path.set_ylabel('Path Length', fontsize=11)
    ax_path.set_title('Path Length Distribution', fontsize=12, fontweight='bold')
    ax_path.grid(True, alpha=0.3, axis='y')

    # 5 Tree Nodes boxplot
    data_to_plot, labels = [], []
    for p in planners:
        if p in results and results[p].get('tree_nodes'):
            data_to_plot.append(results[p]['tree_nodes'])
            labels.append(p)
    if data_to_plot:
        bp = ax_nodes.boxplot(data_to_plot, tick_labels=labels, patch_artist=True)
        for patch, p in zip(bp['boxes'], labels):
            patch.set_facecolor(colors[p]); patch.set_alpha(0.7)
        ax_nodes.set_ylabel('Tree Nodes', fontsize=11)
        ax_nodes.set_yscale('log')
        ax_nodes.set_title('Tree Nodes Distribution', fontsize=12, fontweight='bold')
        ax_nodes.grid(True, alpha=0.3, axis='y')
    else:
        # If no tree_nodes present, show a friendly message
        ax_nodes.axis('off')
        ax_nodes.text(0.5, 0.5, 'No tree_nodes\nin log', ha='center', va='center', fontsize=12)

    plt.tight_layout()
    output_file = 'benchmark_results/car_benchmark_results.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\nSaved plot: {output_file}")
    plt.close()

def clip_axis(ax, data):
    all_vals = np.concatenate(data)
    ymax = np.percentile(all_vals, 95) * 1.2
    ax.set_ylim(0, ymax)

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
