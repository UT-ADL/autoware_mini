#!/usr/bin/env python3

import matplotlib.pyplot as plt
import os
import argparse
import csv
import rosbag
import time

def generate_autonomy_stack_plots(played_bag_file, values_history, monitoring_config, plots_dir):
    plots_dir = os.path.join(plots_dir, 'autonomy_stack')
    os.makedirs(plots_dir, exist_ok=True)
    
    info_text = "Generated from playing bag file: " + played_bag_file
    
    # Clean up previous .png files in the plots directory
    for fname in os.listdir(plots_dir):
        if fname.endswith('.png'):
            fpath = os.path.join(plots_dir, fname)
            os.remove(fpath)

    for component, history in values_history.items():
        freq_values = history['freq']
        delay_values = history['delay']
        if not freq_values or not delay_values:
            continue  # Skip if no data

        # Unpack values and timestamps
        freq, freq_t = zip(*freq_values)
        delay, delay_t = zip(*delay_values)

        # Use the earliest timestamp as t0 for both plots
        t0 = min(freq_t[0], delay_t[0])
        freq_time = [t - t0 for t in freq_t]
        delay_time = [t - t0 for t in delay_t]

        fig, axs = plt.subplots(2, 1, figsize=(10, 7))
        fig.suptitle(f"{component}", fontsize=16)

        # Frequency plot
        axs[0].plot(freq_time, freq, color='blue', label='Frequency (Hz)')
        # Draw warning and error thresholds if available
        config = None
        for topic, conf in monitoring_config.items():
            if conf['component'] == component:
                config = conf
                break
        if config:
            axs[0].axhline(y=config['warning_freq'], color='orange', linestyle='--', linewidth=2.5, label='Warning threshold')
            axs[0].axhline(y=config['error_freq'], color='red', linestyle='--', linewidth=2.5, label='Error threshold')
            # Mark threshold values on y axis
            center_x = 0.5 * (freq_time[0] + freq_time[-1])
            axs[0].text(
                center_x, config['warning_freq'], f"{config['warning_freq']}", 
                color='orange', va='center', ha='center', fontsize=8, fontweight="bold", 
                bbox=dict(facecolor='white', edgecolor='none', boxstyle='round,pad=0.1'),
            )
            axs[0].text(
                center_x, config['error_freq'], f"{config['error_freq']}", 
                color='red', va='center', ha='center', fontsize=8, fontweight="bold", 
                bbox=dict(facecolor='white', edgecolor='none', boxstyle='round,pad=0.1'),
            )
        axs[0].set_title("Frequency over time")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Frequency (Hz)")
        axs[0].grid(True)
        axs[0].legend(loc='upper left')
        axs[0].set_xlim(left=0, right=max(freq_time))

        # Delay plot
        axs[1].plot(delay_time, delay, color='blue', label='Delay (s)')
        if config:
            axs[1].axhline(y=config['warning_delay'], color='orange', linestyle='--', linewidth=2.5, label='Warning threshold')
            axs[1].axhline(y=config['error_delay'], color='red', linestyle='--', linewidth=2.5, label='Error threshold')
            center_x = 0.5 * (freq_time[0] + freq_time[-1])
            axs[1].text(
                center_x, config['warning_delay'], f"{config['warning_delay']}", 
                color='orange', va='center', ha='center', fontsize=8, fontweight="bold", 
                bbox=dict(facecolor='white', edgecolor='none', boxstyle='round,pad=0.1'),
            )
            axs[1].text(
                center_x, config['error_delay'], f"{config['error_delay']}", 
                color='red', va='center', ha='center', fontsize=8, fontweight="bold", 
                bbox=dict(facecolor='white', edgecolor='none', boxstyle='round,pad=0.1'),
            )
        axs[1].set_title("Delay over time")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Delay (s)")
        axs[1].grid(True)
        axs[1].legend(loc='upper left')
        axs[1].set_xlim(left=0, right=max(delay_time))
        
        plt.figtext(0.99, 0.01, s=info_text, fontsize=9, color='gray', ha='right', va='bottom')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plot_path = os.path.join(plots_dir, f"{component.lower().replace(' ', '_')}.png")
        plt.savefig(plot_path)
        plt.close(fig)

def generate_hardware_plots(played_bag_file, hardware_components_history, hardware_components_config, plots_dir):
    plots_dir = os.path.join(plots_dir, 'hardware')
    os.makedirs(plots_dir, exist_ok=True)
    
    info_text = "Generated from playing bag file: " + played_bag_file
    
    # Clean up previous .png files in the plots directory (no glob)
    for fname in os.listdir(plots_dir):
        if fname.endswith('.png'):
            fpath = os.path.join(plots_dir, fname)
            os.remove(fpath)

    for component, values in hardware_components_history.items():
        if not values or len(values) < 5:
            continue  # Skip if not enough data

        # Unpack values and timestamps
        prcnt, prcnt_t = zip(*values)

        # Use the earliest timestamp as t0 for both plots
        t0 = prcnt_t[0]
        prcnt_time = [t - t0 for t in prcnt_t]

        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(prcnt_time, prcnt, color='blue', linewidth=2, label='Usage (%)')
        ax.set_title(f"{component.split(' ')[0]} usage over time", fontsize=12)
        ax.set_xlabel("Time (s)", fontsize=11)
        ax.set_ylabel("Usage (%)", fontsize=11)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.set_ylim(0, 100)
        ax.set_xlim(left=0, right=max(prcnt_time))

        # Draw warning and error thresholds if available
        warning = hardware_components_config[component].get('warning_prcnt')
        error = hardware_components_config[component].get('error_prcnt')
        center_x = 0.5 * (prcnt_time[0] + prcnt_time[-1])
        if warning is not None:
            ax.axhline(y=warning, color='orange', linestyle='--', linewidth=2.5, label='Warning threshold')
            ax.text(
                center_x, hardware_components_config[component]['warning_prcnt'], f"{hardware_components_config[component]['warning_prcnt']}", 
                color='orange', va='center', ha='center', fontsize=8, fontweight="bold", 
                bbox=dict(facecolor='white', edgecolor='none', boxstyle='round,pad=0.1'),
            )
        if error is not None:
            ax.axhline(y=error, color='red', linestyle='--', linewidth=2.5, label='Error threshold')
            ax.text(
                center_x, hardware_components_config[component]['error_prcnt'], f"{hardware_components_config[component]['error_prcnt']}", 
                color='red', va='center', ha='center', fontsize=8, fontweight="bold", 
                bbox=dict(facecolor='white', edgecolor='none', boxstyle='round,pad=0.1'),
            )

        ax.legend(loc='upper left')
        plt.subplots_adjust(bottom=0.16)
        plt.figtext(0.89, 0.01, s=info_text, fontsize=9, color='gray', ha='right', va='bottom')

        
        # Save the plot
        plot_path = os.path.join(plots_dir, f"{component.lower().replace(' ', '_')}.png")
        plt.savefig(plot_path)
        plt.close(fig)

def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate autonomy stack plots from a ROS bag file."
    )
    parser.add_argument(
        "bag_file_name",
        type=str,
        help="""Name of the bag file (e.g. "some_ride.bag")"""
    )
    parser.add_argument(
        "--bag_folder",
        type=str,
        default=os.path.expanduser("~/autoware_mini_ws/src/autoware_mini/data/bags"),
        help="Folder containing the bag file (default: %(default)s)"
    )
    parser.add_argument(
        "--target_folder",
        type=str,
        default=os.path.expanduser("~/autoware_mini_ws/src/autoware_mini/data/monitoring/plots"),
        help="Root folder to save the generated plots (default: %(default)s)"
    )
    return parser.parse_args()

def extract_from_bag(bag_path):
    """
    Extracts values_history and monitoring_config from a bag file.
    Assumes that the bag contains messages of type diagnostic_msgs/DiagnosticArray
    on the /diagnostics topic, and the monitoring config is stored in ~/autoware_mini_ws/src/autoware_mini/config/monitoring/lexus.csv
    """
        
    values_history = {}
    monitoring_config = {}
    
    hardware_components_history = {}
    hardware_components_config = {}

    # Load monitoring configuration from CSV file (only autonomy_stack)
    config_csv = os.path.expanduser("~/autoware_mini_ws/src/autoware_mini/config/monitoring/lexus.csv")
    if os.path.isfile(config_csv):
        with open(config_csv, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                typ = row.get("type", "").strip()
                component = row["component"].strip()
                if typ == "autonomy_stack":
                    topic = row["topic"].strip()
                    values_history[component] = {"freq": [], "delay": []}
                    monitoring_config[topic] = {
                        "component": row["component"].strip(),
                        "warning_freq": float(row["warning_freq"]),
                        "error_freq": float(row["error_freq"]),
                        "warning_delay": float(row["warning_delay"]),
                        "error_delay": float(row["error_delay"]),
                    }
                elif typ == "hardware":
                    hardware_components_history[component] = []
                    hardware_components_config[component] = {
                        "warning_prcnt": float(row["warning_prcnt"]) if row["warning_prcnt"] else None,
                        "error_prcnt": float(row["error_prcnt"]) if row["error_prcnt"] else None,
                    }

    # Extract frequency and delay from diagnostics messages
    with rosbag.Bag(bag_path) as bag:
        
        total_msgs = bag.get_message_count(topic_filters=["/diagnostics"])
        processed = 0
        prev_prcnt = -1
        
        for topic, msg, t in bag.read_messages():
            if topic == "/diagnostics":
                processed += 1
                percent = int((processed / total_msgs) * 100) if total_msgs else 100
                if percent != prev_prcnt:
                    print(f"Progress: {percent}% ({processed}/{total_msgs})", end='\r', flush=True)
                    prev_prcnt = percent

                message = msg.status[0]
                comp = message.name
                        
                # Autonomy stack
                is_autonomy_stack = False
                for topic, conf in monitoring_config.items():
                    if conf["component"] == comp:
                        is_autonomy_stack = True
                        break
                if is_autonomy_stack:
                    freq = None
                    delay = None
                    for kv in message.values:
                        if kv.key == "Frequency (Hz)":
                            freq = float(kv.value)
                        if kv.key == "Delay (s)":
                            delay = float(kv.value)
                    timestamp = msg.header.stamp.to_sec()
                    values_history[comp]["freq"].append((freq, timestamp))
                    values_history[comp]["delay"].append((delay, timestamp))
                # Hardware
                elif "monitor" in comp:
                    received_comp_name = message.name.split("_")[0]
                    comp = None                
                    for key in hardware_components_config.keys():
                        if key.split(" ")[0].lower() == received_comp_name:
                            comp = key
                            break
                    if comp in hardware_components_history.keys():
                        for kv in message.values:
                            if "Load Average" in kv.key:
                                usage = float(kv.value)
                        timestamp = msg.header.stamp.to_sec()
                        hardware_components_history[comp].append((usage, timestamp))
            
        print(f"\nExtraction complete. Processed {processed} / {total_msgs} messages.")
    return values_history, monitoring_config, hardware_components_history, hardware_components_config

if __name__ == "__main__":
    start = time.time()
    
    args = parse_args()
    bag_path = os.path.join(args.bag_folder, args.bag_file_name)
    if not os.path.isfile(bag_path):
        print(f"Bag file not found: {bag_path}")
        exit(1)
    
    print("-"*50)
    print(f"Extracting data from bag file: {bag_path}")
        
    values_history, monitoring_config, hardware_components_history, hardware_components_config = extract_from_bag(bag_path)
    
    # Make the target folder first
    os.makedirs(args.target_folder, exist_ok=True)
    
    # Add bag file name (without .bag) to target folder path
    joined_target_folder = os.path.join(args.target_folder, args.bag_file_name.rsplit(".", 1)[0])
    os.makedirs(joined_target_folder, exist_ok=True)
    
    # Generate autonomy stack plots
    generate_autonomy_stack_plots(
        played_bag_file=args.bag_file_name,
        values_history=values_history,
        monitoring_config=monitoring_config,
        plots_dir=joined_target_folder
    )
    # Generate hardware plots
    generate_hardware_plots(
        played_bag_file=args.bag_file_name,
        hardware_components_history=hardware_components_history,
        hardware_components_config=hardware_components_config,
        plots_dir=joined_target_folder
    )
    
    abs_plots_dir = os.path.abspath(joined_target_folder)
    print(f"Plots generated in: {abs_plots_dir}")
    print(f"Time spent: {time.time() - start:.2f} seconds")
    print("-"*50)