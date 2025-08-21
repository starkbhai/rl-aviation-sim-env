#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import json
import time
import os
from datetime import datetime
import atexit

class FlightDataLogger:
    """
    Flight data logger that collects flight parameters during training
    and generates graphs after program completion
    """
    
    def __init__(self, log_dir="flight_logs", save_interval=100):
        """
        Initialize flight data logger
        
        Args:
            log_dir (str): Directory to save log files
            save_interval (int): Save data every N steps
        """
        self.log_dir = log_dir
        self.save_interval = save_interval
        
        # Create log directory
        os.makedirs(log_dir, exist_ok=True)
        
        # Data storage
        self.flight_data = {
            'step': [],
            'timestamp': [],
            'altitude_m': [],
            'pitch_deg': [],
            'roll_deg': [],
            'heading_deg': [],
            'latitude': [],
            'longitude': [],
            'airspeed_kts': [],
            'reward': [],
            'episode': [],
            'total_reward': []
        }
        
        # Metadata
        self.session_start_time = time.time()
        self.current_episode = 0
        self.step_count = 0
        
        # File paths
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = os.path.join(log_dir, f"flight_data_{timestamp}.csv")
        self.json_file = os.path.join(log_dir, f"flight_metadata_{timestamp}.json")
        self.graph_file = os.path.join(log_dir, f"flight_graphs_{timestamp}.png")
        
        print(f"📊 Flight Data Logger initialized")
        print(f"📁 Log directory: {log_dir}")
        print(f"💾 Data will be saved to: {self.csv_file}")
        
        # Register cleanup function
        atexit.register(self.save_and_generate_graphs)
    
    def log_step(self, observations, reward, episode, total_reward=0):
        """
        Log a single step of flight data
        
        Args:
            observations (list): [altitude, pitch, roll, heading, lat, lon, airspeed]
            reward (float): Step reward
            episode (int): Current episode number
            total_reward (float): Total episode reward
        """
        self.step_count += 1
        self.current_episode = episode
        
        # Store data
        self.flight_data['step'].append(self.step_count)
        self.flight_data['timestamp'].append(time.time() - self.session_start_time)
        self.flight_data['altitude_m'].append(float(observations[0]))
        self.flight_data['pitch_deg'].append(float(observations[1]))
        self.flight_data['roll_deg'].append(float(observations[2]))
        self.flight_data['heading_deg'].append(float(observations[3]))
        self.flight_data['latitude'].append(float(observations[4]))
        self.flight_data['longitude'].append(float(observations[5]))
        self.flight_data['airspeed_kts'].append(float(observations[6]))
        self.flight_data['reward'].append(float(reward))
        self.flight_data['episode'].append(int(episode))
        self.flight_data['total_reward'].append(float(total_reward))
        
        # Periodic save
        if self.step_count % self.save_interval == 0:
            self._save_data_to_csv()
    
    def _save_data_to_csv(self):
        """Save current data to CSV file"""
        try:
            df = pd.DataFrame(self.flight_data)
            df.to_csv(self.csv_file, index=False)
        except Exception as e:
            print(f"[WARNING] Failed to save CSV: {e}")
    
    def save_metadata(self):
        """Save session metadata"""
        metadata = {
            'session_start': datetime.fromtimestamp(self.session_start_time).isoformat(),
            'session_duration_seconds': time.time() - self.session_start_time,
            'total_steps': self.step_count,
            'total_episodes': self.current_episode,
            'log_files': {
                'csv': self.csv_file,
                'graphs': self.graph_file
            }
        }
        
        try:
            with open(self.json_file, 'w') as f:
                json.dump(metadata, f, indent=2)
        except Exception as e:
            print(f"[WARNING] Failed to save metadata: {e}")
    
    def generate_flight_graphs(self):
        """Generate comprehensive flight data graphs"""
        if len(self.flight_data['step']) == 0:
            print("⚠️ No flight data to graph")
            return
        
        print(f"📊 Generating flight graphs...")
        
        # Convert to DataFrame for easier plotting
        df = pd.DataFrame(self.flight_data)
        
        # Create figure with subplots
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('Flight Data Analysis', fontsize=16, fontweight='bold')
        
        # Colors for plots
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        
        # 1. Altitude over time
        axes[0, 0].plot(df['timestamp'], df['altitude_m'], color=colors[0], linewidth=1.5)
        axes[0, 0].set_title('Altitude vs Time', fontweight='bold')
        axes[0, 0].set_xlabel('Time (seconds)')
        axes[0, 0].set_ylabel('Altitude (meters)')
        axes[0, 0].grid(True, alpha=0.3)
        
        # 2. Attitude (Pitch, Roll, Heading)
        axes[0, 1].plot(df['timestamp'], df['pitch_deg'], label='Pitch', color=colors[1], linewidth=1.2)
        axes[0, 1].plot(df['timestamp'], df['roll_deg'], label='Roll', color=colors[2], linewidth=1.2)
        axes[0, 1].set_title('Aircraft Attitude vs Time', fontweight='bold')
        axes[0, 1].set_xlabel('Time (seconds)')
        axes[0, 1].set_ylabel('Angle (degrees)')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # 3. Heading over time
        axes[1, 0].plot(df['timestamp'], df['heading_deg'], color=colors[3], linewidth=1.5)
        axes[1, 0].set_title('Heading vs Time', fontweight='bold')
        axes[1, 0].set_xlabel('Time (seconds)')
        axes[1, 0].set_ylabel('Heading (degrees)')
        axes[1, 0].grid(True, alpha=0.3)
        
        # 4. Airspeed over time
        axes[1, 1].plot(df['timestamp'], df['airspeed_kts'], color=colors[4], linewidth=1.5)
        axes[1, 1].set_title('Airspeed vs Time', fontweight='bold')
        axes[1, 1].set_xlabel('Time (seconds)')
        axes[1, 1].set_ylabel('Airspeed (knots)')
        axes[1, 1].grid(True, alpha=0.3)
        
        # 5. Reward over time
        axes[2, 0].plot(df['timestamp'], df['reward'], color=colors[5], linewidth=1, alpha=0.7)
        # Add moving average for clarity
        if len(df) > 10:
            window_size = min(50, len(df) // 10)
            moving_avg = df['reward'].rolling(window=window_size, center=True).mean()
            axes[2, 0].plot(df['timestamp'], moving_avg, color='red', linewidth=2, label=f'Moving Avg ({window_size})')
            axes[2, 0].legend()
        axes[2, 0].set_title('Reward vs Time', fontweight='bold')
        axes[2, 0].set_xlabel('Time (seconds)')
        axes[2, 0].set_ylabel('Reward')
        axes[2, 0].grid(True, alpha=0.3)
        
        # 6. Flight path (lat/lon)
        axes[2, 1].plot(df['longitude'], df['latitude'], color=colors[0], linewidth=1.5, alpha=0.8)
        axes[2, 1].scatter(df['longitude'].iloc[0], df['latitude'].iloc[0], 
                          color='green', s=100, label='Start', zorder=5)
        axes[2, 1].scatter(df['longitude'].iloc[-1], df['latitude'].iloc[-1], 
                          color='red', s=100, label='End', zorder=5)
        axes[2, 1].set_title('Flight Path (GPS)', fontweight='bold')
        axes[2, 1].set_xlabel('Longitude')
        axes[2, 1].set_ylabel('Latitude')
        axes[2, 1].legend()
        axes[2, 1].grid(True, alpha=0.3)
        
        # Adjust layout
        plt.tight_layout()
        
        # Save graph
        try:
            plt.savefig(self.graph_file, dpi=300, bbox_inches='tight')
            print(f"✅ Graphs saved to: {self.graph_file}")
        except Exception as e:
            print(f"❌ Failed to save graphs: {e}")
        
        # Don't show the plot during training
        plt.close()
    
    def generate_statistics_summary(self):
        """Generate and print flight statistics"""
        if len(self.flight_data['step']) == 0:
            return
        
        df = pd.DataFrame(self.flight_data)
        
        print("\n" + "="*60)
        print("📊 FLIGHT DATA SUMMARY")
        print("="*60)
        
        # Basic stats
        duration = time.time() - self.session_start_time
        print(f"⏱️ Session Duration: {duration:.1f} seconds ({duration/60:.1f} minutes)")
        print(f"📊 Total Steps: {self.step_count:,}")
        print(f"🔄 Total Episodes: {self.current_episode}")
        print(f"📈 Average Steps per Episode: {self.step_count/max(1, self.current_episode):.1f}")
        
        # Flight parameter statistics
        print(f"\n🛩️ Flight Parameters:")
        print(f"   Altitude: {df['altitude_m'].min():.1f}m - {df['altitude_m'].max():.1f}m (avg: {df['altitude_m'].mean():.1f}m)")
        print(f"   Pitch: {df['pitch_deg'].min():.1f}° - {df['pitch_deg'].max():.1f}° (avg: {df['pitch_deg'].mean():.1f}°)")
        print(f"   Roll: {df['roll_deg'].min():.1f}° - {df['roll_deg'].max():.1f}° (avg: {df['roll_deg'].mean():.1f}°)")
        print(f"   Heading: {df['heading_deg'].min():.1f}° - {df['heading_deg'].max():.1f}°")
        print(f"   Airspeed: {df['airspeed_kts'].min():.1f}kts - {df['airspeed_kts'].max():.1f}kts (avg: {df['airspeed_kts'].mean():.1f}kts)")
        
        # Reward statistics
        print(f"\n🎯 Reward Statistics:")
        print(f"   Total Reward: {df['reward'].sum():.1f}")
        print(f"   Average Reward: {df['reward'].mean():.2f}")
        print(f"   Best Reward: {df['reward'].max():.2f}")
        print(f"   Worst Reward: {df['reward'].min():.2f}")
        
        print(f"\n📁 Files Generated:")
        print(f"   📊 Data: {self.csv_file}")
        print(f"   📈 Graphs: {self.graph_file}")
        print(f"   📋 Metadata: {self.json_file}")
        print("="*60)
    
    def save_and_generate_graphs(self):
        """Save all data and generate final graphs"""
        if len(self.flight_data['step']) == 0:
            print("⚠️ No flight data recorded")
            return
        
        print(f"\n🔄 Saving flight data and generating graphs...")
        
        # Save final data
        self._save_data_to_csv()
        self.save_metadata()
        
        # Generate graphs
        self.generate_flight_graphs()
        
        # Print summary
        self.generate_statistics_summary()
        
        print(f"✅ Flight data analysis complete!")