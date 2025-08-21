#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import json
from datetime import datetime
import glob
import argparse

class FlightDataViewer:
    """
    Viewer and analyzer for saved flight data
    """
    
    def __init__(self, log_dir="flight_logs"):
        self.log_dir = log_dir
        
    def list_available_sessions(self):
        """List all available flight data sessions"""
        csv_files = glob.glob(os.path.join(self.log_dir, "flight_data_*.csv"))
        
        if not csv_files:
            print(f"❌ No flight data found in {self.log_dir}")
            return []
        
        sessions = []
        print(f"📊 Available flight data sessions in {self.log_dir}:")
        print("-" * 60)
        
        for i, csv_file in enumerate(sorted(csv_files), 1):
            basename = os.path.basename(csv_file)
            timestamp_str = basename.replace("flight_data_", "").replace(".csv", "")
            
            try:
                # Try to load metadata
                metadata_file = csv_file.replace("flight_data_", "flight_metadata_").replace(".csv", ".json")
                if os.path.exists(metadata_file):
                    with open(metadata_file, 'r') as f:
                        metadata = json.load(f)
                    duration = metadata.get('session_duration_seconds', 0)
                    steps = metadata.get('total_steps', 0)
                    episodes = metadata.get('total_episodes', 0)
                    
                    print(f"{i:2d}. {timestamp_str}")
                    print(f"    📅 Session: {metadata.get('session_start', 'Unknown')}")
                    print(f"    ⏱️  Duration: {duration:.1f}s ({duration/60:.1f}min)")
                    print(f"    📊 Steps: {steps:,} | Episodes: {episodes}")
                else:
                    # Fallback: just show filename
                    print(f"{i:2d}. {timestamp_str}")
                    print(f"    📁 File: {basename}")
                
                sessions.append(csv_file)
                
            except Exception as e:
                print(f"{i:2d}. {timestamp_str} (⚠️ metadata error)")
                sessions.append(csv_file)
        
        print("-" * 60)
        return sessions
    
    def analyze_session(self, csv_file):
        """Analyze a specific flight data session"""
        if not os.path.exists(csv_file):
            print(f"❌ File not found: {csv_file}")
            return
        
        print(f"📊 Analyzing flight data: {os.path.basename(csv_file)}")
        
        try:
            # Load data
            df = pd.read_csv(csv_file)
            
            if len(df) == 0:
                print("⚠️ No data in file")
                return
            
            # Basic statistics
            duration = df['timestamp'].max() if 'timestamp' in df.columns else 0
            
            print(f"\n📈 Flight Data Summary:")
            print(f"   📊 Total data points: {len(df):,}")
            print(f"   ⏱️ Session duration: {duration:.1f}s ({duration/60:.1f}min)")
            print(f"   🔄 Episodes: {df['episode'].max() if 'episode' in df.columns else 'Unknown'}")
            
            # Flight parameters
            print(f"\n🛩️ Flight Parameters:")
            if 'altitude_m' in df.columns:
                print(f"   🏔️ Altitude: {df['altitude_m'].min():.1f}m - {df['altitude_m'].max():.1f}m (avg: {df['altitude_m'].mean():.1f}m)")
            if 'pitch_deg' in df.columns:
                print(f"   📐 Pitch: {df['pitch_deg'].min():.1f}° - {df['pitch_deg'].max():.1f}° (avg: {df['pitch_deg'].mean():.1f}°)")
            if 'roll_deg' in df.columns:
                print(f"   🎯 Roll: {df['roll_deg'].min():.1f}° - {df['roll_deg'].max():.1f}° (avg: {df['roll_deg'].mean():.1f}°)")
            if 'heading_deg' in df.columns:
                print(f"   🧭 Heading: {df['heading_deg'].min():.1f}° - {df['heading_deg'].max():.1f}°")
            if 'airspeed_kts' in df.columns:
                print(f"   🚀 Airspeed: {df['airspeed_kts'].min():.1f}kts - {df['airspeed_kts'].max():.1f}kts (avg: {df['airspeed_kts'].mean():.1f}kts)")
            
            # Reward statistics
            if 'reward' in df.columns:
                print(f"\n🎯 Training Performance:")
                print(f"   💰 Total reward: {df['reward'].sum():.1f}")
                print(f"   📊 Average reward: {df['reward'].mean():.2f}")
                print(f"   🏆 Best reward: {df['reward'].max():.2f}")
                print(f"   📉 Worst reward: {df['reward'].min():.2f}")
            
            return df
            
        except Exception as e:
            print(f"❌ Error analyzing data: {e}")
            return None
    
    def generate_graphs(self, csv_file, save_graphs=True):
        """Generate graphs for a flight data session"""
        df = pd.read_csv(csv_file)
        
        if len(df) == 0:
            print("⚠️ No data to plot")
            return
        
        # Create figure
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle(f'Flight Data Analysis - {os.path.basename(csv_file)}', fontsize=16, fontweight='bold')
        
        # Colors
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        
        # 1. Altitude over time
        if 'altitude_m' in df.columns and 'timestamp' in df.columns:
            axes[0, 0].plot(df['timestamp'], df['altitude_m'], color=colors[0], linewidth=1.5)
            axes[0, 0].set_title('Altitude vs Time', fontweight='bold')
            axes[0, 0].set_xlabel('Time (seconds)')
            axes[0, 0].set_ylabel('Altitude (meters)')
            axes[0, 0].grid(True, alpha=0.3)
        
        # 2. Pitch and Roll
        if 'pitch_deg' in df.columns and 'roll_deg' in df.columns:
            axes[0, 1].plot(df['timestamp'], df['pitch_deg'], label='Pitch', color=colors[1], linewidth=1.2)
            axes[0, 1].plot(df['timestamp'], df['roll_deg'], label='Roll', color=colors[2], linewidth=1.2)
            axes[0, 1].set_title('Aircraft Attitude vs Time', fontweight='bold')
            axes[0, 1].set_xlabel('Time (seconds)')
            axes[0, 1].set_ylabel('Angle (degrees)')
            axes[0, 1].legend()
            axes[0, 1].grid(True, alpha=0.3)
        
        # 3. Heading
        if 'heading_deg' in df.columns:
            axes[0, 2].plot(df['timestamp'], df['heading_deg'], color=colors[3], linewidth=1.5)
            axes[0, 2].set_title('Heading vs Time', fontweight='bold')
            axes[0, 2].set_xlabel('Time (seconds)')
            axes[0, 2].set_ylabel('Heading (degrees)')
            axes[0, 2].grid(True, alpha=0.3)
        
        # 4. Airspeed
        if 'airspeed_kts' in df.columns:
            axes[1, 0].plot(df['timestamp'], df['airspeed_kts'], color=colors[4], linewidth=1.5)
            axes[1, 0].set_title('Airspeed vs Time', fontweight='bold')
            axes[1, 0].set_xlabel('Time (seconds)')
            axes[1, 0].set_ylabel('Airspeed (knots)')
            axes[1, 0].grid(True, alpha=0.3)
        
        # 5. Reward
        if 'reward' in df.columns:
            axes[1, 1].plot(df['timestamp'], df['reward'], color=colors[5], linewidth=1, alpha=0.7)
            # Add moving average
            if len(df) > 50:
                window_size = min(100, len(df) // 10)
                moving_avg = df['reward'].rolling(window=window_size, center=True).mean()
                axes[1, 1].plot(df['timestamp'], moving_avg, color='red', linewidth=2, label=f'Moving Avg')
                axes[1, 1].legend()
            axes[1, 1].set_title('Reward vs Time', fontweight='bold')
            axes[1, 1].set_xlabel('Time (seconds)')
            axes[1, 1].set_ylabel('Reward')
            axes[1, 1].grid(True, alpha=0.3)
        
        # 6. Flight path
        if 'latitude' in df.columns and 'longitude' in df.columns:
            axes[1, 2].plot(df['longitude'], df['latitude'], color=colors[0], linewidth=1.5, alpha=0.8)
            if len(df) > 0:
                axes[1, 2].scatter(df['longitude'].iloc[0], df['latitude'].iloc[0], 
                                  color='green', s=100, label='Start', zorder=5)
                axes[1, 2].scatter(df['longitude'].iloc[-1], df['latitude'].iloc[-1], 
                                  color='red', s=100, label='End', zorder=5)
            axes[1, 2].set_title('Flight Path (GPS)', fontweight='bold')
            axes[1, 2].set_xlabel('Longitude')
            axes[1, 2].set_ylabel('Latitude')
            axes[1, 2].legend()
            axes[1, 2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_graphs:
            # Save graph
            graph_file = csv_file.replace("flight_data_", "flight_graphs_").replace(".csv", ".png")
            try:
                plt.savefig(graph_file, dpi=300, bbox_inches='tight')
                print(f"✅ Graphs saved to: {graph_file}")
            except Exception as e:
                print(f"❌ Failed to save graphs: {e}")
        
        plt.show()
    
    def compare_sessions(self, csv_files):
        """Compare multiple flight data sessions"""
        if len(csv_files) < 2:
            print("⚠️ Need at least 2 sessions to compare")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Flight Data Session Comparison', fontsize=16, fontweight='bold')
        
        colors = plt.cm.tab10(np.linspace(0, 1, len(csv_files)))
        
        for i, csv_file in enumerate(csv_files):
            try:
                df = pd.read_csv(csv_file)
                basename = os.path.basename(csv_file).replace("flight_data_", "").replace(".csv", "")
                color = colors[i]
                
                # Altitude comparison
                if 'timestamp' in df.columns and 'altitude_m' in df.columns:
                    axes[0, 0].plot(df['timestamp'], df['altitude_m'], 
                                   color=color, label=basename, linewidth=1.5, alpha=0.8)
                
                # Reward comparison
                if 'timestamp' in df.columns and 'reward' in df.columns:
                    # Use moving average for cleaner comparison
                    window_size = min(50, len(df) // 10) if len(df) > 10 else 1
                    moving_avg = df['reward'].rolling(window=window_size, center=True).mean()
                    axes[0, 1].plot(df['timestamp'], moving_avg, 
                                   color=color, label=basename, linewidth=2, alpha=0.8)
                
                # Airspeed comparison
                if 'timestamp' in df.columns and 'airspeed_kts' in df.columns:
                    axes[1, 0].plot(df['timestamp'], df['airspeed_kts'], 
                                   color=color, label=basename, linewidth=1.5, alpha=0.8)
                
                # Flight path comparison
                if 'longitude' in df.columns and 'latitude' in df.columns:
                    axes[1, 1].plot(df['longitude'], df['latitude'], 
                                   color=color, label=basename, linewidth=1.5, alpha=0.8)
                
            except Exception as e:
                print(f"⚠️ Error loading {csv_file}: {e}")
        
        # Configure subplots
        axes[0, 0].set_title('Altitude Comparison')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Altitude (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        axes[0, 1].set_title('Reward Comparison (Moving Average)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Reward')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        axes[1, 0].set_title('Airspeed Comparison')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Airspeed (kts)')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        axes[1, 1].set_title('Flight Path Comparison')
        axes[1, 1].set_xlabel('Longitude')
        axes[1, 1].set_ylabel('Latitude')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='Flight Data Viewer and Analyzer')
    parser.add_argument('--log-dir', default='flight_logs', help='Flight logs directory')
    parser.add_argument('--session', type=int, help='Analyze specific session number')
    parser.add_argument('--compare', nargs='+', type=int, help='Compare multiple sessions')
    parser.add_argument('--latest', action='store_true', help='Analyze latest session')
    
    args = parser.parse_args()
    
    viewer = FlightDataViewer(args.log_dir)
    sessions = viewer.list_available_sessions()
    
    if not sessions:
        return
    
    if args.latest:
        # Analyze latest session
        latest_session = sessions[-1]
        print(f"\n📊 Analyzing latest session...")
        df = viewer.analyze_session(latest_session)
        if df is not None:
            viewer.generate_graphs(latest_session)
    
    elif args.session:
        # Analyze specific session
        if 1 <= args.session <= len(sessions):
            session_file = sessions[args.session - 1]
            df = viewer.analyze_session(session_file)
            if df is not None:
                viewer.generate_graphs(session_file)
        else:
            print(f"❌ Invalid session number. Choose 1-{len(sessions)}")
    
    elif args.compare:
        # Compare multiple sessions
        compare_files = []
        for session_num in args.compare:
            if 1 <= session_num <= len(sessions):
                compare_files.append(sessions[session_num - 1])
            else:
                print(f"⚠️ Invalid session number: {session_num}")
        
        if len(compare_files) >= 2:
            viewer.compare_sessions(compare_files)
        else:
            print("❌ Need at least 2 valid sessions to compare")
    
    else:
        # Interactive mode
        print(f"\n💡 Usage examples:")
        print(f"   python flight_data_viewer.py --latest")
        print(f"   python flight_data_viewer.py --session 1")
        print(f"   python flight_data_viewer.py --compare 1 2 3")

if __name__ == "__main__":
    main()