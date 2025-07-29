#!/usr/bin/env python3
"""
Robust Simulation Comparison Script
Compares vehicle counts (excluding pedestrians) between Simulation A and Simulation B from FCD XML files.
Handles incomplete/malformed XML entries gracefully.
"""

import re
import logging
import matplotlib.pyplot as plt
import pandas as pd
from typing import Dict, List, Tuple, Optional
import os
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class RobustFCDParser:
    """Robust parser for FCD XML files that handles malformed entries."""
    
    def __init__(self):
        # Regex patterns to extract data from XML lines
        self.timestep_pattern = r'<timestep time="([^"]+)">'
        self.vehicle_pattern = r'<vehicle[^>]*id="([^"]*)"[^>]*>'
        self.person_pattern = r'<person[^>]*id="([^"]*)"[^>]*>'
        self.timestep_end_pattern = r'</timestep>'
        
    def clean_xml_line(self, line: str) -> Optional[str]:
        """
        Clean and validate XML line.
        Returns None if line is invalid/incomplete.
        """
        line = line.strip()
        
        # Skip empty lines
        if not line:
            return None
            
        # Skip comment lines
        if line.startswith('<!--') or line.startswith('<?xml') or line.startswith('<fcd-export'):
            return None
            
        # Check for incomplete XML tags (missing closing >)
        if '<' in line and not line.endswith('>') and not line.endswith('/>'):
            logger.debug(f"Skipping incomplete line: {line[:50]}...")
            return None
            
        # Check for malformed attributes (unclosed quotes)
        if line.count('"') % 2 != 0:
            logger.debug(f"Skipping line with unclosed quotes: {line[:50]}...")
            return None
            
        return line
    
    def extract_timestep(self, line: str) -> Optional[float]:
        """Extract timestep value from timestep tag."""
        match = re.search(self.timestep_pattern, line)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                return None
        return None
    
    def extract_vehicle_id(self, line: str) -> Optional[str]:
        """Extract vehicle ID from vehicle tag."""
        match = re.search(self.vehicle_pattern, line)
        return match.group(1) if match else None
    
    def extract_person_id(self, line: str) -> Optional[str]:
        """Extract person ID from person tag."""
        match = re.search(self.person_pattern, line)
        return match.group(1) if match else None
    
    def is_timestep_end(self, line: str) -> bool:
        """Check if line is a timestep closing tag."""
        return bool(re.search(self.timestep_end_pattern, line))
    
    def parse_fcd_file(self, file_path: str) -> Dict[float, int]:
        """
        Parse FCD XML file and return timestep -> vehicle count mapping.
        Handles malformed XML gracefully.
        """
        logger.info(f"Parsing FCD file: {file_path}")
        
        if not os.path.exists(file_path):
            logger.error(f"File not found: {file_path}")
            return {}
        
        timestep_counts = {}
        current_timestep = None
        current_vehicle_count = 0
        line_number = 0
        valid_timesteps = 0
        
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                for line in file:
                    line_number += 1
                    
                    # Clean and validate the line
                    clean_line = self.clean_xml_line(line)
                    if clean_line is None:
                        continue
                    
                    try:
                        # Check for timestep start
                        timestep = self.extract_timestep(clean_line)
                        if timestep is not None:
                            # Save previous timestep data if exists
                            if current_timestep is not None:
                                timestep_counts[current_timestep] = current_vehicle_count
                                valid_timesteps += 1
                            
                            current_timestep = timestep
                            current_vehicle_count = 0
                            continue
                        
                        # Check for timestep end
                        if self.is_timestep_end(clean_line):
                            if current_timestep is not None:
                                timestep_counts[current_timestep] = current_vehicle_count
                                valid_timesteps += 1
                                current_timestep = None
                                current_vehicle_count = 0
                            continue
                        
                        # Count only vehicles in current timestep (exclude persons/pedestrians)
                        if current_timestep is not None:
                            if self.extract_vehicle_id(clean_line):
                                current_vehicle_count += 1
                    
                    except Exception as e:
                        logger.debug(f"Error processing line {line_number}: {e}")
                        continue
                
                # Handle last timestep if file doesn't end with </timestep>
                if current_timestep is not None:
                    timestep_counts[current_timestep] = current_vehicle_count
                    valid_timesteps += 1
        
        except Exception as e:
            logger.error(f"Error reading file {file_path}: {e}")
            return {}
        
        logger.info(f"Successfully parsed {valid_timesteps} timesteps from {line_number} lines")
        logger.info(f"Timestep range: {min(timestep_counts.keys()) if timestep_counts else 'N/A'} - {max(timestep_counts.keys()) if timestep_counts else 'N/A'}")
        
        return timestep_counts

class RobustSummaryParser:
    """Robust parser for Summary XML files that handles malformed entries."""
    
    def __init__(self):
        # Regex pattern to extract data from summary XML step lines
        self.step_pattern = r'<step time="([^"]+)"[^>]*meanSpeed="([^"]+)"[^>]*>'
        
    def clean_xml_line(self, line: str) -> Optional[str]:
        """
        Clean and validate XML line.
        Returns None if line is invalid/incomplete.
        """
        line = line.strip()
        
        # Skip empty lines
        if not line:
            return None
            
        # Skip comment lines and XML headers
        if line.startswith('<!--') or line.startswith('<?xml') or line.startswith('<summary'):
            return None
            
        # Check for incomplete XML tags (missing closing >)
        if '<' in line and not line.endswith('>') and not line.endswith('/>'):
            logger.debug(f"Skipping incomplete summary line: {line[:50]}...")
            return None
            
        # Check for malformed attributes (unclosed quotes)
        if line.count('"') % 2 != 0:
            logger.debug(f"Skipping summary line with unclosed quotes: {line[:50]}...")
            return None
            
        return line
    
    def extract_step_data(self, line: str) -> Optional[Tuple[float, float]]:
        """Extract timestep and meanSpeed from step tag."""
        match = re.search(self.step_pattern, line)
        if match:
            try:
                timestep = float(match.group(1))
                mean_speed = float(match.group(2))
                # Handle negative values (SUMO uses -1.00 for invalid/no data)
                if mean_speed < 0:
                    mean_speed = 0.0
                return timestep, mean_speed
            except ValueError:
                return None
        return None
    
    def extract_collision_data(self, line: str) -> Optional[Tuple[float, int]]:
        """Extract timestep and collision count from a step line."""
        if 'step time=' in line and 'collisions=' in line:
            try:
                # Extract timestep
                time_match = re.search(r'time="([^"]+)"', line)
                collision_match = re.search(r'collisions="([^"]+)"', line)
                
                if time_match and collision_match:
                    timestep = float(time_match.group(1))
                    collisions = int(collision_match.group(1))
                    return timestep, collisions
            except ValueError:
                return None
        return None
    
    def parse_collision_data(self, file_path: str) -> Dict[float, int]:
        """
        Parse Summary XML file and return timestep -> collision count mapping.
        Handles malformed XML gracefully.
        """
        logger.info(f"Parsing collision data from: {file_path}")
        
        if not os.path.exists(file_path):
            logger.error(f"Summary file not found: {file_path}")
            return {}
        
        timestep_collisions = {}
        line_number = 0
        valid_steps = 0
        total_collisions = 0
        
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                for line in file:
                    line_number += 1
                    
                    # Clean and validate the line
                    clean_line = self.clean_xml_line(line)
                    if clean_line is None:
                        continue
                    
                    try:
                        # Extract collision data
                        collision_data = self.extract_collision_data(clean_line)
                        if collision_data:
                            timestep, collisions = collision_data
                            timestep_collisions[timestep] = collisions
                            total_collisions += collisions
                            valid_steps += 1
                    
                    except Exception as e:
                        logger.debug(f"Error processing collision line {line_number}: {e}")
                        continue
        
        except Exception as e:
            logger.error(f"Error reading collision data from {file_path}: {e}")
            return {}
        
        logger.info(f"Successfully parsed {valid_steps} steps for collision data")
        logger.info(f"Total collisions detected: {total_collisions}")
        if total_collisions > 0:
            logger.warning(f"⚠️  COLLISIONS DETECTED: {total_collisions} collisions found in simulation!")
        else:
            logger.info("✅ No collisions detected - simulation running cleanly")
        
        return timestep_collisions
    
    def parse_summary_file(self, file_path: str) -> Dict[float, float]:
        """
        Parse Summary XML file and return timestep -> mean speed mapping.
        Handles malformed XML gracefully.
        """
        logger.info(f"Parsing summary file: {file_path}")
        
        if not os.path.exists(file_path):
            logger.error(f"Summary file not found: {file_path}")
            return {}
        
        timestep_speeds = {}
        line_number = 0
        valid_steps = 0
        
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                for line in file:
                    line_number += 1
                    
                    # Clean and validate the line
                    clean_line = self.clean_xml_line(line)
                    if clean_line is None:
                        continue
                    
                    try:
                        # Extract speed data
                        step_data = self.extract_step_data(clean_line)
                        if step_data:
                            timestep, mean_speed = step_data
                            timestep_speeds[timestep] = mean_speed
                            valid_steps += 1
                    
                    except Exception as e:
                        logger.debug(f"Error processing summary line {line_number}: {e}")
                        continue
        
        except Exception as e:
            logger.error(f"Error reading summary file {file_path}: {e}")
            return {}
        
        logger.info(f"Successfully parsed {valid_steps} steps from {line_number} lines")
        if timestep_speeds:
            min_time = min(timestep_speeds.keys())
            max_time = max(timestep_speeds.keys())
            logger.info(f"Timestep range: {min_time} - {max_time}")
        
        return timestep_speeds

def align_timesteps(sim_a_data: Dict[float, int], sim_b_data: Dict[float, int]) -> Tuple[List[float], List[int], List[int]]:
    """
    Align timesteps between two simulations and return common timesteps.
    """
    if not sim_a_data or not sim_b_data:
        logger.warning("One or both simulations have no valid data")
        return [], [], []
    
    # Get common timesteps
    timesteps_a = set(sim_a_data.keys())
    timesteps_b = set(sim_b_data.keys())
    common_timesteps = sorted(timesteps_a.intersection(timesteps_b))
    
    if not common_timesteps:
        logger.warning("No common timesteps found between simulations")
        return [], [], []
    
    # Extract aligned data
    aligned_timesteps = []
    aligned_counts_a = []
    aligned_counts_b = []
    
    for timestep in common_timesteps:
        aligned_timesteps.append(timestep)
        aligned_counts_a.append(sim_a_data[timestep])
        aligned_counts_b.append(sim_b_data[timestep])
    
    logger.info(f"Found {len(common_timesteps)} common timesteps")
    return aligned_timesteps, aligned_counts_a, aligned_counts_b

def align_speed_timesteps(sim_a_speeds: Dict[float, float], sim_b_speeds: Dict[float, float]) -> Tuple[List[float], List[float], List[float]]:
    """
    Align timesteps between two simulations for speed data and return common timesteps.
    """
    if not sim_a_speeds or not sim_b_speeds:
        logger.warning("One or both simulations have no valid speed data")
        return [], [], []
    
    # Get common timesteps
    timesteps_a = set(sim_a_speeds.keys())
    timesteps_b = set(sim_b_speeds.keys())
    common_timesteps = sorted(timesteps_a.intersection(timesteps_b))
    
    if not common_timesteps:
        logger.warning("No common timesteps found between speed simulations")
        return [], [], []
    
    # Extract aligned data
    aligned_timesteps = []
    aligned_speeds_a = []
    aligned_speeds_b = []
    
    for timestep in common_timesteps:
        aligned_timesteps.append(timestep)
        aligned_speeds_a.append(sim_a_speeds[timestep])
        aligned_speeds_b.append(sim_b_speeds[timestep])
    
    logger.info(f"Found {len(common_timesteps)} common timesteps for speed comparison")
    return aligned_timesteps, aligned_speeds_a, aligned_speeds_b

def create_comparison_plot(timesteps: List[float], counts_a: List[int], counts_b: List[int], 
                          output_file: str = "robust_simulation_comparison.png", 
                          step_interval: int = 100):
    """
    Create a line plot comparing vehicle counts between simulations.
    """
    if not timesteps:
        logger.error("No data to plot")
        return
    
    # Sample data at specified intervals
    sampled_indices = range(0, len(timesteps), step_interval)
    sampled_timesteps = [timesteps[i] for i in sampled_indices]
    sampled_counts_a = [counts_a[i] for i in sampled_indices]
    sampled_counts_b = [counts_b[i] for i in sampled_indices]
    
    # Create the plot
    plt.figure(figsize=(12, 8))
    
    plt.plot(sampled_timesteps, sampled_counts_a, 'b-', linewidth=2, label='Real World', alpha=0.8)
    plt.plot(sampled_timesteps, sampled_counts_b, 'r-', linewidth=2, label='Digital Twin', alpha=0.8)
    
    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('Number of Vehicles', fontsize=12)
    plt.title('Vehicle Count Comparison: Real World vs Digital Twin', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    
    # Add statistics to the plot
    avg_a = sum(sampled_counts_a) / len(sampled_counts_a) if sampled_counts_a else 0
    avg_b = sum(sampled_counts_b) / len(sampled_counts_b) if sampled_counts_b else 0
    max_a = max(sampled_counts_a) if sampled_counts_a else 0
    max_b = max(sampled_counts_b) if sampled_counts_b else 0
    
    stats_text = f'Avg A: {avg_a:.1f} | Max A: {max_a}\nAvg B: {avg_b:.1f} | Max B: {max_b}'
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    logger.info(f"Plot saved as: {output_file}")
    
    # Print summary statistics
    logger.info(f"Simulation A - Average: {avg_a:.1f}, Max: {max_a}, Min: {min(sampled_counts_a) if sampled_counts_a else 0}")
    logger.info(f"Simulation B - Average: {avg_b:.1f}, Max: {max_b}, Min: {min(sampled_counts_b) if sampled_counts_b else 0}")

def create_speed_comparison_plot(timesteps: List[float], speeds_a: List[float], speeds_b: List[float], 
                                output_file: str = "speed_comparison.png", 
                                step_interval: int = 100):
    """
    Create a line plot comparing mean speeds between simulations.
    """
    if not timesteps:
        logger.error("No speed data to plot")
        return
    
    # Sample data at specified intervals
    sampled_indices = range(0, len(timesteps), step_interval)
    sampled_timesteps = [timesteps[i] for i in sampled_indices]
    sampled_speeds_a = [speeds_a[i] for i in sampled_indices]
    sampled_speeds_b = [speeds_b[i] for i in sampled_indices]
    
    # Create the plot
    plt.figure(figsize=(12, 8))
    
    plt.plot(sampled_timesteps, sampled_speeds_a, 'b-', linewidth=2, label='Simulation A', alpha=0.8)
    plt.plot(sampled_timesteps, sampled_speeds_b, 'r-', linewidth=2, label='Simulation B', alpha=0.8)
    
    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('Mean Speed (m/s)', fontsize=12)
    plt.title('Mean Speed Comparison: Real World vs Digital Twin', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    
    # Add statistics to the plot
    avg_a = sum(sampled_speeds_a) / len(sampled_speeds_a) if sampled_speeds_a else 0
    avg_b = sum(sampled_speeds_b) / len(sampled_speeds_b) if sampled_speeds_b else 0
    max_a = max(sampled_speeds_a) if sampled_speeds_a else 0
    max_b = max(sampled_speeds_b) if sampled_speeds_b else 0
    
    stats_text = f'Avg A: {avg_a:.2f} m/s | Max A: {max_a:.2f} m/s\nAvg B: {avg_b:.2f} m/s | Max B: {max_b:.2f} m/s'
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    logger.info(f"Speed comparison plot saved as: {output_file}")
    
    # Print summary statistics
    logger.info(f"Simulation A Speed - Average: {avg_a:.2f} m/s, Max: {max_a:.2f} m/s, Min: {min(sampled_speeds_a) if sampled_speeds_a else 0:.2f} m/s")
    logger.info(f"Simulation B Speed - Average: {avg_b:.2f} m/s, Max: {max_b:.2f} m/s, Min: {min(sampled_speeds_b) if sampled_speeds_b else 0:.2f} m/s")

def create_collision_plot(timesteps: List[float], collisions: List[int], 
                         output_file: str = "collision_monitoring_simB.png", 
                         step_interval: int = 100):
    """
    Create a plot monitoring collision count over time for SimB.
    This plot helps identify simulation quality issues.
    """
    if not timesteps:
        logger.error("No collision data to plot")
        return
    
    # Sample data at specified intervals
    sampled_indices = range(0, len(timesteps), step_interval)
    sampled_timesteps = [timesteps[i] for i in sampled_indices]
    sampled_collisions = [collisions[i] for i in sampled_indices]
    
    # Create the plot
    plt.figure(figsize=(12, 6))
    
    # Use different colors based on collision count
    colors = ['green' if c == 0 else 'orange' if c <= 2 else 'red' for c in sampled_collisions]
    
    plt.scatter(sampled_timesteps, sampled_collisions, c=colors, alpha=0.7, s=30)
    plt.plot(sampled_timesteps, sampled_collisions, 'b-', linewidth=1, alpha=0.5)
    
    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('Number of Collisions', fontsize=12)
    plt.title('Collision Monitoring: Digital Twin', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    
    # Add statistics and interpretation
    total_collisions = sum(sampled_collisions)
    max_collisions = max(sampled_collisions) if sampled_collisions else 0
    collision_timesteps = len([c for c in sampled_collisions if c > 0])
    
    # Color-coded legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='green', label='No Collisions (Good)'),
        Patch(facecolor='orange', label='<10 Collisions (Warning)'),
        Patch(facecolor='red', label='10+ Collisions (Critical)')
    ]
    plt.legend(handles=legend_elements, loc='upper right')
    
    # Add interpretation text
    if total_collisions == 0:
        status = "✅ EXCELLENT: No collisions detected"
        status_color = "green"
    elif total_collisions <= 10:
        status = "⚠️ WARNING: Few collisions detected"
        status_color = "orange"
    else:
        status = "❌ CRITICAL: Many collisions detected"
        status_color = "red"
    
    stats_text = f'Total Collisions: {total_collisions}\nMax per step: {max_collisions}\nTimesteps with collisions: {collision_timesteps}\n\n{status}'
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor=status_color, alpha=0.2))
    
    # Set y-axis to start from 0 and add some padding
    plt.ylim(bottom=0, top=max(max_collisions + 1, 1))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()
    
    logger.info(f"Collision monitoring plot saved to: {output_file}")
    logger.info(f"Collision Summary - Total: {total_collisions}, Max per step: {max_collisions}, Affected timesteps: {collision_timesteps}")

def main():
    """Main function to run the simulation comparison."""
    
    # File paths
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sim_a_fcd_file = os.path.join(current_dir, "fcd_simA.xml")
    sim_b_fcd_file = os.path.join(current_dir, "fcd_simB.xml")
    sim_a_summary_file = os.path.join(current_dir, "summary_simA.xml")
    sim_b_summary_file = os.path.join(current_dir, "summary_simB.xml")
    
    print("Starting robust simulation comparison analysis...")
    print(f"Simulation A FCD file: {sim_a_fcd_file}")
    print(f"Simulation B FCD file: {sim_b_fcd_file}")
    print(f"Simulation A Summary file: {sim_a_summary_file}")
    print(f"Simulation B Summary file: {sim_b_summary_file}")
    
    logger.info("Starting robust simulation analysis...")
    
    # Initialize parsers
    fcd_parser = RobustFCDParser()
    summary_parser = RobustSummaryParser()
    
    # Parse FCD files for vehicle counts
    sim_a_data = fcd_parser.parse_fcd_file(sim_a_fcd_file)
    sim_b_data = fcd_parser.parse_fcd_file(sim_b_fcd_file)
    
    # Parse Summary files for speed data
    sim_a_speeds = summary_parser.parse_summary_file(sim_a_summary_file)
    sim_b_speeds = summary_parser.parse_summary_file(sim_b_summary_file)
    
    # Parse collision data for SimB (for quality monitoring)
    sim_b_collisions = summary_parser.parse_collision_data(sim_b_summary_file)
    
    # Check if we have valid data
    if not sim_a_data and not sim_b_data:
        logger.error("No valid FCD data found in either simulation file")
    elif not sim_a_data:
        logger.warning("No valid FCD data found in Simulation A")
    elif not sim_b_data:
        logger.warning("No valid FCD data found in Simulation B")
    
    if not sim_a_speeds and not sim_b_speeds:
        logger.error("No valid speed data found in either summary file")
    elif not sim_a_speeds:
        logger.warning("No valid speed data found in Simulation A summary")
    elif not sim_b_speeds:
        logger.warning("No valid speed data found in Simulation B summary")
    
    # Create vehicle count comparison if data is available
    if sim_a_data or sim_b_data:
        timesteps, counts_a, counts_b = align_timesteps(sim_a_data, sim_b_data)
        
        if timesteps:
            # Create plot with sampling every 50 timesteps (adjustable)
            create_comparison_plot(timesteps, counts_a, counts_b, step_interval=50)
        else:
            logger.error("No common timesteps found between simulations for vehicle counts")
    
    # Create speed comparison if data is available
    if sim_a_speeds or sim_b_speeds:
        speed_timesteps, speeds_a, speeds_b = align_speed_timesteps(sim_a_speeds, sim_b_speeds)
        
        if speed_timesteps:
            # Create speed comparison plot with sampling every 50 timesteps (adjustable)
            create_speed_comparison_plot(speed_timesteps, speeds_a, speeds_b, step_interval=50)
        else:
            logger.error("No common timesteps found between simulations for speed data")
    
    # Create collision monitoring plot for SimB
    if sim_b_collisions:
        collision_timesteps = sorted(sim_b_collisions.keys())
        collision_counts = [sim_b_collisions[t] for t in collision_timesteps]
        
        # Create collision monitoring plot with sampling every 50 timesteps (adjustable)
        create_collision_plot(collision_timesteps, collision_counts, step_interval=50)
    else:
        logger.warning("No collision data found for Simulation B")
    
    print("\nAnalysis complete! Check the generated plots:")
    if sim_a_data or sim_b_data:
        print("- robust_simulation_comparison.png (Vehicle Count Comparison)")
    if sim_a_speeds or sim_b_speeds:
        print("- speed_comparison.png (Mean Speed Comparison)")
    if sim_b_collisions:
        print("- collision_monitoring_simB.png (Collision Monitoring for SimB)")

if __name__ == "__main__":
    main()
