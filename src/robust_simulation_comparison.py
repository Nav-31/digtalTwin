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
    
    plt.plot(sampled_timesteps, sampled_counts_a, 'b-', linewidth=2, label='Simulation A', alpha=0.8)
    plt.plot(sampled_timesteps, sampled_counts_b, 'r-', linewidth=2, label='Simulation B', alpha=0.8)
    
    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('Number of Vehicles', fontsize=12)
    plt.title('Vehicle Count Comparison: Simulation A vs Simulation B', fontsize=14, fontweight='bold')
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

def main():
    """Main function to run the simulation comparison."""
    
    # File paths
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sim_a_file = os.path.join(current_dir, "fcd_simA.xml")
    sim_b_file = os.path.join(current_dir, "fcd_simB.xml")
    
    print("Starting robust simulation comparison analysis...")
    print(f"Simulation A FCD file: {sim_a_file}")
    print(f"Simulation B FCD file: {sim_b_file}")
    
    logger.info("Starting robust simulation analysis...")
    
    # Initialize parser
    parser = RobustFCDParser()
    
    # Parse both simulation files
    sim_a_data = parser.parse_fcd_file(sim_a_file)
    sim_b_data = parser.parse_fcd_file(sim_b_file)
    
    # Check if we have valid data
    if not sim_a_data and not sim_b_data:
        logger.error("No valid data found in either simulation file")
        return
    elif not sim_a_data:
        logger.warning("No valid data found in Simulation A")
    elif not sim_b_data:
        logger.warning("No valid data found in Simulation B")
    
    # Align timesteps and create comparison
    timesteps, counts_a, counts_b = align_timesteps(sim_a_data, sim_b_data)
    
    if timesteps:
        # Create plot with sampling every 50 timesteps (adjustable)
        create_comparison_plot(timesteps, counts_a, counts_b, step_interval=50)
    else:
        logger.error("No common timesteps found between simulations")
    
    print("\nAnalysis complete! Check the generated plot:")
    print("- robust_simulation_comparison.png")

if __name__ == "__main__":
    main()
