# Digital Twin Project

## Prerequisites

### Required Software
- **Python 3.8+**
- **SUMO (Simulation of Urban Mobility)**: Download and install from [https://sumo.dlr.de/docs/Installing/index.html](https://sumo.dlr.de/docs/Installing/index.html)
- **Git** (for cloning the repository)

### Verify SUMO Installation
```bash
# Check if SUMO is properly installed
sumo --version
sumo-gui --version
```

## Installation

### 1. Create Virtual Environment (Recommended)
```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Linux/Mac:
source venv/bin/activate
# On Windows:
venv\Scripts\activate
```

### 2. Install Python Dependencies
```bash
pip install -r requirements.txt
```

## Usage

### Running Simulations

Use the main simulation runner to execute both scenarios simultaneously:

```bash
cd src
python run_simulation.py --configA dub.sumocfg.xml --configB simB.sumocfg.xml --portA 8813 --portB 8814 --steps 10000 --interval 0.1 --verbose
```

**Parameters:**
- `--configA`: Configuration file for Simulation A
- `--configB`: Configuration file for Simulation B  
- `--portA`: TraCI port for Simulation A (default: 8813)
- `--portB`: TraCI port for Simulation B (default: 8814)
- `--steps`: Number of simulation steps (default: 10000)
- `--interval`: Time interval between steps in seconds (default: 0.1)
- `--verbose`: Enable detailed logging

### Analyzing Results

After running simulations, analyze and compare the results:

```bash
cd src
python results_script.py
```

This will generate:
- Vehicle count comparison plots
- Mean speed analysis charts
- Collision monitoring reports