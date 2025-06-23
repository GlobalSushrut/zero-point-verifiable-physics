# Demo Application Guide

## Overview

The Zero Point Physics Engine includes a comprehensive demo application that showcases its core capabilities, including real-time simulation, formal verification, and military-grade features. This guide explains how to run and customize the demo.

## Running the Demo

The easiest way to run the demo is using the provided script:

```bash
./run_demo.sh
```

This script will build the project if needed, set appropriate CPU affinity, and launch the demo with default parameters.

## Command-line Options

The demo application supports several command-line options:

```bash
./military_demo [options]

Options:
  --steps N              Number of simulation steps (default: 1000)
  --dt N.N               Time step in seconds (default: 0.016)
  --nodes N              Number of simulation nodes (default: 1000)
  --headless             Run without visualization (for benchmarking)
  --verification-level N Set verification level (0-4, default: 3)
  --threads N            Number of worker threads (default: 3)
  --seed N               Random seed for reproducibility
  --record-stats         Record statistics to CSV file
  --dump-state           Export simulation state periodically
  --verbose              Enable detailed logging
```

## Demo Scenarios

The demo includes several built-in scenarios:

```bash
# Standard collision test
./run_demo.sh --scenario collision

# Black hole simulation
./run_demo.sh --scenario black_hole

# Military verification test
./run_demo.sh --scenario verification

# Performance benchmark
./run_demo.sh --scenario benchmark --headless
```

## Visualizing Results

When not running in headless mode, the demo provides real-time visualization:

- **Blue spheres**: Standard physics nodes
- **Red spheres**: Nodes involved in collisions
- **Green lines**: Constraint connections
- **Yellow highlights**: Verification failures
- **Info panel**: Shows performance metrics and verification statistics

## Performance Benchmarking

To benchmark the engine performance:

```bash
./run_demo.sh --scenario benchmark --headless --steps 10000 --record-stats
```

Results will be saved to `performance_results.csv` with the following metrics:

- Average step time (microseconds)
- FPS (frames per second)
- CPU utilization (%)
- Verification success rate (%)
- Verification time (microseconds)
- Energy efficiency score

## Sample Output

```
Zero Point Military-Grade Physics Demo
-------------------------------------
Initialization complete. Starting simulation...

Step: 1000/1000 (100.0%)
Nodes: 1000
Collisions detected: 142
Verification success rate: 99.8%
Average confidence: 0.972
CPU utilization: 76.3%
Energy efficiency: 0.95

Performance Summary:
Average step time: 1243.6 μs
Memory usage: 28.4 MB
Peak collision checks: 12489
Verification overhead: 412.7 μs (33.2%)

Simulation complete. Exiting...
```

## Extending the Demo

You can create custom scenarios by modifying the demo source code:

1. Add a new scenario class that inherits from `DemoScenario`
2. Implement the required methods: `initialize()`, `update()`, `render()`
3. Register your scenario in `ScenarioFactory::createScenario()`
4. Run with `--scenario your_scenario_name`

See `test/military_demo/scenarios/` for examples.
