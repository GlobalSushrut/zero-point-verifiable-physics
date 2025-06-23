#!/bin/bash

# Zero Point Verifiable Physics Engine Demo Runner
# This script runs the military-grade physics demonstration with configurable settings

# Default values
STEPS=100
MODE="standard"
VERBOSE=0

# Process command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    -s|--steps)
      STEPS="$2"
      shift 2
      ;;
    --headless)
      MODE="headless"
      shift
      ;;
    -v|--verbose)
      VERBOSE=1
      shift
      ;;
    -h|--help)
      echo "Zero Point Verifiable Physics Engine Demo Runner"
      echo ""
      echo "Usage: ./run_demo.sh [options]"
      echo ""
      echo "Options:"
      echo "  -s, --steps NUMBER     Set simulation steps (default: 100)"
      echo "  --headless             Run in headless mode (no OpenGL)"
      echo "  -v, --verbose          Enable verbose output"
      echo "  -h, --help             Show this help message"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Use --help for usage information."
      exit 1
      ;;
  esac
done

# Ensure build directory exists
if [ ! -d "build" ]; then
  echo "Building Zero Point Physics Engine..."
  mkdir -p build
  cd build
  cmake ..
  make -j4
  cd ..
fi

# Set CPU affinity for deterministic performance
echo "Setting CPU affinity to cores 1, 2, 3..."
COMMAND="taskset -c 1,2,3 ./build/zero_point/military_demo --steps=$STEPS"

# Add mode flags
if [ "$MODE" == "headless" ]; then
  COMMAND="$COMMAND --headless"
fi

# Add verbose flag
if [ "$VERBOSE" -eq 1 ]; then
  COMMAND="$COMMAND --verbose"
fi

# Run the demo
echo "Running Zero Point Physics Engine Military Demo with $STEPS steps..."
echo "Command: $COMMAND"
echo "----------------------------------------"

# Execute the command
eval $COMMAND

# Show summary stats at the end
if [ $? -eq 0 ]; then
  echo "----------------------------------------"
  echo "🚀 Demo completed successfully!"
  echo "To view more detailed metrics, check the output above."
  echo "Run with --verbose for detailed performance data."
fi
