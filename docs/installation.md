# Installation and Setup Guide

## System Requirements

- **Operating System**: Linux (Ubuntu 20.04+ recommended)
- **CPU**: Multi-core processor (4+ cores recommended)
- **RAM**: 8GB+ recommended
- **Disk Space**: 500MB for core libraries
- **Compiler**: GCC 9.4.0+ or Clang 10.0+
- **Build System**: CMake 3.10+

## Dependencies

- **Core Libraries**: 
  - Boost 1.71+
  - OpenSSL 1.1.1+
  - Eigen 3.3.7+
  - OpenMP 4.5+

- **Optional Dependencies**:
  - Python 3.8+ (for visualization tools)
  - CUDA 11.0+ (for GPU acceleration)

## Basic Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/GlobalSushrut/zero-point-verifiable-physics.git
cd zero-point-verifiable-physics
```

### Step 2: Install Dependencies

On Ubuntu/Debian systems:

```bash
sudo apt update
sudo apt install build-essential cmake \
    libboost-all-dev \
    libeigen3-dev \
    libssl-dev \
    libgomp1 \
    python3-dev \
    python3-numpy \
    python3-matplotlib
```

### Step 3: Build the Project

```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
```

### Step 4: Run Tests

```bash
make test
```

## Advanced Configuration

### CPU Affinity Settings

For deterministic performance, set CPU affinity:

```bash
# In your application:
std::vector<int> cpu_cores = {1, 2, 3};  // Use cores 1-3
solver->getScheduler()->setCpuAffinity(cpu_cores);

# Or using taskset:
taskset -c 1,2,3 ./military_demo
```

### Verification Levels

Configure the verification level based on your needs:

```cpp
// Create physics solver with appropriate verification level
auto solver = std::make_shared<PhysicsSolverMilitary>(
    VerificationLevel::MILITARY);  // For highest assurance
```

Available verification levels:
- `NONE`: No verification (maximum performance)
- `BASIC`: Essential conservation laws only
- `STANDARD`: Standard physics verification
- `MILITARY`: Military-grade with cryptographic proofs
- `PARANOID`: Ultra-high precision with zero tolerance

## Docker Installation

For containerized deployment:

```bash
# Build the Docker image
docker build -t zero-point-physics .

# Run the container with demo
docker run --rm zero-point-physics ./run_demo.sh

# For interactive shell
docker run -it --rm zero-point-physics /bin/bash
```

## Troubleshooting

### Common Issues

1. **Build Errors**:
   - Ensure you have the correct compiler version
   - Check that all dependencies are installed

2. **Performance Issues**:
   - Set CPU affinity to avoid context switching
   - Disable power management features for consistent timing

3. **Verification Failures**:
   - Adjust error tolerance thresholds in config
   - Check for numerical stability issues with extreme values

### Getting Help

For additional help:
- Submit issues on GitHub
- Check documentation in the `/docs` directory
- Contact the maintainers at support@zero-point-physics.com

## License Information

This project is protected by a proprietary license. See the `LICENSE` file for details.
