# Setup Guide: AI-Native Robotics Textbook

This guide will help you set up your development environment for the AI-Native Robotics textbook examples.

## System Requirements

### Minimum Requirements
- **RAM**: 8GB
- **CPU**: Quad-core processor
- **GPU**: Integrated graphics (discrete GPU recommended for better Gazebo performance)
- **Disk Space**: 20GB free space

### Recommended Requirements
- **RAM**: 16GB
- **CPU**: Hexa-core or better
- **GPU**: Dedicated graphics card
- **Disk Space**: 50GB free space

## Operating System Setup

### Option 1: Ubuntu 22.04 LTS (Native - Recommended)

**Install Ubuntu 22.04**:
1. Download Ubuntu 22.04 LTS from https://ubuntu.com/download/desktop
2. Create bootable USB or install in VM
3. Follow installation wizard
4. Update system after installation:
```bash
sudo apt update && sudo apt upgrade -y
```

### Option 2: Windows with WSL2

**Install WSL2**:
1. Open PowerShell as Administrator:
```powershell
wsl --install -d Ubuntu-22.04
```

2. Restart computer when prompted
3. Launch Ubuntu from Start menu
4. Create username and password
5. Update WSL2 Ubuntu:
```bash
sudo apt update && sudo apt upgrade -y
```

**Install WSLg for GUI support** (enables RViz, Gazebo):
- WSLg is included in Windows 11 and recent Windows 10 updates
- Verify with: `wslg --version`

### Option 3: macOS with Docker

**Install Docker Desktop**:
1. Download from https://www.docker.com/products/docker-desktop
2. Install and launch Docker Desktop
3. Increase resources: Docker → Preferences → Resources
   - CPUs: 4+
   - Memory: 8GB+
   - Disk: 20GB+

**Use ROS 2 Docker image**:
```bash
docker pull osrf/ros:humble-desktop-full
docker run -it --rm osrf/ros:humble-desktop-full
```

## ROS 2 Installation

### Install ROS 2 Humble (Ubuntu 22.04 / WSL2)

**1. Set up ROS 2 repositories**:
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**2. Install ROS 2 Humble**:
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop-full -y
```

**3. Install development tools**:
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update
```

**4. Set up environment**:
Add to `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**5. Verify installation**:
```bash
ros2 --version
# Should output: ros2 cli version X.X.X
```

### Install ROS 2 Iron (Optional - for compatibility testing)

Replace `humble` with `iron` in above commands.

## Gazebo Classic Installation

### Install Gazebo Classic 11

**Ubuntu 22.04 / WSL2**:
```bash
sudo apt install gazebo11 libgazebo11-dev -y
sudo apt install ros-humble-gazebo-ros-pkgs -y
```

**Verify installation**:
```bash
gazebo --version
# Should output: Gazebo multi-robot simulator, version 11.x.x
```

**Test Gazebo**:
```bash
gazebo
# GUI window should open (may take 30-60 seconds on first launch)
```

**WSL2 Note**: If Gazebo doesn't open, ensure WSLg is working:
```bash
# Test X11 forwarding
xclock
# Should display a clock window
```

## Python Environment Setup

### Install Python 3 and pip

```bash
sudo apt install python3-pip python3-venv -y
python3 --version  # Should be 3.10+
```

### Install ROS 2 Python packages

```bash
sudo apt install python3-rclpy python3-std-msgs python3-geometry-msgs \
  python3-sensor-msgs -y
```

### Create virtual environment (optional)

```bash
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
pip install --upgrade pip
```

## Additional Tools

### Install RViz2 (Visualization)

```bash
sudo apt install ros-humble-rviz2 -y
```

Test RViz:
```bash
rviz2
# RViz window should open
```

### Install rqt tools (Debugging)

```bash
sudo apt install ros-humble-rqt* -y
```

Useful rqt tools:
- `rqt_graph`: Visualize ROS 2 computation graph
- `rqt_console`: View ROS 2 logs
- `rqt_plot`: Plot topic data in real-time

## Verification Checklist

Run these commands to verify your setup:

```bash
# ROS 2 installation
ros2 --version                  # Should output version

# ROS 2 topics
ros2 topic list                 # Should list topics (may be empty)

# ROS 2 nodes
ros2 node list                  # Should list nodes (may be empty)

# Gazebo
gazebo --version                # Should output version 11.x

# RViz
rviz2 --version                 # Should output version

# Python ROS 2
python3 -c "import rclpy; print('rclpy OK')"  # Should print "rclpy OK"
```

**All checks should pass before proceeding with textbook examples.**

## Clone Textbook Repository

```bash
cd ~
git clone https://github.com/AsmaIqbal01/ai-native-book.git
cd ai-native-book
```

## Test with Hello World Example

Once setup is complete, test with a simple ROS 2 example:

```bash
# Terminal 1: Run talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Run listener
ros2 run demo_nodes_cpp listener
```

You should see messages being published and received.

## Performance Tuning

### For limited hardware (8GB RAM):

**Reduce Gazebo graphics quality**:
Edit `~/.gazebo/gui.ini`:
```ini
[rendering]
anti_aliasing=0
shadows=0
```

**Limit physics update rate**:
In launch files, set:
```xml
<real_time_update_rate>10.0</real_time_update_rate>
```

### For WSL2 users:

**Allocate more resources** to WSL2 by creating `C:\Users\<YourUsername>\.wslconfig`:
```ini
[wsl2]
memory=8GB
processors=4
```

Restart WSL2 after changes:
```powershell
wsl --shutdown
```

## Troubleshooting

If you encounter issues during setup, see [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) for common problems and solutions.

## Next Steps

Once your environment is set up:
1. Read [docs/modules/ros2-nervous-system.md](./docs/modules/ros2-nervous-system.md) (Module 1)
2. Run code examples from `examples/ros2-module/`
3. Follow along with tutorials and exercises

---

**Setup complete!** You're ready to start learning AI-native robotics.

For questions or issues, refer to the troubleshooting guide or open an issue on GitHub.
