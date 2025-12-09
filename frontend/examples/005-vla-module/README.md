# VLA Module - Code Examples and Simulations

**Module**: Module 4 - Vision-Language-Action Systems
**Purpose**: Executable code examples and simulation scenarios for hands-on learning

## Directory Structure

```
examples/vla-module/
├── datasets/           # Mini labeled dataset for perception validation
│   ├── images/        # RGB images from simulation
│   ├── depth/         # Depth maps
│   ├── masks/         # Segmentation masks
│   ├── labels.json    # Bounding boxes and categories
│   ├── data_loader.py # Dataset loading utility
│   └── README.md      # Dataset documentation
│
├── perception/        # Perception pipeline examples (Chapter 2)
│   ├── rgb_processor.py        # RGB camera processing
│   ├── depth_processor.py      # Depth data integration
│   ├── segmentation_node.py    # Lightweight segmentation
│   └── README.md
│
├── planning/          # LLM planning examples (Chapter 5)
│   ├── llm_planner.py          # Real LLM API integration
│   ├── mock_planner.py         # Mock planner fallback
│   ├── plan_schema.json        # JSON schema for action plans
│   ├── safety_validator.py     # Plan safety validation
│   ├── prompts/                # LLM prompt templates
│   └── README.md
│
├── execution/         # Behavior tree execution (Chapter 6)
│   ├── behaviors/              # Individual behavior implementations
│   │   ├── navigate.py        # Navigation behavior
│   │   ├── approach_object.py # Object approach behavior
│   │   └── align_gripper.py   # Gripper alignment behavior
│   ├── vla_executor.py         # Main VLA executor
│   └── README.md
│
├── integration/       # Complete VLA pipeline (Chapter 7)
│   ├── vla_system.py           # Complete VLA system
│   ├── launch/                 # ROS 2 launch files
│   │   ├── vla_gazebo.launch.py
│   │   └── vla_isaac.launch.py
│   └── README.md
│
├── scenarios/         # Simulation world files (Chapter 7)
│   ├── workspace_simple.world
│   ├── workspace_objects.world
│   ├── navigation_manipulation.world
│   └── README.md
│
└── safety/            # Safety examples (Chapter 8)
    ├── constraint_validator.py
    ├── emergency_stop.py
    ├── audit_logger.py
    └── README.md
```

## Quick Start

### Prerequisites

1. **ROS 2 Environment**:
   ```bash
   # Ubuntu 22.04 with ROS 2 Humble
   sudo apt install ros-humble-desktop
   source /opt/ros/humble/setup.bash
   ```

2. **Python Dependencies**:
   ```bash
   pip install opencv-python numpy py_trees torch torchvision pydantic pytest
   ```

3. **Optional LLM APIs**:
   ```bash
   pip install openai anthropic google-generativeai
   ```

4. **Simulation**:
   ```bash
   # Gazebo Classic
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

### Running Examples

#### 1. Test Dataset Loader

```bash
cd examples/vla-module/datasets/
python data_loader.py --validate --stats
```

#### 2. Run Perception Pipeline

```bash
cd examples/vla-module/perception/
python rgb_processor.py --test
```

#### 3. Test LLM Planner (Mock Mode)

```bash
cd examples/vla-module/planning/
python mock_planner.py --test
```

#### 4. Run Complete VLA System in Simulation

```bash
cd examples/vla-module/integration/
ros2 launch launch/vla_gazebo.launch.py
```

## Chapter-by-Chapter Examples

### Chapter 1: Introduction to VLA Systems
- **Location**: `content/module4-vla/ch01-introduction/`
- **Examples**: Theory and diagrams only (no code)

### Chapter 2: Robot Perception Pipeline
- **Location**: `examples/vla-module/perception/`
- **Examples**:
  - RGB camera processing
  - Depth integration
  - Lightweight segmentation
  - Dataset validation

### Chapter 3: Language Understanding
- **Location**: `examples/vla-module/planning/`
- **Examples**:
  - Command parsing
  - Safety filtering
  - Structured representation

### Chapter 4: VLA System Architecture
- **Location**: `examples/vla-module/integration/`
- **Examples**:
  - VLA orchestrator
  - Component interfaces
  - ROS 2 messages/services

### Chapter 5: High-Level Task Planning
- **Location**: `examples/vla-module/planning/`
- **Examples**:
  - LLM planner (real API)
  - Mock planner (fallback)
  - JSON schema validation
  - Prompt templates

### Chapter 6: Action Execution
- **Location**: `examples/vla-module/execution/`
- **Examples**:
  - Behavior tree implementation
  - Individual behaviors
  - ROS 2 action integration

### Chapter 7: Simulation Scenarios
- **Location**: `examples/vla-module/scenarios/` + `integration/`
- **Examples**:
  - Navigation scenarios
  - Manipulation scenarios
  - Complete VLA demonstrations
  - Mini-project

### Chapter 8: Safety, Ethics, Limitations
- **Location**: `examples/vla-module/safety/`
- **Examples**:
  - Safety validators
  - Emergency stop patterns
  - Audit logging
  - Edge case testing

## Development Workflow

### 1. Start with Dataset Validation

```bash
cd examples/vla-module/datasets/
python data_loader.py --validate
```

**Expected**: Dataset integrity check passes

### 2. Test Perception Components

```bash
cd examples/vla-module/perception/
python rgb_processor.py --dataset ../datasets/ --visualize
```

**Expected**: RGB processing works, visualizations display

### 3. Test Planning with Mock Planner

```bash
cd examples/vla-module/planning/
python mock_planner.py --command "Pick up the red cube"
```

**Expected**: JSON plan generated and validated

### 4. Test Execution with Behavior Trees

```bash
cd examples/vla-module/execution/
python vla_executor.py --test
```

**Expected**: Behavior tree executes successfully

### 5. Run Complete VLA Pipeline in Simulation

```bash
cd examples/vla-module/integration/
ros2 launch launch/vla_gazebo.launch.py
```

**Expected**: Gazebo launches, VLA system processes visual input and executes plans

## Troubleshooting

### Dataset not found

**Issue**: `FileNotFoundError: labels.json not found`

**Solution**: Dataset needs to be generated. See `datasets/README.md` for generation instructions.

### ROS 2 not sourced

**Issue**: `ros2: command not found`

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc for persistence
```

### Python dependencies missing

**Issue**: `ImportError: No module named 'cv2'`

**Solution**:
```bash
pip install opencv-python numpy torch torchvision
```

### LLM API key not set

**Issue**: `ValueError: OPENAI_API_KEY not set`

**Solution**:
```bash
export OPENAI_API_KEY="your-api-key-here"
# Or use mock planner fallback
```

### Gazebo won't launch

**Issue**: Gazebo fails to start or crashes

**Solution**:
```bash
# Check Gazebo installation
gazebo --version

# Reinstall if needed
sudo apt install --reinstall ros-humble-gazebo-ros-pkgs
```

## Performance Optimization

### For Student Laptops (Mid-Range Hardware)

1. **Use Lightweight Perception Models**:
   - MobileNet-based segmentation (runs on CPU)
   - Reduce image resolution if needed (640x480 instead of 1920x1080)

2. **Optimize Gazebo Settings**:
   - Reduce physics update rate
   - Simplify object geometries
   - Disable shadows/reflections if slow

3. **Cache LLM Responses**:
   - Use mock planner for development
   - Cache API responses for repeated scenarios
   - Minimize token usage with concise prompts

4. **Use Performance Monitoring**:
   ```bash
   # Monitor perception latency
   ros2 topic hz /perception/output

   # Monitor CPU/memory usage
   htop
   ```

## Testing

### Unit Tests

```bash
cd examples/vla-module/
pytest tests/
```

### Integration Tests

```bash
cd examples/vla-module/integration/
ros2 launch launch/vla_gazebo.launch.py test:=true
```

### Performance Benchmarks

```bash
cd examples/vla-module/
python scripts/benchmark.py --all
```

**Expected Performance**:
- Perception: <200ms per frame
- LLM Planning: <5s per plan
- Behavior Tree Execution: Real-time (30+ Hz)

## Contributing

If you find issues or want to contribute improvements:

1. File an issue in the textbook repository
2. Submit a pull request with your changes
3. Include tests and documentation for new features

## License

MIT License - See LICENSE file in repository root

## Contact

For questions or support:
- Repository: [Add repository URL]
- Issues: [Add issues URL]
- Documentation: See `content/module4-vla/` for full module content
