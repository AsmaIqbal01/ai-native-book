# Implementation Plan: The Robotic Nervous System (ROS 2) Module

**Branch**: `002-ros2-module` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-ros2-module/spec.md`

## Summary

Create a comprehensive educational module teaching ROS 2 fundamentals for humanoid robotics. The module covers ROS 2 architecture (computation graph, nodes, topics, services, actions), practical Python (rclpy) implementation with 1-2 complete examples per topic, bridging Python AI agents to robot controllers, and URDF structure using a custom simplified humanoid robot. Content will be beginner-friendly with an optional "Further Exploration" appendix for advanced students. All examples run safely in Gazebo Classic simulation using a custom 5-7 DOF humanoid model.

**Technical Approach**: Design-first educational content with simulation-validated code examples, ASCII diagrams for architecture visualization, and progressive complexity from concepts → basic implementation → realistic applications.

## Technical Context

**Language/Version**: Python 3.8+, Markdown (Docusaurus-compatible)
**Primary Dependencies**: ROS 2 Humble/Iron, rclpy, Gazebo Classic 11, standard ROS 2 message packages (geometry_msgs, sensor_msgs, std_msgs)
**Storage**: Git repository for code examples, URDF files, and launch scripts (companion GitHub repo)
**Testing**: Manual simulation testing (Gazebo), ros2 CLI tools verification, constitution compliance validation
**Target Platform**: Linux (Ubuntu 22.04), WSL2/Docker for Windows/macOS users
**Project Type**: Educational content module (textbook chapter format)
**Performance Goals**:
- Code examples run on mid-range hardware (8GB RAM, integrated graphics)
- Gazebo simulation stable at 10Hz+ update rate
- Module completion time: 4 hours hands-on practice
**Constraints**:
- All examples simulation-only (no hardware risk per constitution)
- Beginner-accessible language (avoid ROS 2 jargon without explanation)
- Code must pass 100% automated testing on Humble/Iron
- Markdown compatible with Docusaurus static site generator
**Scale/Scope**:
- Module length: ~8,000-10,000 words
- Code examples: 10-12 complete Python scripts (1-2 per major section)
- Custom URDF: 1 simplified humanoid model (~200-300 lines XML)
- ASCII diagrams: 5-7 architecture visualizations
- Appendix exercises: 5-8 advanced challenges

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Code Quality & Best Practices (Robotics Focus)
- ✅ All code examples MUST follow PEP 8 and ROS 2 conventions
- ✅ Code MUST be modular with single-responsibility functions
- ✅ Comments MUST explain robotics-specific logic, not what code does
- ✅ Complexity MUST be avoided; prioritize readability for beginners
- ✅ Dependencies MUST be explicitly declared with version constraints

### Testing & Validation Standards
- ✅ Every code example MUST be run in simulation before publication
- ✅ Examples MUST include verification instructions (expected output, ros2 commands)
- ✅ ROS 2 nodes MUST include test commands (ros2 topic echo, ros2 service call)
- ✅ Simulation examples MUST document expected robot behavior
- ✅ LLM-generated code MUST be manually reviewed for hallucinated APIs

### User Experience & Consistency
- ✅ Module MUST follow textbook schema: Title, Summary, Learning Outcomes, Sections
- ✅ Terminology MUST remain uniform (maintain glossary consistency)
- ✅ Technical terms MUST be defined on first use
- ✅ Examples MUST progress simple → complex within sections
- ✅ Code formatting MUST be consistent (fenced blocks, syntax highlighting)

### Performance & Accessibility
- ✅ Code MUST run on mid-range laptops (8GB RAM, quad-core CPU, integrated graphics)
- ✅ Simulation examples start with lightweight configurations
- ✅ Initial Gazebo examples use simple geometries and minimal sensors
- ✅ Resource requirements MUST be documented for each example

### Safety Requirements
- ✅ NO unsafe robot control instructions (no disabling safety limits, uncontrolled motion)
- ✅ NO instructions for physical robots without simulation equivalents
- ✅ NO dangerous, untested, or hazardous robotics actions
- ✅ Motion commands MUST include velocity and acceleration limits
- ✅ Simulation MUST be default teaching environment

**Constitution Check Result**: ✅ PASSED - All requirements align with constitution principles

## Project Structure

### Documentation (this feature)

```text
specs/002-ros2-module/
├── spec.md                    # Feature specification (COMPLETED)
├── plan.md                    # This file (IN PROGRESS)
├── checklists/
│   └── requirements.md        # Specification quality validation (COMPLETED)
└── tasks.md                   # Implementation tasks (PENDING - created by /sp.tasks)
```

### Content Output Structure

```text
docs/
└── modules/
    └── ros2-nervous-system.md    # Main module content (Docusaurus-compatible)

examples/
└── ros2-module/
    ├── nodes/
    │   ├── simple_publisher.py
    │   ├── simple_subscriber.py
    │   └── sensor_publisher.py
    ├── services/
    │   ├── calculator_server.py
    │   └── calculator_client.py
    ├── actions/
    │   ├── move_arm_server.py
    │   └── move_arm_client.py
    ├── agent_bridge/
    │   ├── simple_agent.py
    │   └── closed_loop_agent.py
    ├── urdf/
    │   ├── simple_humanoid.urdf
    │   ├── simple_humanoid.urdf.xacro (optional)
    │   └── meshes/ (if needed for visualization)
    └── launch/
        ├── simple_publisher_demo.launch.py
        ├── service_demo.launch.py
        ├── action_demo.launch.py
        └── humanoid_sim.launch.py

assets/
└── diagrams/
    ├── ros2-computation-graph.txt (ASCII art)
    ├── topic-pub-sub-flow.txt
    ├── service-request-response.txt
    ├── action-lifecycle.txt
    └── humanoid-urdf-structure.txt
```

**Structure Decision**: Educational content structure with companion code repository. Main module markdown file in `docs/modules/`, all runnable examples in `examples/ros2-module/` with organized subdirectories by topic. ASCII diagrams stored as text files for easy embedding in markdown. URDF and launch files co-located with code examples for complete simulation setup.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*No violations identified. All requirements align with constitution principles.*

## Phase 0: Research & Discovery

### Research Goals

1. **ROS 2 Learning Resources Audit**
   - Review official ROS 2 documentation structure and terminology
   - Identify common beginner pain points in ROS 2 tutorials
   - Analyze existing educational materials for best practices
   - Document standard ROS 2 message types for humanoid robots

2. **Gazebo Classic & Humanoid Robotics**
   - Research simplified humanoid robot designs (5-7 DOF)
   - Study existing URDF structures for educational purposes
   - Identify appropriate joint types and sensor placements
   - Validate Gazebo Classic 11 compatibility with ROS 2 Humble/Iron

3. **Python Agent Patterns**
   - Research common Python class structures for robot agents
   - Identify best practices for agent-ROS 2 integration
   - Study controller interfaces (position, velocity, effort)
   - Document safety patterns for simulation control

### Research Questions

1. What are the 3-5 most common ROS 2 beginner mistakes to address proactively?
2. Which standard message types are most relevant for humanoid robot examples (geometry_msgs, sensor_msgs)?
3. What joint configuration (revolute vs. prismatic, DOF distribution) provides best learning balance?
4. How should Python agents structure decision-making loops with ROS 2 callbacks?
5. What ASCII diagram styles are most effective for visualizing ROS 2 computation graphs?

### Research Deliverables

- `research.md`: Findings on ROS 2 educational best practices, humanoid URDF design, agent patterns
- List of standard ROS 2 message types to use in examples
- Custom humanoid design specifications (joints, links, sensors)
- Compilation of common ROS 2 errors and debugging solutions

## Phase 1: Design & Architecture

### Content Architecture

**Module Structure**:

```markdown
# The Robotic Nervous System: ROS 2

## Summary
[2-3 paragraphs: ROS 2 as middleware connecting AI to hardware]

## Learning Outcomes
- Understand ROS 2 architecture (nodes, topics, services, actions)
- Create and run Python ROS 2 nodes using rclpy
- Implement all three communication patterns with examples
- Connect Python agents to simulated robot controllers
- Read and modify URDF for humanoid robots

## Section 1: ROS 2 Architecture Overview
### Subsections:
- What is ROS 2? (middleware explanation)
- The Computation Graph (nodes, topics, services, actions)
- DDS and Distributed Systems (conceptual overview)
- When to Use Each Communication Pattern (decision tree)

## Section 2: Creating and Running ROS 2 Nodes
### Subsections:
- ROS 2 Package Structure (minimal organization)
- Your First Publisher Node (complete example)
- Your First Subscriber Node (complete example)
- Debugging with ros2 CLI Tools

## Section 3: Topics, Services, and Actions
### Subsections:
- Topics: Publish/Subscribe Pattern (sensor data example)
- Services: Request/Response Pattern (calculation example)
- Actions: Long-Running Tasks (arm movement example)
- Choosing the Right Pattern (comparison table)

## Section 4: Bridging Python Agents to ROS Controllers
### Subsections:
- Python Agent Structure (class-based design)
- Publishing Commands to Actuators (velocity control)
- Subscribing to Sensor Feedback (closed-loop control)
- Safety Patterns in Simulation

## Section 5: Understanding URDF for Humanoid Robots
### Subsections:
- URDF Structure: Links and Joints
- Custom Simplified Humanoid Model (complete URDF walkthrough)
- Sensors in URDF (camera, IMU definitions)
- Visualizing Robots in RViz and Gazebo

## Further Exploration (Appendix)
- Deep Dive: DDS Architecture and Vendors
- QoS Policies Explained (reliability, durability, history)
- Performance Tuning for ROS 2
- Advanced Debugging Workflows
- Introduction to Custom Messages and Services
```

### Custom Humanoid URDF Design

**Specifications**:

```yaml
Name: SimpleHumanoid
Total DOF: 6
Total Links: 5
Total Joints: 5

Structure:
  base_link: # Torso (fixed to world in simulation)
    shape: box
    dimensions: [0.3, 0.2, 0.4] # width, depth, height (meters)
    mass: 5.0 kg
    sensors:
      - imu (torso_imu): orientation, angular velocity, linear acceleration

  head:
    parent: base_link
    joint_type: revolute (pan)
    axis: [0, 0, 1] # yaw rotation
    limits: [-1.57, 1.57] rad (~-90° to +90°)
    mass: 1.0 kg
    sensors:
      - camera (head_camera): RGB, 640x480, 60° FOV

  left_arm:
    shoulder_joint:
      parent: base_link
      joint_type: revolute
      axis: [0, 1, 0] # pitch rotation
      limits: [-1.57, 1.57] rad
      mass: 0.5 kg (upper arm link)

    elbow_joint:
      parent: left_upper_arm
      joint_type: revolute
      axis: [0, 1, 0] # pitch rotation
      limits: [0, 2.36] rad (0° to ~135°)
      mass: 0.3 kg (forearm link)

  right_arm:
    # Mirror of left_arm structure

Controllers:
  - position_controllers for all joints
  - joint_state_controller for feedback
```

**Rationale**:
- 6 DOF total: manageable for beginners, demonstrates key concepts
- Symmetric arms: teaches URDF patterns without overwhelming complexity
- Single-axis joints: easier to visualize and control than multi-axis
- Box primitives: simple collision/visualization, no mesh dependencies
- Essential sensors only: camera (perception), IMU (balance/orientation)

### Code Example Templates

#### Example 1: Simple Publisher (Section 2)
```python
# simple_publisher.py
# Purpose: Demonstrate basic topic publishing with string messages
# Verification: ros2 topic echo /chatter

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Example 2: Service Server (Section 3)
```python
# calculator_server.py
# Purpose: Demonstrate service request/response pattern
# Verification: ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class CalculatorServer(Node):
    def __init__(self):
        super().__init__('calculator_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Calculator service ready')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: {request.a} + {request.b} = {response.sum}'
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CalculatorServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Example 3: Python Agent Bridge (Section 4)
```python
# simple_agent.py
# Purpose: Demonstrate Python agent controlling simulated robot
# Verification: Launch Gazebo with SimpleHumanoid, observe head rotation

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SimpleAgent(Node):
    """
    Simple agent that commands robot head to pan left and right
    """
    def __init__(self):
        super().__init__('simple_agent')

        # Publisher to head position controller
        self.head_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )

        # Agent decision loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.time_elapsed = 0.0

        self.get_logger().info('Simple agent started - commanding head pan')

    def control_loop(self):
        """Agent decision-making: sinusoidal head movement"""
        # Safety: limit head pan to ±90 degrees
        MAX_PAN = 1.57  # radians (~90 degrees)

        # Sinusoidal command: smooth back-and-forth
        head_position = MAX_PAN * math.sin(self.time_elapsed)

        # Publish command
        msg = Float64MultiArray()
        msg.data = [head_position]  # Single joint command
        self.head_cmd_pub.publish(msg)

        self.get_logger().info(f'Head command: {head_position:.2f} rad')
        self.time_elapsed += 0.1

def main(args=None):
    rclpy.init(args=args)
    agent = SimpleAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ASCII Diagram Designs

**ROS 2 Computation Graph**:
```text
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 Computation Graph                  │
└─────────────────────────────────────────────────────────────┘

    ┌──────────────┐         /camera/image          ┌──────────────┐
    │ Camera Node  │ ──────────────────────────────> │ Vision Node  │
    └──────────────┘         (Topic: Publish)        └──────────────┘
                                                             │
                                                             │ /object_detected
                                                             │ (Topic)
                                                             ▼
    ┌──────────────┐                                 ┌──────────────┐
    │ Control Node │ <────────────────────────────── │ Planning Node│
    └──────────────┘     (Service: Get Trajectory)   └──────────────┘
           │
           │ /arm/position
           │ (Topic: Command)
           ▼
    ┌──────────────┐
    │  Robot Arm   │
    │ (Simulation) │
    └──────────────┘

Legend:
  ──────>  Topic (Publish/Subscribe - one-way, continuous)
  <─────>  Service (Request/Response - synchronous, one-time)
  [Node]   ROS 2 Node (process running specific task)
```

### Design Deliverables

- `design-document.md`: Complete module content outline with section summaries
- `custom-humanoid-spec.yaml`: Full URDF specifications (joints, links, sensors, controllers)
- `code-examples-spec.md`: Complete list of 10-12 code examples with purposes and verification steps
- `ascii-diagrams.txt`: All 5-7 diagrams in final ASCII art format
- `appendix-exercises.md`: 5-8 advanced challenges with solutions outline

## Phase 2: Implementation

### Implementation Workflow

**Step 1: Custom Humanoid URDF Creation**
1. Create `simple_humanoid.urdf` with 5 links, 5 joints per design spec
2. Add IMU sensor plugin to torso link
3. Add camera sensor plugin to head link
4. Define Gazebo-specific tags (colors, friction, controllers)
5. Create launch file to spawn robot in Gazebo
6. Test: Visualize in RViz, spawn in Gazebo, verify joint control

**Step 2: Core Code Examples Development**
1. Implement Example 1-2 (publisher/subscriber) → Test with ros2 topic echo
2. Implement Example 3-4 (service server/client) → Test with ros2 service call
3. Implement Example 5-6 (action server/client) → Test action lifecycle
4. Implement Example 7-8 (agent bridge) → Test with Gazebo simulation
5. Document verification steps and expected outputs for each

**Step 3: Module Content Writing**
1. Write Summary and Learning Outcomes sections
2. Write Section 1 (Architecture) with ASCII diagrams embedded
3. Write Section 2 (Nodes) with code examples 1-2 inline, annotations
4. Write Section 3 (Communication) with code examples 3-6, comparison table
5. Write Section 4 (Agent Bridge) with code examples 7-8, safety notes
6. Write Section 5 (URDF) with complete URDF walkthrough, RViz/Gazebo instructions

**Step 4: Appendix Development**
1. Write "Deep Dive: DDS Architecture" section (conceptual, no code)
2. Write "QoS Policies Explained" with simple table-based examples
3. Write "Performance Tuning" with ros2 profiling tips
4. Write "Advanced Debugging" workflows with rqt tools
5. Write "Custom Messages" intro with simple .msg file example
6. Create 5-8 hands-on exercises building on main module concepts

**Step 5: Integration & Polish**
1. Ensure all terminology consistent (cross-reference with intro chapter glossary)
2. Add cross-references between sections (e.g., "see Section 3.2 for service details")
3. Verify all code examples have proper syntax highlighting (```python markers)
4. Embed ASCII diagrams at appropriate locations in content
5. Add "Further Reading" callouts for official ROS 2 docs

### Implementation Checklist

- [ ] Custom humanoid URDF created and tested in Gazebo
- [ ] All 10-12 code examples written, tested, and documented
- [ ] All 5-7 ASCII diagrams created and embedded
- [ ] Main module content (Sections 1-5) written in Markdown
- [ ] Appendix content written with 5-8 exercises
- [ ] Launch files created for all simulation scenarios
- [ ] Companion repository organized with examples/, urdf/, launch/, assets/
- [ ] All ros2 CLI verification commands tested and documented
- [ ] Cross-references and terminology consistency verified
- [ ] Docusaurus compatibility validated (frontmatter, markdown features)

## Phase 3: Validation & Testing

### Testing Strategy

**Level 1: Code Validation**
- Execute every Python code example in clean ROS 2 Humble environment
- Verify all ros2 CLI commands produce expected outputs
- Test URDF loads without errors in RViz and Gazebo
- Validate launch files start simulations successfully
- Document any dependency issues or environment requirements

**Level 2: Simulation Testing**
- Spawn custom humanoid in Gazebo, verify joint control
- Run publisher/subscriber examples, verify message flow with ros2 topic echo
- Test service examples, verify synchronous request/response
- Test action examples, verify goal/feedback/result lifecycle
- Run agent bridge examples, observe robot behavior in simulation
- Capture expected outputs (terminal logs, Gazebo screenshots if needed)

**Level 3: Constitution Compliance**
- Verify all code follows PEP 8 and ROS 2 conventions
- Check all comments explain "why" not "what"
- Verify no unsafe robot control patterns (velocity limits present)
- Confirm simulation-only approach (no hardware references)
- Validate terminology consistency with constitution glossary

**Level 4: Educational Quality**
- Read module as beginner: identify confusing sections
- Verify examples progress simple → complex
- Check all technical terms defined on first use
- Validate ASCII diagrams clearly illustrate concepts
- Test that module completion fits 4-hour target

### Test Environment

```yaml
Required Setup:
  OS: Ubuntu 22.04 LTS (or WSL2/Docker equivalent)
  ROS 2: Humble Hawksbill (primary), Iron Irwini (secondary validation)
  Gazebo: Classic 11.x
  Python: 3.10+
  Hardware: 8GB RAM, quad-core CPU, integrated graphics

Validation Commands:
  - ros2 run <package> <node>  # Test node execution
  - ros2 topic echo <topic>    # Verify message publishing
  - ros2 service call <service> <type> "{...}"  # Test services
  - ros2 action send_goal <action> <type> "{...}"  # Test actions
  - ros2 node info <node>      # Verify node structure
  - rqt_graph                  # Visualize computation graph
  - rviz2                      # Visualize robot model
  - gazebo                     # Run simulation
```

### Validation Checklist

- [ ] All code examples execute without errors on ROS 2 Humble
- [ ] All code examples execute without errors on ROS 2 Iron (compatibility check)
- [ ] Custom humanoid URDF loads in RViz without warnings
- [ ] Custom humanoid spawns in Gazebo with functional joint control
- [ ] All launch files start successfully without missing dependencies
- [ ] Publisher/subscriber example verified with ros2 topic echo
- [ ] Service example verified with ros2 service call
- [ ] Action example verified with lifecycle observation
- [ ] Agent bridge example moves robot in Gazebo as expected
- [ ] All ASCII diagrams render correctly in Markdown preview
- [ ] Module reads coherently for beginner audience (peer review)
- [ ] Terminology consistent with introduction chapter and constitution
- [ ] 100% constitution compliance (no safety violations, tested code)
- [ ] Estimated completion time ≤ 4 hours (student feedback or pilot test)

## Phase 4: Deployment Preparation

### Deliverables

**Primary Deliverable**: `docs/modules/ros2-nervous-system.md`
- Complete module content ready for Docusaurus deployment
- Frontmatter with metadata (title, description, sidebar position)
- All sections, code examples, diagrams, and appendix

**Supporting Deliverables**:
1. Companion GitHub repository: `ai-native-book-examples/ros2-module/`
   - `README.md` with setup instructions
   - `examples/` directory with all Python scripts
   - `urdf/` directory with SimpleHumanoid URDF
   - `launch/` directory with all launch files
   - `assets/` directory with ASCII diagrams (source files)

2. Documentation:
   - `SETUP.md`: ROS 2 + Gazebo installation guide
   - `TROUBLESHOOTING.md`: Common errors and solutions
   - `VERIFICATION.md`: How to verify each example works

3. Testing Artifacts:
   - Test logs from Humble and Iron validation
   - Screenshots of expected Gazebo outputs (optional)
   - Constitution compliance checklist (completed)

### Deployment Checklist

- [ ] Module markdown formatted for Docusaurus (correct frontmatter)
- [ ] All code examples tested in final repository structure
- [ ] Companion repository organized and documented
- [ ] README includes clear setup instructions for students
- [ ] TROUBLESHOOTING guide addresses common ROS 2 beginner issues
- [ ] All files committed to git with descriptive messages
- [ ] Module linked in textbook table of contents (site navigation)
- [ ] Peer review completed (robotics expert + student perspective)
- [ ] Constitution final compliance sign-off
- [ ] Ready for GitHub Pages deployment

## Risk Mitigation

### Identified Risks & Mitigations

| Risk | Impact | Mitigation Strategy |
|------|--------|---------------------|
| ROS 2 API changes between Humble/Iron break examples | High | Test on both distributions; use stable rclpy APIs only; document version-specific differences in notes |
| Custom URDF too complex for beginners to understand | Medium | Keep to 5-7 DOF; use box primitives; provide line-by-line annotations; test with beginner feedback |
| Gazebo performance issues on student hardware | Medium | Use lightweight robot model; minimal sensors; provide cloud simulation alternatives; document performance tuning |
| Code examples too verbose for textbook readability | Medium | Limit to 1-2 examples per topic; use "..." for boilerplate; full scripts in repository; highlight key lines |
| Students struggle with asynchronous ROS 2 concepts | High | Start with synchronous-style examples; gradually introduce callbacks; explain execution model clearly; debugging tips |
| URDF syntax intimidates students | Medium | Present as "configuration file" not "code"; focus on structure over syntax; visual aids; make section skippable if needed |
| Appendix exercises too difficult without guidance | Low | Provide solution outlines; include hints; design as "challenges" not "requirements"; make truly optional |

## Timeline Estimate

**Note**: Educational content creation typically requires iteration and review cycles.

- **Phase 0 (Research)**: 2-3 days
  - Day 1: ROS 2 educational resources audit, message type research
  - Day 2: Humanoid URDF design research, Gazebo validation
  - Day 3: Python agent patterns, compile research deliverables

- **Phase 1 (Design)**: 3-4 days
  - Day 1-2: Complete content outline, custom humanoid specification
  - Day 3: Code example templates, ASCII diagram designs
  - Day 4: Appendix exercises design, review and iterate

- **Phase 2 (Implementation)**: 5-7 days
  - Day 1: Custom URDF creation and testing
  - Day 2-3: Core code examples development (10-12 scripts)
  - Day 4-5: Module content writing (Sections 1-5)
  - Day 6: Appendix content and exercises
  - Day 7: Integration, polish, cross-references

- **Phase 3 (Validation)**: 2-3 days
  - Day 1: Code and simulation testing (all examples)
  - Day 2: Constitution compliance, educational quality review
  - Day 3: Iterate based on testing feedback, final validation

- **Phase 4 (Deployment Prep)**: 1-2 days
  - Day 1: Repository organization, documentation writing
  - Day 2: Final formatting, peer review, deployment checklist

**Total Estimated Time**: 13-19 days (2.5-4 weeks with review cycles)

## Success Criteria

Module is complete when:

1. ✅ All 23 functional requirements from spec.md are satisfied
2. ✅ All 12 success criteria from spec.md are met (students can create nodes, services, actions, etc.)
3. ✅ 100% of code examples execute successfully on ROS 2 Humble and Iron
4. ✅ Custom humanoid URDF loads and simulates correctly in Gazebo Classic 11
5. ✅ Constitution compliance validation passes (code quality, testing, safety, consistency)
6. ✅ Module content is Docusaurus-compatible and renders correctly
7. ✅ Companion repository is complete with setup documentation
8. ✅ Peer review (robotics expert + student) confirms beginner-friendliness
9. ✅ Estimated completion time ≤ 4 hours (validated with pilot students)
10. ✅ "Further Exploration" appendix provides clear path for advanced students

**Final Gate**: Module approved for publication when all success criteria met and constitution sign-off obtained.
