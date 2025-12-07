# Feature Specification: Chapter 2 / Module 1 - The Robotic Nervous System (ROS 2)

**Chapter**: 2
**Module**: 1 (The Robotic Nervous System)
**Feature Branch**: `002-ros2-module`
**Created**: 2025-12-07
**Status**: Draft
**Book Structure**: Chapter 2 of 5 (Intro → **ROS 2** → Digital Twin → Isaac → VLA)
**Prerequisites**: Chapter 1 (Introduction to Physical AI)
**Input**: User description: "ROS 2 module for robotics textbook covering nodes, topics, services, actions, URDF, and bridging Python agents to ROS controllers in simulation"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understanding ROS 2 Architecture (Priority: P1)

Students learn the fundamental ROS 2 architecture including the computation graph, nodes, topics, services, and actions through clear explanations and visual diagrams.

**Why this priority**: This is the conceptual foundation for all ROS 2 work. Without understanding the architecture and communication patterns, students cannot effectively design or debug robot systems. This knowledge is prerequisite for implementing any ROS 2 code.

**Independent Test**: Can be fully tested by asking students to draw or describe the ROS 2 computation graph for a simple robot scenario (e.g., camera publishing images to a processing node). Delivers architectural understanding needed for all subsequent ROS 2 development.

**Acceptance Scenarios**:

1. **Given** a student reading the ROS 2 Architecture Overview section, **When** they encounter the computation graph concept, **Then** they can explain how nodes communicate via topics, services, and actions
2. **Given** ASCII diagrams showing publisher/subscriber relationships, **When** students visualize message flow, **Then** they can identify which communication pattern (topic/service/action) is appropriate for different scenarios
3. **Given** explanations of ROS 2 middleware and DDS, **When** students compare to other communication systems, **Then** they understand why ROS 2 enables distributed robot control
4. **Given** real-world robot examples, **When** students map components to ROS 2 concepts, **Then** they can identify potential nodes, topics, and services in a humanoid robot system

---

### User Story 2 - Creating and Running Basic ROS 2 Nodes (Priority: P1)

Students write, run, and test their first ROS 2 Python nodes using rclpy, experiencing hands-on the publish/subscribe pattern with runnable code examples.

**Why this priority**: Practical implementation of the most fundamental ROS 2 pattern. Students must be able to create basic nodes before tackling more complex communication patterns. This is the "Hello World" of ROS 2 that enables all further learning.

**Independent Test**: Can be tested by having students create a simple publisher node that sends sensor data and a subscriber node that receives it, then verify message exchange using ros2 topic echo. Delivers hands-on ability to create functioning ROS 2 programs.

**Acceptance Scenarios**:

1. **Given** step-by-step Python code examples using rclpy, **When** students follow the instructions to create a publisher node, **Then** they can successfully publish messages to a topic and verify output with ros2 topic commands
2. **Given** a subscriber node example, **When** students implement a callback function to process incoming messages, **Then** they can receive and display data from a topic
3. **Given** instructions for ROS 2 package structure, **When** students organize their code files, **Then** they understand where to place Python nodes and how to make them executable
4. **Given** examples of sensor data publishing (simulated camera, IMU), **When** students run these examples in their environment, **Then** they see realistic robot data flowing through ROS 2 topics
5. **Given** debugging instructions, **When** students encounter common errors (import failures, topic name mismatches), **Then** they can diagnose and fix issues using ros2 node and topic commands

---

### User Story 3 - Implementing Services and Actions (Priority: P2)

Students learn request/response patterns via ROS 2 services and long-running task management via actions, with working code examples for each pattern.

**Why this priority**: Services and actions are essential for robot control (e.g., requesting inverse kinematics, executing navigation goals), but are less fundamental than basic pub/sub. Students can build simple systems with just topics, but need services/actions for realistic robot behaviors.

**Independent Test**: Can be tested by having students implement a service for a simple computation (e.g., calculate trajectory) and an action for a simulated long-running task (e.g., move robot arm to position). Delivers capability to use all three core ROS 2 communication patterns.

**Acceptance Scenarios**:

1. **Given** service examples with server and client code, **When** students implement a service for a robot capability (e.g., sensor calibration, position query), **Then** they can send requests and receive responses synchronously
2. **Given** explanations of when to use services vs topics, **When** students design a robot control system, **Then** they choose the appropriate communication pattern for each interaction
3. **Given** action examples with goal/feedback/result structure, **When** students implement an action for a time-consuming task, **Then** they can send goals, monitor progress via feedback, and receive final results
4. **Given** action client code, **When** students cancel an in-progress action, **Then** they understand action lifecycle management and can implement timeout/cancel behavior
5. **Given** comparisons of topics/services/actions, **When** students encounter a new robot requirement, **Then** they can justify which communication pattern to use

---

### User Story 4 - Bridging Python Agents to ROS Controllers (Priority: P2)

Students learn to connect Python-based AI agents or control logic to ROS 2 actuator controllers, enabling agent-driven robot behavior in simulation.

**Why this priority**: This bridges the gap between AI/agent logic and robot control, which is central to the textbook's "AI-native robotics" theme. Critical for later chapters on LLM integration, but students need ROS 2 fundamentals first.

**Independent Test**: Can be tested by having students create a Python agent that publishes velocity commands to a simulated robot and observes the robot move in Gazebo. Delivers the capability to control robots from high-level Python code.

**Acceptance Scenarios**:

1. **Given** examples of Python agent classes, **When** students implement an agent that publishes commands to ROS topics, **Then** they can control simulated robot actuators (wheels, joints) from agent logic
2. **Given** simulated robot environments in Gazebo, **When** students run agent-to-ROS bridge code, **Then** they observe the robot executing commands in real-time
3. **Given** explanations of controller interfaces, **When** students connect to position/velocity/effort controllers, **Then** they understand how to send different types of actuator commands
4. **Given** sensor feedback integration examples, **When** students implement agent decision-making based on ROS sensor topics, **Then** they create closed-loop agent-robot systems
5. **Given** safety and error handling patterns, **When** students implement agent-controller bridges, **Then** they include appropriate bounds checking and fallback behavior

---

### User Story 5 - Understanding URDF for Humanoid Robots (Priority: P3)

Students learn the Unified Robot Description Format (URDF) structure, including links, joints, and sensors, with humanoid robot examples they can visualize and modify.

**Why this priority**: URDF knowledge is important for working with robot models and simulation, but students can run existing robot models without deep URDF understanding initially. This is valuable context that enhances simulation work but isn't required for basic ROS 2 programming.

**Independent Test**: Can be tested by having students modify a simple URDF file (e.g., change link dimensions, add a sensor) and visualize the changes in RViz or Gazebo. Delivers ability to understand and modify robot descriptions.

**Acceptance Scenarios**:

1. **Given** URDF structure explanations with annotated examples, **When** students read a humanoid robot URDF file, **Then** they can identify links (body parts), joints (connections), and their properties
2. **Given** a small humanoid URDF snippet with explanations, **When** students analyze the robot structure, **Then** they understand parent-child link relationships and joint types (revolute, prismatic)
3. **Given** sensor definitions in URDF, **When** students examine camera and lidar tags, **Then** they can locate sensor specifications and understand how sensors are attached to robot links
4. **Given** visualization tools (RViz, Gazebo), **When** students load a URDF file, **Then** they can visualize the robot model and verify its structure matches the URDF definition
5. **Given** examples of modifying URDF parameters, **When** students adjust joint limits or link masses, **Then** they understand how URDF parameters affect simulation behavior

---

### Edge Cases

- What happens when students use different ROS 2 distributions (Humble vs Iron vs Jazzy)? (Provide version compatibility notes; examples tested on Humble and Iron; note API differences where relevant)
- How do students handle ROS 2 setup if they're on Windows vs Linux vs macOS? (Provide environment setup guidance; recommend Docker or WSL2 for Windows users; link to official ROS 2 installation docs)
- What if students have limited computational resources and Gazebo runs slowly? (Provide lightweight simulation alternatives; explain how to reduce physics complexity; offer headless simulation options)
- How do we handle students who want to test on real hardware? (Emphasize simulation-first approach per constitution; note that code examples work on real robots but recommend simulation for safety and accessibility)
- What if ROS 2 package dependencies conflict or fail to install? (Provide troubleshooting section; recommend using rosdep; include common error messages and solutions)
- How do students debug when messages aren't flowing between nodes? (Include debugging workflow using ros2 topic list/echo, ros2 node info, and rqt_graph visualization)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Module MUST provide a clear explanation of ROS 2 architecture including nodes, topics, services, and actions
- **FR-002**: Module MUST include ASCII or text-based diagrams illustrating the ROS 2 computation graph and message flow patterns
- **FR-003**: Module MUST provide step-by-step Python code examples using rclpy for creating publisher and subscriber nodes
- **FR-004**: Module MUST include complete, runnable examples of ROS 2 services with both server and client implementations
- **FR-005**: Module MUST demonstrate ROS 2 actions with goal, feedback, and result handling in Python
- **FR-006**: Module MUST explain when to use topics vs services vs actions with decision criteria and use cases
- **FR-007**: Module MUST show how to bridge Python agents to ROS 2 actuator controllers with working examples
- **FR-008**: Module MUST provide examples that run successfully in Gazebo Classic (version 11) using a custom simplified humanoid robot (5-7 DOF)
- **FR-009**: Module MUST explain URDF structure including links, joints, and sensor definitions using the custom simplified humanoid robot as the teaching example
- **FR-010**: Module MUST include the complete URDF for the custom simplified humanoid (torso, head, 2 arms, basic sensors) with detailed annotations explaining each component
- **FR-011**: All Python code examples MUST run without errors on ROS 2 Humble and Iron distributions
- **FR-012**: All code examples MUST include verification instructions (expected output, ros2 commands to test functionality)
- **FR-013**: Module MUST provide troubleshooting guidance for common ROS 2 setup and runtime errors
- **FR-014**: Module MUST follow the textbook schema with Title, Summary, Learning Outcomes, and organized Sections
- **FR-015**: Module MUST maintain consistency in terminology with the introduction chapter and constitution guidelines
- **FR-016**: Module MUST include only simulation-safe examples that do not risk hardware damage
- **FR-017**: All code examples MUST be tested and validated before publication per constitution requirements
- **FR-018**: Module MUST specify ROS 2 package dependencies and installation requirements clearly
- **FR-019**: Module MUST provide examples of integrating sensor data (camera, IMU, lidar) into ROS 2 nodes
- **FR-020**: Module MUST include instructions for visualizing ROS 2 computation graphs using rqt_graph or similar tools
- **FR-021**: Each major section (Nodes, Services, Actions, Agent Bridge, URDF) MUST include 1-2 complete, well-documented code examples inline with full annotations
- **FR-022**: Module MUST include a "Further Exploration" appendix with optional advanced topics: DDS architecture, QoS policies, performance tuning, and advanced debugging
- **FR-023**: Companion GitHub repository MUST be provided containing all code examples, custom humanoid URDF files, Gazebo launch scripts, and additional example variations

### Key Entities *(include if feature involves data)*

- **ROS 2 Node**: Fundamental computation unit in ROS 2; represents a single executable process that performs specific tasks; key attributes include node name, subscribed topics, published topics, provided services, and action servers
- **Topic**: Named communication channel for publish/subscribe messaging; characterized by topic name, message type, and Quality of Service (QoS) settings
- **Service**: Synchronous request/response communication pattern; defined by service name, request message type, response message type, and server node
- **Action**: Long-running task communication pattern with goal/feedback/result; includes action name, goal type, feedback type, result type, and lifecycle states
- **Message**: Data structure passed between nodes; defined by message type (e.g., sensor_msgs/Image, geometry_msgs/Twist) and field definitions
- **URDF Model**: XML-based robot description; contains links (rigid bodies), joints (connections between links), sensors, and physical properties (mass, inertia)
- **Package**: Organizational unit for ROS 2 code; contains Python/C++ nodes, message definitions, launch files, and configuration

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can create a functioning ROS 2 publisher node that sends messages and verify transmission using ros2 topic echo
- **SC-002**: Students can implement a subscriber node that processes incoming messages and demonstrates working callback functions
- **SC-003**: Students can implement a service server and client pair, successfully sending requests and receiving responses
- **SC-004**: Students can create an action that executes a multi-step task with progress feedback in simulation
- **SC-005**: Students can connect a Python agent to a simulated robot in Gazebo and observe the robot responding to agent commands
- **SC-006**: Students can read a URDF file and identify at least 5 key components (links, joints, sensors) and explain their purpose
- **SC-007**: 90% of students report the module is clear and examples are easy to follow (beginner-friendliness metric)
- **SC-008**: Students can choose the appropriate communication pattern (topic/service/action) for a given robot control scenario with 80%+ accuracy
- **SC-009**: All code examples pass automated testing on ROS 2 Humble and Iron with 100% success rate
- **SC-010**: Students complete the module exercises in under 4 hours of hands-on practice (paced learning metric)
- **SC-011**: Students can debug common ROS 2 issues using ros2 CLI tools after completing troubleshooting section
- **SC-012**: Module content aligns 100% with constitution requirements (code quality, testing, safety, consistency)

## Assumptions *(optional)*

- Students have completed the Introduction to Physical AI chapter and understand basic robotics concepts
- Students have basic Python programming skills (functions, classes, imports)
- Students have access to a computer with ROS 2 installed (Humble or Iron distribution) or can use Docker/WSL2
- Students have at least 8GB RAM and can run Gazebo simulation (or have access to cloud simulation resources)
- Students are working in a Linux environment (native, WSL2, or Docker container)
- Students have completed ROS 2 installation and can run basic ros2 commands (ros2 topic list, ros2 node list)
- The textbook will provide installation guidance in an appendix or setup chapter; this module assumes ROS 2 is already configured
- Students have internet access for installing ROS 2 packages via apt/pip
- Later chapters will build on ROS 2 knowledge for LLM integration, advanced control, and multi-robot systems

## Out of Scope *(optional)*

### Completely Out of Scope (Not in Module or Appendix)
- Lifecycle nodes and component composition - covered in advanced chapters if needed
- ROS 2 build systems (colcon, ament) beyond basic package structure - students use pre-built packages for this module
- C++ ROS 2 development - module focuses exclusively on Python (rclpy) for consistency with AI/agent development
- ROS 1 (legacy) or ROS 1-to-ROS 2 bridge - textbook is ROS 2 native
- Multi-robot systems and namespacing - covered in later chapters
- ROS 2 security (SROS2) - important but deferred to security-focused chapters
- Real hardware interfacing - all examples simulation-based per constitution safety requirements
- Gazebo Ignition/Isaac Sim - using Gazebo Classic (version 11) exclusively for consistency

### Moved to Appendix (Previously Out of Scope, Now Optional)
- Detailed QoS (Quality of Service) configuration - now in "Further Exploration" appendix
- DDS architecture and vendor comparisons - now in appendix
- Performance optimization techniques - now in appendix
- Custom message/service definitions (introduction only) - now in appendix
- Advanced debugging workflows - now in appendix

## Dependencies *(optional)*

- Requires Introduction to Physical AI chapter (001-physical-ai-intro) to be completed first
- Requires constitution.md robotics content generation rules and testing standards
- Assumes ROS 2 Humble or Iron distribution is installed (installation guide may be separate chapter/appendix)
- Requires Gazebo Classic (version 11) for all simulation examples
- Requires creation of custom simplified humanoid URDF model (5-7 DOF: torso, head, 2 arms) with basic sensors (camera, IMU, joint encoders)
- May reference rqt_graph, RViz, and other ROS 2 visualization tools (installation instructions needed)
- Assumes Python 3.8+ environment with rclpy and standard ROS 2 Python packages
- Later chapters will depend on this module's ROS 2 foundation (LLM integration, navigation, manipulation)

## Risks *(optional)*

- **Risk**: ROS 2 rapid development means API changes between distributions could break examples
  - **Mitigation**: Test on both Humble (LTS) and Iron; document version-specific differences; use stable APIs; plan for updates with each ROS 2 LTS release

- **Risk**: Gazebo simulation may be too resource-intensive for some student computers
  - **Mitigation**: Provide lightweight simulation options; include instructions for reducing graphics quality; offer cloud simulation alternatives (AWS RoboMaker, Google Cloud); test examples on mid-range hardware (8GB RAM)

- **Risk**: ROS 2 installation and environment setup can be frustrating for beginners on non-Linux systems
  - **Mitigation**: Provide Docker containers with pre-configured environments; recommend WSL2 for Windows; include comprehensive troubleshooting section; link to official ROS 2 installation docs

- **Risk**: Students may struggle with asynchronous programming concepts (callbacks, threading) inherent in ROS 2
  - **Mitigation**: Start with simple synchronous-style examples; gradually introduce async concepts; provide clear explanations of callback execution; include debugging tips for race conditions

- **Risk**: Balancing code completeness (runnable examples) with brevity (textbook readability) is challenging
  - **Mitigation**: Use code snippets with annotations in main text; provide full scripts in appendix or companion repository; highlight key lines in examples; use "..." to indicate boilerplate

- **Risk**: URDF syntax can be intimidating and dry for students
  - **Mitigation**: Use simplified humanoid examples; focus on intuition over exhaustive reference; provide visual aids; make URDF section optional/reference material rather than prerequisite

## Clarifications Resolved *(from /sp.clarify)*

### Code Example Structure (Clarified 2025-12-07)

**Decision**: Use 1-2 complete, well-documented examples per major topic
- Each major section (Nodes, Services, Actions, Agent Bridge, URDF) includes 1-2 complete, runnable code examples inline
- Examples prioritize clarity and modularity over brevity
- Additional variations and advanced examples provided in companion GitHub repository
- All inline code is fully annotated with comments explaining key concepts
- Each example includes verification instructions using ros2 CLI tools

**Rationale**: Keeps module readable and focused while ensuring students have working reference implementations. Companion repository allows exploration without cluttering main content.

### Simulation Environment (Clarified 2025-12-07)

**Decision**: Gazebo Classic with Simple Custom Humanoid Robot
- **Simulator**: Gazebo Classic (version 11) for stability and broad compatibility
- **Robot Model**: Custom simplified humanoid with 5-7 DOF (degrees of freedom)
  - Components: torso (base), head (1 DOF pan), 2 arms (2-3 DOF each)
  - Total: ~5-7 joints, manageable for beginners
  - URDF created from scratch as teaching example
  - Includes basic sensors: camera (head), IMU (torso), joint encoders
- Simple wheeled mobile base optional for initial pub/sub examples before introducing humanoid complexity

**Rationale**: Gazebo Classic provides stable, well-documented environment. Custom simplified humanoid allows teaching URDF from scratch without overwhelming students with full 20+ DOF robot complexity. Easier to visualize and understand than complex existing models like OP3.

### Advanced Topics Placement (Clarified 2025-12-07)

**Decision**: Include advanced topics in module appendix exercises
- Main module sections remain beginner-friendly with conceptual DDS/QoS overview
- Module appendix titled "Further Exploration" includes:
  - Deep dive into DDS architecture and vendors
  - QoS policy exploration (reliability, durability, history, lifespan, deadline)
  - Performance tuning and optimization techniques
  - Advanced debugging workflows
  - Custom message/service definitions (brief introduction)
- Appendix clearly marked as optional for motivated students
- Appendix exercises include hands-on challenges building on main module concepts

**Rationale**: Keeps main module accessible to beginners while providing growth path for advanced students. All ROS 2 content consolidated in one module rather than deferring to separate advanced chapters.

## Notes *(optional)*

- This module is the technical foundation for all ROS 2 work in the textbook; quality and clarity are critical
- Companion GitHub repository MUST include all code examples, custom humanoid URDF files, and Gazebo launch scripts
- ASCII diagrams for ROS 2 architecture should be clear and simple; avoid overly complex graph visualizations
- Code examples should prioritize readability over optimal ROS 2 practices (e.g., verbose naming, extra comments)
- Include "Further Reading" callouts for students who want deeper understanding of DDS, QoS, or ROS 2 internals
- Test all examples in a clean ROS 2 Humble environment with Gazebo Classic 11 to catch missing dependencies
- Custom humanoid URDF should be versioned and documented with design rationale
- Consider creating short demo videos showing expected output of each major example (optional supplement)
- This specification intentionally avoids implementation details (file formats for publishing, specific simulation plugins) per constitution guidelines
- Ensure consistency in terminology: "ROS 2 node" not "ROS2 node", "topic" not "channel", "service" not "RPC"
