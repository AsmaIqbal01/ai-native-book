---
id: 003
title: Plan ROS 2 Module Implementation
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: ros2-module
branch: main
user: umzaid
command: /sp.plan
labels: ["planning", "implementation", "ros2", "urdf", "code-examples", "simulation"]
links:
  spec: specs/002-ros2-module/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-ros2-module/plan.md
tests:
 - Constitution check passed (all requirements align)
 - Custom humanoid URDF design specified (5-7 DOF)
 - 10-12 code examples planned with verification steps
---

## Prompt

/sp.plan
id: module1_plan
title: "Plan for Module 1: The Robotic Nervous System (ROS 2)"
type: planning

instructions:
  - Break the module into actionable steps for content generation.
  - Include planning for sections, code examples, simulations, diagrams, and validations.
  - Ensure the plan aligns with the constitution and spec rules.
  - Prioritize beginner-friendly explanations while keeping examples runnable.

context:
  - Module 1 teaches ROS 2 fundamentals for humanoid robots.
  - Topics: nodes, topics, services, actions, Python (rclpy), URDF.
  - Must be safe for simulation; no dangerous physical robot instructions.
  - Output will be used in Docusaurus as `docs/module1.md`.

plan_steps:
  1. Outline the module structure
  2-6. Plan each section with code examples and diagrams
  7. Validation & Review workflow
  8. Deliverable preparation

## Response snapshot

Created comprehensive implementation plan for ROS 2 module with 4 phases (Research, Design, Implementation, Validation) and complete technical specifications.

### Plan Highlights

**Technical Context Defined**:
- ROS 2 Humble/Iron with rclpy, Gazebo Classic 11, Python 3.8+
- Educational content module (~8,000-10,000 words)
- 10-12 complete code examples, 1 custom humanoid URDF (~200-300 lines)
- 5-7 ASCII diagrams for architecture visualization
- Performance goals: runs on 8GB RAM, 4-hour completion time
- 100% simulation-based (no hardware risk)

**Constitution Check**: ✅ PASSED
- All code quality, testing, safety, consistency requirements aligned
- Simulation-first approach, beginner-accessible language
- Manual testing with ros2 CLI validation
- Modular code with clear comments

**Custom Humanoid URDF Design**:
```yaml
Name: SimpleHumanoid
DOF: 6 (1 head pan, 2x2 arm joints)
Links: 5 (torso, head, 2x upper/forearm)
Sensors: IMU (torso), camera (head), joint encoders
Joint types: revolute only (single-axis for simplicity)
Primitives: boxes (no mesh dependencies)
Controllers: position_controllers + joint_state_controller
```

**Rationale**: 6 DOF is manageable for beginners, symmetric arms teach patterns, single-axis joints easier to visualize, essential sensors only.

**Content Structure**:
```
Module: The Robotic Nervous System (ROS 2)
├── Summary & Learning Outcomes
├── Section 1: ROS 2 Architecture Overview
│   ├── What is ROS 2? (middleware)
│   ├── Computation Graph (nodes, topics, services, actions)
│   ├── DDS and Distributed Systems (conceptual)
│   └── Communication Pattern Decision Tree
├── Section 2: Creating and Running ROS 2 Nodes
│   ├── ROS 2 Package Structure
│   ├── Publisher Node (Example 1)
│   ├── Subscriber Node (Example 2)
│   └── Debugging with ros2 CLI
├── Section 3: Topics, Services, and Actions
│   ├── Topics: Publish/Subscribe (sensor example)
│   ├── Services: Request/Response (calculator example)
│   ├── Actions: Long-Running Tasks (arm movement example)
│   └── Pattern Comparison Table
├── Section 4: Bridging Python Agents to ROS Controllers
│   ├── Python Agent Structure (class-based)
│   ├── Publishing Commands (velocity control)
│   ├── Subscribing to Sensors (closed-loop)
│   └── Safety Patterns
├── Section 5: Understanding URDF for Humanoid Robots
│   ├── URDF Structure: Links and Joints
│   ├── Custom Simplified Humanoid (complete walkthrough)
│   ├── Sensors in URDF
│   └── Visualizing in RViz and Gazebo
└── Further Exploration (Appendix)
    ├── DDS Architecture Deep Dive
    ├── QoS Policies Explained
    ├── Performance Tuning
    ├── Advanced Debugging
    └── Custom Messages Introduction
```

**Code Examples Planned** (10-12 total):
1. **simple_publisher.py**: Basic topic publishing with strings
2. **simple_subscriber.py**: Basic topic subscription with callback
3. **sensor_publisher.py**: Realistic sensor data publishing
4. **calculator_server.py**: Service server (addition example)
5. **calculator_client.py**: Service client calling server
6. **move_arm_server.py**: Action server (arm movement simulation)
7. **move_arm_client.py**: Action client with goal/feedback/result
8. **simple_agent.py**: Python agent commanding robot (open-loop)
9. **closed_loop_agent.py**: Agent with sensor feedback integration
10. **simple_humanoid.urdf**: Complete custom humanoid URDF with annotations

Each example includes:
- Purpose comment
- Verification command (ros2 CLI)
- Full implementation with inline comments
- Expected output documentation

**ASCII Diagram Designs**:
1. ROS 2 Computation Graph (nodes, topics, services)
2. Topic Publish/Subscribe Flow
3. Service Request/Response Pattern
4. Action Lifecycle States
5. Agent-Controller Bridge Architecture
6. Humanoid URDF Structure Tree
7. (Optional) DDS Communication Stack

**4-Phase Implementation Workflow**:

**Phase 0: Research & Discovery** (2-3 days)
- Audit ROS 2 educational best practices
- Research humanoid URDF designs (5-7 DOF)
- Study Python agent patterns for robot control
- Compile common beginner mistakes and solutions
- Deliverable: research.md with findings

**Phase 1: Design & Architecture** (3-4 days)
- Complete module content outline (all sections)
- Finalize custom humanoid URDF specification
- Design all code example templates
- Create ASCII diagram designs
- Plan appendix exercises (5-8 challenges)
- Deliverables: design-document.md, custom-humanoid-spec.yaml, code-examples-spec.md, ascii-diagrams.txt

**Phase 2: Implementation** (5-7 days)
- Create custom_humanoid.urdf and test in Gazebo
- Implement all 10-12 code examples with verification
- Write module content (Sections 1-5) in Markdown
- Develop appendix content and exercises
- Create launch files for simulation demos
- Ensure terminology consistency and cross-references

**Phase 3: Validation & Testing** (2-3 days)
- Execute all code examples in clean ROS 2 Humble environment
- Test URDF in RViz and Gazebo
- Verify all ros2 CLI commands work as documented
- Constitution compliance check (code quality, safety, testing)
- Educational quality review (beginner-friendliness)
- Pilot test: confirm 4-hour completion time

**Phase 4: Deployment Preparation** (1-2 days)
- Format module for Docusaurus (frontmatter, markdown)
- Organize companion GitHub repository
- Write SETUP.md, TROUBLESHOOTING.md, README.md
- Final peer review (expert + student perspective)
- Deployment checklist completion

**Validation Strategy**:
- **Level 1**: Code executes on ROS 2 Humble/Iron without errors
- **Level 2**: Simulations produce expected behavior in Gazebo
- **Level 3**: Constitution compliance (safety, testing, consistency)
- **Level 4**: Educational quality (beginner-friendly, 4-hour target)

**Test Environment**:
```
OS: Ubuntu 22.04 / WSL2
ROS 2: Humble (primary), Iron (secondary)
Gazebo: Classic 11.x
Python: 3.10+
Hardware: 8GB RAM, quad-core, integrated graphics
```

**Success Criteria** (10 gates):
1. All 23 functional requirements satisfied
2. All 12 success criteria met (students can create nodes, etc.)
3. 100% code execution on Humble and Iron
4. Custom humanoid simulates correctly in Gazebo
5. Constitution compliance passed
6. Docusaurus-compatible markdown
7. Companion repository complete
8. Peer review confirms beginner-friendliness
9. Completion time ≤ 4 hours validated
10. Appendix provides clear advanced path

**Risk Mitigation**:
- ROS 2 API changes: Test on Humble + Iron, use stable APIs
- URDF complexity: Keep 5-7 DOF, box primitives, line-by-line annotations
- Gazebo performance: Lightweight model, cloud alternatives, tuning docs
- Code verbosity: 1-2 examples per topic, full scripts in repo
- Async concepts: Start synchronous, gradual introduction, clear execution model
- URDF intimidation: Present as "config", focus structure, visual aids, skippable
- Appendix difficulty: Solution outlines, hints, truly optional

**Timeline Estimate**: 13-19 days (2.5-4 weeks with review cycles)

## Outcome

- ✅ Impact: Complete implementation plan ready for ROS 2 module generation; clear phased workflow from research to deployment
- 🧪 Tests: Constitution check passed; all requirements align with safety, testing, consistency principles
- 📁 Files: Created specs/002-ros2-module/plan.md (616 lines) with technical context, 4-phase workflow, code templates, URDF design
- 🔁 Next prompts: Execute Phase 0 research, then proceed with `/sp.tasks` to break plan into granular implementation tasks
- 🧠 Reflection: Plan successfully balances educational quality with technical rigor; custom humanoid design (6 DOF) provides optimal learning complexity; phased approach ensures validation at each stage

## Evaluation notes (flywheel)

- Failure modes observed: None; plan comprehensively addresses specification requirements and clarification decisions
- Graders run and results (PASS/FAIL): Constitution check PASSED (all 5 principle categories aligned)
- Prompt variant (if applicable): Standard /sp.plan workflow adapted for educational content (not traditional software); template modified to focus on content structure vs. code architecture
- Next experiment (smallest change to try): Consider creating visual mockup of custom humanoid before URDF implementation; ASCII representation in plan may benefit from 3D visualization tool output; also consider pre-validation of Gazebo Classic 11 compatibility with ROS 2 Iron distribution before full implementation
