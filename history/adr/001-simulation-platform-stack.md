# ADR-001: Simulation Platform Stack for Digital Twin Module

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 003-digital-twin-module
- **Context:** Module 2 requires a simulation platform for teaching students Digital Twin concepts for humanoid robotics. The platform must support physics simulation, sensor integration (LiDAR, Depth Camera, IMU), ROS 2 integration, and run on standard student hardware (8GB RAM minimum). Students are beginners who completed Module 1 (ROS 2 basics) and need a stable, well-documented simulation environment that works across Ubuntu, WSL2, and Docker.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines entire technical foundation for module
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Gazebo Ignition, Isaac Sim, Webots, MuJoCo all considered
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects all code examples, dependencies, student setup, performance requirements
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

We will use the following integrated simulation stack:

- **Physics Simulator:** Gazebo Classic 11.x
- **ROS Distribution:** ROS 2 Humble Hawksbill (primary), Iron Irwini (compatibility tested)
- **Integration Layer:** `gazebo_ros_pkgs` for Gazebo-ROS 2 bridge
- **Programming Language:** Python 3.10+ for ROS 2 nodes
- **Target Platforms:** Ubuntu 22.04 LTS (native), WSL2 with Ubuntu 22.04, Docker with ROS 2 Humble image
- **Performance Target:** ≥0.5x real-time factor on 8GB RAM systems

## Consequences

### Positive

- **Mature and Stable:** Gazebo Classic 11 has extensive documentation, long-term stability, and proven ROS 2 Humble integration
- **Wide Adoption:** Large community, abundant tutorials, established patterns for education
- **Hardware Accessibility:** Runs on integrated graphics and 8GB RAM systems (critical for student access)
- **Deterministic Simulation:** Fixed random seeds enable reproducible experiments for grading and debugging
- **Beginner-Friendly:** Comprehensive error messages, visual debugging (Gazebo GUI), familiar to educators
- **Strong ROS 2 Integration:** `gazebo_ros_pkgs` provides battle-tested sensor plugins (LiDAR, Depth, IMU) with standard ROS message types
- **Multi-Platform Support:** Works on Ubuntu, WSL2, Docker without major code changes

### Negative

- **Deprecation Risk:** Gazebo Classic is being phased out in favor of Gazebo Ignition/Fortress (though Classic is still maintained)
- **Limited Graphics Fidelity:** Physics-focused, not photorealistic (why Unity teaser is included)
- **Performance Constraints:** Real-time factor <1.0 on low-end hardware may slow demonstrations
- **Plugin Learning Curve:** XML-based sensor configuration in URDF can be verbose for beginners
- **Version Lock-in:** Tight coupling to Gazebo Classic 11 and ROS 2 Humble means migration effort if ecosystem shifts

## Alternatives Considered

### Alternative Stack A: Gazebo Ignition/Fortress + ROS 2 Humble
- **Components:** Gazebo Ignition (modern version), same ROS 2 Humble, `ros_gz` bridge
- **Why Rejected:** Less stable ROS 2 integration (ros_gz still maturing), fewer tutorials for beginners, higher system requirements, documentation gaps for educational use cases

### Alternative Stack B: Isaac Sim + ROS 2
- **Components:** NVIDIA Isaac Sim, ROS 2 Humble, Isaac Sim ROS 2 bridge
- **Why Rejected:** Requires NVIDIA GPU (excludes students with integrated graphics), steep learning curve, heavy system requirements (16GB+ RAM), proprietary platform

### Alternative Stack C: Webots + ROS 2
- **Components:** Webots R2023b, ROS 2 Humble
- **Why Rejected:** Smaller community than Gazebo, less documentation for humanoid robotics, weaker sensor plugin ecosystem, unfamiliar to most robotics educators

### Alternative Stack D: MuJoCo + ROS 2
- **Components:** MuJoCo (DeepMind), custom ROS 2 bindings
- **Why Rejected:** No native ROS 2 integration (requires custom bridge), physics-focused but lacks Gazebo's robotics-specific tooling (world files, GUI), primarily used for RL research not education

## References

- Feature Spec: `specs/003-digital-twin-module/spec.md`
- Implementation Plan: `specs/003-digital-twin-module/plan.md`
- Related ADRs: ADR-002 (Sensor Architecture), ADR-003 (Unity Integration Approach)
- Evaluator Evidence: None (initial design decision)
- [Gazebo Classic Documentation](http://classic.gazebosim.org/tutorials)
- [ROS 2 Humble + Gazebo Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
