# ADR-004: Platform Support Strategy for Digital Twin Module

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 003-digital-twin-module
- **Context:** Module 2 targets a diverse student population with varying operating systems (Windows, macOS, Linux) and hardware constraints (8GB RAM minimum, integrated graphics common). ROS 2 and Gazebo Classic are primarily developed for Ubuntu Linux, which creates platform accessibility challenges. A decision was needed to balance accessibility (support all platforms), simplicity (single environment), and reliability (minimize platform-specific bugs).

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - affects student onboarding, setup complexity, and code portability
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - native Linux only, VM-based, cloud-based, multi-platform all evaluated
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - impacts SETUP.md, all launch files, code examples, validation testing, performance requirements
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

We will support **three platform configurations** as first-class targets:

- **Primary Platform: Ubuntu 22.04 LTS (Native)**
  - ROS 2 Humble Hawksbill + Gazebo Classic 11.x installed via apt
  - Best performance (direct hardware access)
  - Recommended for students with existing Linux systems or willing to dual-boot

- **Windows Support: WSL2 with Ubuntu 22.04**
  - Windows Subsystem for Linux 2 with Ubuntu 22.04 distribution
  - ROS 2 Humble + Gazebo Classic 11.x installed within WSL2
  - GUI rendering via X server (VcXsrv, X410) or WSLg (Windows 11)
  - Good performance (near-native), familiar OS for many students

- **macOS Support: Docker with ROS 2 Humble Image**
  - Official `ros:humble` Docker image + Gazebo Classic 11.x
  - X11 forwarding via XQuartz for Gazebo GUI and RViz
  - Acceptable performance (containerized), works on Intel and Apple Silicon (Rosetta 2)

- **Validation Strategy:**
  - All code examples tested on all three platforms before release
  - SETUP.md includes platform-specific installation guides
  - TROUBLESHOOTING.md documents common platform-specific issues

## Consequences

### Positive

- **Maximum Accessibility:** Students on Windows, macOS, and Linux can all complete module (no platform exclusion)
- **Low Hardware Barrier:** WSL2 and Docker work on 8GB RAM systems with integrated graphics (no GPU required)
- **Consistent ROS 2 Environment:** All platforms run same ROS 2 Humble + Gazebo 11.x versions (reduces "works on my machine" issues)
- **No Dual-Boot Required:** Windows/macOS users can keep their primary OS (lowers friction for beginners)
- **Standard Tools:** WSL2 and Docker are industry-standard developer tools (students learn transferable skills)
- **Future-Proof:** Docker and WSL2 track upstream Ubuntu/ROS releases (long-term viability)

### Negative

- **Complexity:** SETUP.md must document three platform workflows (more documentation to maintain)
- **Testing Overhead:** All code changes require validation on Ubuntu, WSL2, and Docker (3x testing effort)
- **Performance Variance:** Docker on macOS slower than native Ubuntu (may not hit ≥0.5x real-time on older Macs)
- **GUI Setup Friction:** WSL2 X server and macOS XQuartz add extra setup steps (potential student confusion)
- **Platform-Specific Bugs:** Different platforms may expose Gazebo or ROS 2 bugs (e.g., WSL2 GPU passthrough issues)
- **Resource Fragmentation:** Supporting three platforms dilutes troubleshooting effort (harder to deeply optimize any one)

## Alternatives Considered

### Alternative Strategy A: Ubuntu Native Only
- **Components:**
  - Only support Ubuntu 22.04 LTS (native installation)
  - Require students to dual-boot or use dedicated Linux machine
  - No WSL2 or Docker support
- **Why Rejected:**
  - Excludes students without Linux machines or dual-boot capability (high barrier to entry)
  - Dual-booting is intimidating for beginners (risk of data loss, boot issues)
  - Reduces module accessibility (contradicts "beginner-friendly" goal from spec)
  - Many students on Windows/macOS would be blocked from completing module

### Alternative Strategy B: Virtual Machine (VirtualBox, VMware)
- **Components:**
  - Official VM image with Ubuntu 22.04 + ROS 2 + Gazebo pre-installed
  - Students download VM image, import to VirtualBox/VMware
- **Why Rejected:**
  - Poor performance (VM overhead ~20-30%, may not achieve ≥0.5x real-time on 8GB RAM)
  - Large download size (10-20GB VM image, excludes students with limited bandwidth)
  - GPU passthrough complex/unreliable in VMs (Gazebo rendering issues)
  - VM maintenance burden (updating image for ROS 2 patches, Gazebo updates)

### Alternative Strategy C: Cloud-Based Simulation (AWS RoboMaker, Google Colab)
- **Components:**
  - Cloud-hosted Gazebo simulations accessed via web browser
  - No local installation required
  - Students run code examples in cloud notebooks
- **Why Rejected:**
  - Requires stable internet (excludes students with unreliable connectivity)
  - Cost barrier (AWS RoboMaker charges per simulation hour, not free for students)
  - Limited customization (cloud platforms restrict URDF/world file uploads)
  - Doesn't teach local development skills (students can't run simulations on own hardware)
  - Dependency on third-party service availability (platform outages block learning)

### Alternative Strategy D: Multi-Platform Native Support (Ubuntu, Windows, macOS)
- **Components:**
  - Native ROS 2 installation on Ubuntu, Windows (ROS 2 Windows binaries), macOS (Homebrew)
  - Native Gazebo installation on all platforms
- **Why Rejected:**
  - ROS 2 Windows support incomplete (many packages unavailable, Gazebo integration poor)
  - macOS ROS 2 support experimental (no official apt packages, brittle Homebrew builds)
  - Testing nightmare (three completely different toolchains, dependency paths, build systems)
  - High maintenance burden (platform-specific code patches, brittle cross-platform compatibility)

## References

- Feature Spec: `specs/003-digital-twin-module/spec.md` (NFR-015: Accessibility)
- Implementation Plan: `specs/003-digital-twin-module/plan.md` (Technical Context: Target Platform)
- Related ADRs: ADR-001 (Simulation Platform Stack - ROS 2 Humble + Gazebo Classic)
- Evaluator Evidence: None (initial design decision)
- [WSL2 Installation Guide](https://learn.microsoft.com/en-us/windows/wsl/install)
- [Docker Official Images: ROS](https://hub.docker.com/_/ros)
