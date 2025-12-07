# ADR-003: Unity Integration Approach for Digital Twin Module

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 003-digital-twin-module
- **Context:** Module 2 initially planned to cover both Gazebo (physics simulation) and Unity (high-fidelity visualization) in depth. During spec clarification, risk analysis revealed the module would be too long (6+ hours) and complex for beginners. Unity simulation has a steep learning curve, unstable ROS 2 bridge integration, and serves a different purpose (visualization/rendering) than Gazebo (physics). A decision was needed: (1) deep Unity integration, (2) brief teaser with external resources, or (3) omit Unity entirely.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines scope of Module 2, affects future module planning (Module 2B)
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - full integration, teaser, omission all evaluated
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects module length, content structure, student prerequisites, file structure (unity/ directory)
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

We will use a **teaser-only approach for Unity** with the following structure:

- **Content Strategy:**
  - Unity coverage limited to 1-2 pages in Module 2 (Priority P3)
  - Content explains Unity vs Gazebo trade-offs (graphics quality vs physics accuracy)
  - No Unity setup, installation, or hands-on tutorial in Module 2

- **External Resource Links:**
  - Link to official Unity 2021 LTS installation guides (platform-specific)
  - Link to Unity Robotics Hub (`ros-tcp-connector` for ROS 2 integration)
  - Link to URDF Importer package for importing robot models
  - Link to external tutorials for basic Unity-ROS 2 visualization setup

- **File Structure:**
  - `examples/digital-twin-module/unity/README.md` contains curated links and high-level steps
  - No Unity project files, scripts, or scenes in Module 2 repository

- **Future Module:**
  - Full Unity deep dive deferred to **Module 2B** (separate feature, separate timeline)
  - Module 2B will cover Unity installation, scene creation, URDF import, ROS 2-Unity bridge, and visualization examples

## Consequences

### Positive

- **Focused Scope:** Module 2 stays within 4-5 hour completion target (Gazebo focus)
- **Reduced Complexity:** Students learn one simulation platform thoroughly before introducing second platform
- **Beginner-Friendly:** No Unity prerequisites, installation troubleshooting, or dependency conflicts in Module 2
- **Lower Risk:** Avoids unstable Unity-ROS 2 bridge issues blocking student progress in Module 2
- **Flexibility:** Students interested in Unity can self-study via external links; those not interested skip cleanly
- **Clear Roadmap:** Module 2B provides clear path for advanced visualization (students know what's coming)
- **Reduced Maintenance:** No Unity code to maintain, update, or debug in Module 2 repository

### Negative

- **Incomplete Visualization Story:** Students don't get hands-on Unity experience in Module 2 (must wait for 2B)
- **External Dependency Risk:** Links to external Unity tutorials may break or become outdated (requires periodic validation)
- **Fragmented Learning Path:** Students must complete two modules (2 + 2B) for full simulation coverage (vs single comprehensive module)
- **No Unity Code Examples:** Students can't run Unity examples immediately after Module 2 (must self-study or wait for 2B)
- **Potential Student Frustration:** "Teaser" may feel incomplete if students are excited about Unity but can't practice

## Alternatives Considered

### Alternative Approach A: Full Unity Integration in Module 2
- **Components:**
  - Unity 2021 LTS installation tutorial (all platforms)
  - Unity project with imported SimpleHumanoid URDF
  - ROS 2-Unity bridge setup (`ros-tcp-connector`)
  - Unity scene with robot visualization
  - Example: Sending `/joint_states` to Unity for real-time robot rendering
- **Why Rejected:**
  - Module length balloons to 6-8 hours (too long for single module)
  - High complexity for beginners (two platforms simultaneously)
  - Unity-ROS 2 bridge (`ros-tcp-connector`) has installation issues on some systems (medium risk per spec)
  - Students without GPUs may struggle with Unity (8GB RAM systems have integrated graphics)
  - Maintenance burden increases (Gazebo + Unity code to keep updated)

### Alternative Approach B: Omit Unity Entirely
- **Components:**
  - No Unity content in Module 2
  - No future Module 2B planned
  - Focus 100% on Gazebo for all simulation needs
- **Why Rejected:**
  - Unity is industry-relevant for robot visualization, demos, and human-robot interaction (educational value)
  - Spec explicitly requested Unity coverage (omission doesn't meet requirements)
  - Students lose exposure to high-fidelity rendering (important for presentations, research)
  - Limits career readiness (Unity used in HRI, surgical robotics, entertainment robotics)

### Alternative Approach C: Unity-Only Module (Replace Gazebo)
- **Components:**
  - Use Unity for both physics and visualization
  - Unity Physics engine for simulation
  - No Gazebo content
- **Why Rejected:**
  - Unity physics less mature than Gazebo for robotics (not designed for sensor-accurate physics)
  - ROS 2 integration weaker in Unity vs Gazebo (smaller community, fewer plugins)
  - Performance worse on low-end hardware (Unity requires GPU for physics+rendering)
  - Gazebo is industry standard for robotics simulation (omitting it hurts student employability)

## References

- Feature Spec: `specs/003-digital-twin-module/spec.md` (Clarifications section, 2025-12-07 session)
- Implementation Plan: `specs/003-digital-twin-module/plan.md`
- Related ADRs: ADR-001 (Simulation Platform Stack - Gazebo primary platform)
- Evaluator Evidence: None (design decision from spec clarification session)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Unity 2021 LTS](https://unity.com/releases/lts)
