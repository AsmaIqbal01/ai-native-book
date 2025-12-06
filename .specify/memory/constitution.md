# AI-Native Robotics Textbook Constitution

<!--
SYNC IMPACT REPORT
==================
Version Change: 0.0.0 → 1.0.0
Created Date: 2025-12-07
Constitution Type: Initial creation for AI-Native Robotics Textbook project

Principles Created:
- I. Code Quality & Best Practices (Robotics Focus)
- II. Testing & Validation Standards
- III. User Experience & Consistency
- IV. Performance & Accessibility

Additional Sections:
- Content Generation Rules
- Project Structure & Versioning
- Safety Requirements
- Deployment & Publication
- Governance

Templates Status:
✅ plan-template.md - Constitution Check section aligns with new principles
✅ spec-template.md - Requirements structure supports robotics/simulation context
✅ tasks-template.md - Task categorization compatible with new principles
⚠ Commands in .claude/commands/ - May need review for robotics-specific workflows

Follow-up TODOs: None - All placeholders filled

Rationale for MAJOR version (1.0.0):
- Initial constitution establishment
- Defines core architectural constraints (ROS 2, simulation, AI/LLM integration)
- Sets non-negotiable testing and validation requirements
- Establishes safety-first approach for robotics content
-->

## Core Principles

### I. Code Quality & Best Practices (Robotics Focus)

Code examples MUST be correct, runnable, and follow best practices for Python, ROS 2, and simulation tools (Gazebo, Webots).

**Requirements:**
- All code MUST run without errors when executed in the specified environment
- Python code MUST follow PEP 8 style guidelines
- ROS 2 code MUST follow ROS 2 conventions (node naming, topic structure, parameter handling)
- Code MUST use consistent naming conventions: snake_case for Python/ROS 2, clear descriptive names
- Code MUST be modular with single-responsibility functions and classes
- Code MUST minimize side effects and global state
- Comments MUST be included for robotics-specific logic, simulation parameters, and agent-control algorithms
- Comments MUST explain "why" rather than "what" (code should be self-documenting for "what")
- Complexity MUST be avoided unless justified by educational value; prioritize readability over cleverness
- All dependencies MUST be explicitly declared with version constraints where critical

**Rationale:** Students learning robotics need reliable, clear examples. Broken or unclear code damages trust and learning outcomes. Robotics systems have unique complexity (hardware abstraction, real-time constraints, distributed systems) that requires careful documentation.

### II. Testing & Validation Standards

All code examples MUST be validated before inclusion in the textbook. This is NON-NEGOTIABLE.

**Requirements:**
- Every code example MUST be run locally or in simulation before publication
- Every code example MUST include verification instructions (expected output, behavior, or state)
- ROS 2 nodes MUST include test commands (e.g., `ros2 topic echo`, `ros2 service call`)
- ROS 2 services and action servers MUST include working test examples or client code
- Simulation workflows (Gazebo/Webots) MUST document expected visual/behavioral outputs
- Simulation examples MUST include verification steps (robot position, sensor readings, task completion)
- LLM-generated code MUST be manually reviewed for:
  - Correct imports and dependencies
  - Runtime consistency (no race conditions, proper cleanup)
  - Hallucinated APIs or non-existent functions
  - Security issues (hardcoded credentials, unsafe deserialization)
- Integration tests MUST be provided for multi-node ROS 2 systems
- All test commands and validation procedures MUST be documented in the example

**Rationale:** Untested code in educational materials leads to student frustration and wasted time. Robotics code often fails in subtle ways (timing, network, hardware abstraction) that only manifest at runtime. LLMs can hallucinate plausible-looking but non-functional code.

### III. User Experience & Consistency

Content MUST follow a consistent structure and style to provide a professional, predictable learning experience.

**Requirements:**
- Every chapter MUST follow the schema: Title, Summary, Learning Outcomes, Sections
- Every module MUST use consistent heading hierarchy (H1 for chapters, H2 for major sections, H3 for subsections)
- Diagrams, tables, and code blocks MUST use consistent formatting conventions
- Terminology MUST remain uniform across all chapters (maintain a glossary if needed)
- Technical terms MUST be defined on first use in each chapter
- Examples MUST progress from simple to complex within each chapter
- Content MUST be accessible to beginners while offering depth for advanced learners
- Each chapter MUST include:
  - Clear learning objectives at the start
  - Practical examples demonstrating concepts
  - Summary of key takeaways at the end
- Cross-references between chapters MUST use consistent linking format
- Code formatting MUST be consistent: language tags in fenced blocks, syntax highlighting enabled

**Rationale:** Inconsistent structure and terminology increases cognitive load and makes the textbook harder to use as a reference. Students should be able to predict where to find information. Uniform progression from simple to complex supports scaffolded learning.

### IV. Performance & Accessibility

Code and simulation examples MUST run efficiently on mid-range student hardware.

**Requirements:**
- Code examples MUST run on mid-range laptops (8GB RAM, quad-core CPU, integrated graphics)
- Simulation examples MUST start with lightweight configurations before introducing full-physics models
- Initial Gazebo/Webots examples MUST use simple geometries and minimal sensors
- Full-physics simulations MUST include performance tuning guidance (reducing physics steps, simplifying meshes)
- ROS 2 communication patterns MUST avoid unnecessary overhead:
  - Use appropriate QoS profiles
  - Avoid excessive publication rates for tutorial code
  - Minimize large message types in basic examples
- AI/LLM agent examples MUST minimize API latency and cost:
  - Cache responses where appropriate
  - Use smaller models for simple tasks
  - Batch requests when possible
  - Document API cost implications
- Resource requirements MUST be documented for each example (RAM, CPU, GPU if needed)
- Fallback options MUST be provided for resource-intensive examples (cloud alternatives, simplified versions)

**Rationale:** Students should not need expensive hardware to learn. Robotics simulation can be resource-intensive; starting simple and scaling up teaches performance awareness. LLM API costs can surprise students; transparency and optimization are educational.

## Content Generation Rules

These rules govern how textbook content is created, validated, and maintained.

**Structure & Schema:**
- All content MUST follow the chapter schema defined in specification templates
- Chapters MUST NOT be published until complete (all sections, examples, diagrams)
- Explanations MUST be clear, structured, and suitable for beginners
- Theory and practice MUST be balanced: concepts explained, then demonstrated with ROS 2, Gazebo, Python, or LLM examples
- Code samples MUST be complete and runnable (no pseudocode unless explicitly labeled as such)
- Diagrams MUST be provided in text, ASCII art, or standard image formats (PNG, SVG)
- Diagrams MUST include alt-text descriptions for accessibility

**Writing Style:**
- Writing style MUST remain consistent across all modules (tone, voice, formality level)
- Active voice MUST be preferred over passive voice
- Second person ("you") MUST be used for instructions and tutorials
- Technical depth MUST match the target audience (undergraduate students, some programming background)
- Jargon MUST be minimized; unavoidable terms MUST be defined

**Module Lifecycle:**
- Each module MUST be independently specified, generated, reviewed, and deployed
- Modules MUST NOT be pushed to production until peer-reviewed
- Regeneration is allowed but MUST NOT break existing chapter structure or cross-references
- Changes to published modules MUST be tracked in version history

## Project Structure & Versioning

**MVP Structure:**
- The Minimum Viable Product (MVP) MUST contain:
  - One introductory chapter (overview, prerequisites, setup)
  - Four core modules (topics TBD in feature specifications)
- Additional modules MAY be added after MVP validation

**Versioning Policy:**
- Version format: MAJOR.MINOR.PATCH
- MAJOR increments after each complete module addition post-MVP
- MINOR increments for significant updates to existing modules (new sections, examples)
- PATCH increments for corrections, clarifications, formatting fixes
- Version history MUST be maintained in repository changelog
- Each module MUST document its version and last-updated date

**Trigger for Updates:**
- Major updates MUST be triggered using `/sp.specify` feature specifications
- Constitution changes MUST be rare and only for fundamental principle updates

## Safety Requirements

Robotics content carries inherent safety risks. These rules are NON-NEGOTIABLE.

**Prohibited Content:**
- No unsafe robot control instructions (e.g., disabling safety limits, uncontrolled motion)
- No instructions for physical robots without corresponding safe simulation examples
- No dangerous, untested, or hazardous robotics actions
- No code that could cause physical harm if executed incorrectly (e.g., high-speed motion without bounds checking)

**Mandatory Safety Practices:**
- All physical robot instructions MUST include safe simulation equivalents
- Motion commands MUST include velocity and acceleration limits
- Robot control examples MUST include emergency stop patterns
- Safety warnings MUST be clearly marked for any code that interfaces with physical hardware
- Simulation MUST be the default teaching environment; physical robots are optional extensions

**Rationale:** Student safety is paramount. Robotics mistakes can cause injury or equipment damage. Teaching safe practices from the start builds professional habits.

## Deployment & Publication

**GitHub & Version Control:**
- After completing each module, changes MUST be committed to git
- Commits MUST be pushed to GitHub after module completion and review
- GitHub Pages MUST serve as the published source of truth for the textbook
- All changes MUST be traceable through git history with meaningful commit messages

**Publication Requirements:**
- Only complete, reviewed modules MAY be published to GitHub Pages
- Published content MUST pass all validation checks (code tested, links verified, formatting correct)
- Breaking changes to URLs or chapter structure MUST include redirects or migration notes

## Governance

**Authority:**
This constitution supersedes all other practices and guidelines. When conflicts arise, constitution principles take precedence.

**Amendments:**
- Amendments require:
  - Documented justification for the change
  - Review of impact on existing content and workflows
  - Migration plan for affected modules
  - Approval via pull request review process
- Amendments MUST follow semantic versioning for the constitution itself
- All amendments MUST update the Sync Impact Report at the top of this file

**Compliance:**
- All content pull requests MUST verify compliance with this constitution
- Complexity or deviations MUST be justified in writing (use Complexity Tracking in plan.md)
- Automated checks SHOULD be used where possible (linting, link checking, code validation)
- Manual review MUST verify adherence to principles that cannot be automated (clarity, pedagogical quality, safety)

**Living Document:**
- This constitution is a living document and should evolve with the project
- Feedback from students and educators SHOULD inform amendments
- Regular reviews (per major module release) SHOULD assess whether principles remain appropriate

**Enforcement:**
- The `/sp.plan` command MUST perform a Constitution Check before Phase 0 research
- The Constitution Check MUST be re-run after Phase 1 design
- Any violations flagged in the Constitution Check MUST be either resolved or justified in the Complexity Tracking table

---

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
