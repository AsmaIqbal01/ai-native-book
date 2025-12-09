---
id: 003
title: Plan Module 2 Digital Twin
stage: plan
date: 2025-12-07
surface: cli
model: gemini-pro
feature: 003-digital-twin-module
branch: 003-digital-twin-module
user: asmaiqbal
command: /sp.plan
labels: [plan, module-2, digital-twin]
links:
  spec: specs/003-digital-twin-module/spec.md
  ticket:
  adr:
  pr:
files:
- path: specs/003-digital-twin-module/plan.md
  action: modified
- path: specs/003-digital-twin-module/research.md
  action: created
- path: specs/003-digital-twin-module/data-model.md
  action: created
tests:

---

## Prompt

```text
---
description: Execute the implementation planning workflow using the plan template to generate design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/setup-plan.ps1 -Json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'm Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType gemini`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage: 
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.plan
id: module2_plan
title: "Plan for Module 2: Digital Twin (Gazebo & Unity)"
type: plan

context:
  - Module 2 focuses on teaching Digital Twin simulations for humanoid robots.
  - Gazebo is the primary focus; Unity content will be a brief teaser only.
  - All content must be beginner-friendly, runnable, and safe.
  - Target platform: Docusaurus + GitHub Pages, code examples in ROS 2 Humble/Iron + Python.

phases:
  Phase 1: Setup & Preparation
    tasks:
      - T001: Create module directory (`docs/module2/`) and subfolders (`code/`, `assets/`, `diagrams/`)
      - T002: Prepare Markdown templates for sections and code snippets
      - T003: Initialize Git branch for Module 2
      - T004: Add placeholders for Gazebo worlds, robot models, and sensor diagrams
      - T005: Confirm environment requirements and hardware assumptions for students
    estimated_time: 10 minutes

  Phase 2: Gazebo Core Content
    tasks:
      - T006: Write "Gazebo Simulation Overview" section with ASCII diagrams
      - T007: Create step-by-step instructions to set up Gazebo environment and spawn humanoid robot
      - T008: Write "Simulating Sensors" section with LiDAR, Depth Camera, and IMU examples
      - T009: Provide Python/ROS 2 code snippets for subscribing to sensor topics and reading data
      - T010: Include best practices for lightweight simulation, topic efficiency, and safe experimentation
    estimated_time: 60-90 minutes

  Phase 3: Unity Teaser (Optional)
    tasks:
      - T011: Write a 1-2 page teaser introducing Unity high-fidelity rendering
      - T012: Explain conceptual scene setup, camera placement, and sensor visualization
      - T013: Include conceptual diagrams only (no full Unity tutorials)
    estimated_time: 20-30 minutes

  Phase 4: Review & Validation
    tasks:
      - T014: Validate all code snippets run without errors in ROS 2 Humble/Iron
      - T015: Check diagrams and examples for clarity and consistency
      - T016: Ensure writing style is consistent with Module 1
      - T017: Confirm adherence to constitution principles: code quality, UX, performance, and safety
    estimated_time: 20 minutes

  Phase 5: Deployment
    tasks:
      - T018: Commit all module content, diagrams, and code to GitHub
      - T019: Push to GitHub Pages for live preview
      - T020: Increment version (2.1) in repository
    estimated_time: 5-10 minutes

deliverables:
  - Full Module 2 markdown ready for Docusaurus
  - Runnable Python/ROS 2 code snippets
  - ASCII/text diagrams for Gazebo and sensors
  - Unity teaser section with conceptual diagrams
  - GitHub commit and deployed Pages version
```

---

## Response snapshot

- **Branch**: `003-digital-twin-module`
- **`IMPL_PLAN` path**: `C:\Users\asmaiqbal\ai-native-book\specs\003-digital-twin-module\plan.md`
- **Generated artifacts**:
    - `C:\Users\asmaiqbal\ai-native-book\specs\003-digital-twin-module\research.md`
    - `C:\Users\asmaiqbal\ai-native-book\specs\003-digital-twin-module\data-model.md`

## Outcome

- ✅ Impact: The implementation plan for Module 2 has been successfully generated, including the technical context, constitution check, project structure, and data model.
- 🧪 Tests: No automated tests were run during this planning phase.
- 📁 Files: `specs/003-digital-twin-module/plan.md` was modified, and `research.md` and `data-model.md` were created.
- 🔁 Next prompts: `/sp.tasks`
- 🧠 Reflection: The planning phase effectively translated the clarified feature specification into a structured implementation plan. The manual creation of the PHR due to script failure remains a process improvement opportunity.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.ps1` script failed to execute, requiring manual PHR creation. The root cause appears to be an issue with executing PowerShell scripts from the current environment.
- Graders run and results (PASS/FAIL): Not applicable.
- Prompt variant (if applicable): Not applicable.
- Next experiment (smallest change to try): Investigate the PowerShell script execution issue or provide a more robust fallback mechanism for PHR creation.

```