---
id: 001
title: Spec 1 Core RAG Backend Planning
stage: plan
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: refactor-deployment-parity-fix
branch: refactor-deployment-parity-fix
user: asmaiqbal
command: /sp.plan
labels: ["spec-1", "rag-foundation", "planning", "architecture"]
links:
  spec: specs/refactor-deployment-parity-fix/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/refactor-deployment-parity-fix/spec.md
 - specs/refactor-deployment-parity-fix/plan.md
 - specs/refactor-deployment-parity-fix/research.md
 - specs/refactor-deployment-parity-fix/data-model.md
 - specs/refactor-deployment-parity-fix/quickstart.md
 - specs/refactor-deployment-parity-fix/contracts/ingest.json
 - specs/refactor-deployment-parity-fix/contracts/query.json
 - specs/refactor-deployment-parity-fix/contracts/health.json
tests:
 - None (planning phase, no code implementation yet)
---

## Prompt

```text
— Spec 1 Planning Prompt

Role:
You are a principal AI systems architect and technical planner responsible for defining Spec 1 of an AI-Native system.
Your task is to produce a clear, implementation-ready plan that will guide engineering work.

Spec 1 Context

Spec 1 represents the foundational capability of the system.
All later specs (agents, RAG, frontend integration, token efficiency) depend on this spec being correct and stable.

This plan must:

Define what Spec 1 is

Define what Spec 1 is NOT

Establish non-negotiable success criteria

Prevent scope creep

Your Planning Responsibilities
1. Define the Purpose of Spec 1

Clearly explain:

What core problem Spec 1 solves

Why this spec must exist before others

What capability the system gains once Spec 1 is complete

2. Define System Boundaries

Explicitly state:

What functionality belongs in Spec 1

What functionality must be excluded and deferred to later specs

❌ Do NOT include future features
❌ Do NOT reference later specs as if they already exist

3. Architectural Expectations

Describe at a high level:

Expected components (backend, data, config, etc.)

Responsibilities of each component

Any required technology choices (if mandatory)

Do NOT design implementation details yet — this is planning, not coding.

4. Success Criteria (Non-Negotiable)

List clear, testable conditions that must be true to mark Spec 1 as complete.

Each criterion should be:

Objective

Verifiable

Binary (pass/fail)

5. Out-of-Scope Items

Explicitly list:

Features

Integrations

Optimizations
that are not allowed in Spec 1.

This section is critical to prevent overengineering.

6. Dependencies & Assumptions

Document:

Any assumptions made

Any external dependencies (if unavoidable)

What later specs will safely assume once Spec 1 is complete

Required Output Format
Spec 1 — Planning Document

1. Purpose
- ...

2. Scope
- Included:
  - ...
- Excluded:
  - ...

3. High-Level Architecture
- ...

4. Success Criteria
- [ ] ...
- [ ] ...
- [ ] ...

5. Explicit
```

## Response snapshot

**Spec 1 Definition Created**: Complete specification document defining the foundational RAG backend capability.

**Core Capability**: Accept textbook content → chunk → embed → store in Qdrant → semantic retrieval via HTTP API.

**Key Decisions**:
1. **Scope**: Intentionally minimal - ingestion and retrieval ONLY. No LLM integration, no agents, no frontend.
2. **Technology Stack**:
   - FastAPI (backend framework)
   - Qdrant (vector database)
   - OpenAI text-embedding-3-small (embeddings - chosen over Cohere for 5x lower cost)
   - Python 3.11+ with Pydantic for type safety

3. **Architecture**: Single backend API with clear service layers:
   - Chunking Service (750 chars, 100 overlap)
   - Embedding Service (OpenAI/Cohere fallback)
   - Retriever Service (Qdrant search)
   - Context Builder (format chunks with metadata)

**Planning Artifacts Generated**:
1. ✅ `spec.md` - Complete Spec 1 definition with success criteria
2. ✅ `plan.md` - Implementation plan with technical context and constitution check
3. ✅ `research.md` - Phase 0 research decisions (chunking, embedding provider, schema, error handling, testing)
4. ✅ `data-model.md` - Qdrant schema and Pydantic models
5. ✅ `contracts/` - OpenAPI schemas for all endpoints (ingest, query, health)
6. ✅ `quickstart.md` - Developer onboarding guide

**Constitution Check**: ✅ PASS - No violations. Spec 1 aligns with all principles.

**Critical Research Decisions**:
- **Chunking**: 750 characters with 100 char overlap (balanced precision + context)
- **Embedding Provider**: OpenAI text-embedding-3-small (5x cheaper than Cohere, 1536 dimensions)
- **Qdrant Schema**: 6 metadata fields (chapter, section, page, doc_id, chunk_index, chunk_text)
- **Error Handling**: Custom exceptions with exponential backoff for rate limits
- **Testing**: Dual approach - unit tests (mocked) + integration tests (real Qdrant)

## Outcome

- ✅ Impact: Foundation spec defined for entire RAG system. Spec 2 (Agents), Spec 3 (Advanced RAG), Spec 4 (Frontend), Spec 5 (Token Efficiency) all depend on this.
- 🧪 Tests: No tests yet (planning phase). Test strategy defined in research.md for implementation phase.
- 📁 Files: 8 files created (spec, plan, research, data-model, quickstart, 3 API contracts)
- 🔁 Next prompts:
  - Run `/sp.tasks` to generate tasks.md (implementation checklist)
  - Run `/sp.implement` to execute tasks in TDD workflow
  - Run `/sp.adr` if architectural decisions warrant ADR documentation
- 🧠 Reflection: Spec 1 is intentionally minimal to prevent scope creep. Clear boundaries prevent feature additions (no LLM, no agents, no frontend). OpenAI embedding choice based on cost analysis ($0.02/1M vs Cohere $0.10/1M). Constitution check passed with zero violations.

## Evaluation notes (flywheel)

- Failure modes observed: None (planning phase successful)
- Graders run and results (PASS/FAIL):
  - Constitution Check: PASS (zero violations)
  - Scope Validation: PASS (clear in/out boundaries)
  - Technical Feasibility: PASS (all dependencies external, no blockers)
- Prompt variant (if applicable): Original planning prompt (user-provided)
- Next experiment (smallest change to try):
  - Validate chunking strategy with sample textbook chapter (empirical test of 750/100 parameters)
  - Benchmark OpenAI vs Cohere embedding quality on retrieval precision
  - Test Qdrant HNSW parameters (m=16, ef_construct=100) for speed vs accuracy tradeoff

---

**Planning Phase Complete**. Ready for `/sp.tasks` to generate implementation checklist.
