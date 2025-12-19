# Chapter 5: High-Level Task Planning with LLMs

**Status**: Draft - Template
**Last Updated**: 2025-12-09

## Chapter Summary

<!-- TODO: Content from T040-T046 - LLM-based planning -->

This chapter covers using LLMs for safe high-level robot planning, JSON schema validation, prompt engineering, and real API + mock fallback implementation.

## Learning Objectives

- [ ] Use LLMs for high-level robot task planning
- [ ] Design JSON schemas for safe action plans
- [ ] Engineer prompts for consistent structured outputs
- [ ] Implement real API integration (OpenAI/Anthropic/Google)
- [ ] Create mock planner fallback for offline learning

---

## 5.1 LLM-Based Planning for Robotics

<!-- TODO: Content from T041 - LLM planning fundamentals -->

**Key Principle**: LLMs generate HIGH-LEVEL plans only (never low-level control)

---

## 5.2 JSON Schema for Safe Action Plans

<!-- TODO: Content from T042 - JSON schema definition -->

### Example Plan Schema

```json
{
  "type": "object",
  "required": ["task_type", "parameters", "safety_constraints", "preconditions"],
  "properties": {
    "task_type": {"type": "string", "enum": ["navigate", "approach", "align"]},
    "parameters": {"type": "object"},
    "safety_constraints": {"type": "object"},
    "preconditions": {"type": "array"}
  }
}
```

**Full Schema**: See [plan_schema.json](code-examples/plan_schema.json)

---

## 5.3 Prompt Engineering for Consistency

<!-- TODO: Content from T043 - Prompt engineering -->

### Template Example

```
You are a robot task planner. Generate a safe, high-level plan in JSON format.

Input: {observation}
Command: {user_command}

Output JSON with fields: task_type, parameters, safety_constraints, preconditions.
Valid task_types: ["navigate", "approach", "align"]
```

**Code Example**: See [llm_planner.py](code-examples/llm_planner.py)

---

## 5.4 Real API Integration

<!-- TODO: Content from T087 - Real LLM API -->

### API Setup

```python
# TODO: OpenAI/Anthropic/Google setup
```

---

## 5.5 Mock Planner Fallback

<!-- TODO: Content from T088 - Mock planner -->

```python
# TODO: Template-based mock responses
```

**Code Example**: See [mock_planner.py](code-examples/mock_planner.py)

---

## 5.6 Safety Validation

<!-- TODO: Content from T086 - Safety validator -->

**Code Example**: See [safety_validator.py](code-examples/safety_validator.py)

---

## Key Takeaways

- LLMs excel at high-level planning, not low-level control
- JSON schema ensures parseable, validated plans
- Prompt engineering critical for consistent outputs
- Mock fallback ensures accessibility without API costs

## Next Chapter

[Chapter 6: Action Execution and Behavioral Control →](/docs/chapter5/module4-vla/ch06-execution/)
