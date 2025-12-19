# Chapter 3: Language Understanding and Command Parsing

**Status**: Draft - Template
**Last Updated**: 2025-12-09

## Chapter Summary

<!-- TODO: Explain natural language understanding for robotics -->

This chapter covers natural language interpretation for VLA systems, including command parsing, safety filtering, and structured command representation.

## Learning Objectives

- [ ] Parse natural language commands for robot control
- [ ] Implement safety filters (blocklist/allowlist)
- [ ] Validate and sanitize language inputs
- [ ] Convert natural language to structured robot commands
- [ ] Understand failure modes in language understanding

---

## 3.1 Natural Language Interpretation for Robotics

<!-- TODO: Content from T029 - NL interpretation fundamentals -->

---

## 3.2 Command Parsing

<!-- TODO: Content from T030 - Command parsing techniques -->

**Code Example**: See [command_parser.py](code-examples/command_parser.py)

---

## 3.3 Safety Filtering and Validation

<!-- TODO: Content from T031 - Safety filtering -->

### Blocklist Approach

```python
# TODO: Example of unsafe command blocklist
UNSAFE_KEYWORDS = ['force', 'torque', 'joint', 'velocity', ...]
```

### Allowlist Approach

```python
# TODO: Example of safe command allowlist
SAFE_ACTIONS = ['navigate', 'approach', 'align', ...]
```

**Code Example**: See [safety_filter.py](code-examples/safety_filter.py)

---

## 3.4 Structured Command Representation

<!-- TODO: Content from T032 - JSON schema preview -->

### From Natural Language to JSON

```json
{
  "command": "move to the red cube",
  "action_type": "navigate_to_object",
  "parameters": {
    "target_object": "red cube",
    "approach_distance": 0.5
  },
  "safety_constraints": {
    "max_velocity": 0.5,
    "collision_avoidance": true
  }
}
```

---

## Key Takeaways

- Natural language commands must be parsed and validated for safety
- Multiple safety layers: blocklist, allowlist, schema validation
- Structured representation (JSON) enables downstream processing
- Always validate before passing commands to robot

## Review Questions

1. Why is safety filtering critical for language-controlled robots?
2. What's the difference between blocklist and allowlist approaches?
3. How does structured representation improve safety?

## Next Chapter

[Chapter 4: VLA System Architecture →](/docs/chapter5/module4-vla/ch04-architecture/)
