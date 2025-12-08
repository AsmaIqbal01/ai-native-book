"""Sub-chapter writing skills for creating detailed, focused technical content."""

from typing import Dict, List, Optional
from agents import function_tool
import json


@function_tool
def create_subchapter_header(
    subchapter_number: str,
    subchapter_title: str,
    estimated_reading_time: Optional[int] = None
) -> str:
    """
    Create a professional header for a sub-chapter.

    Args:
        subchapter_number: The sub-chapter number (e.g., "1.1", "2.3")
        subchapter_title: The title of the sub-chapter
        estimated_reading_time: Optional reading time in minutes

    Returns:
        Formatted sub-chapter header as markdown
    """
    header = f"## {subchapter_number} {subchapter_title}\n\n"

    if estimated_reading_time:
        header += f"*Estimated reading time: {estimated_reading_time} minutes*\n\n"

    header += "---\n\n"

    return header


@function_tool
def add_definition(term: str, definition: str, context: Optional[str] = None) -> str:
    """
    Add a formal definition for a term or concept.

    Args:
        term: The term being defined
        definition: The formal definition
        context: Optional additional context or explanation

    Returns:
        Formatted definition block
    """
    definition_block = f"\n**{term}**\n\n"
    definition_block += f"> {definition}\n"

    if context:
        definition_block += f"\n{context}\n"

    return definition_block


@function_tool
def add_concept_explanation(
    concept_name: str,
    explanation: str,
    importance: Optional[str] = None
) -> str:
    """
    Add a detailed concept explanation.

    Args:
        concept_name: Name of the concept
        explanation: Detailed explanation of the concept
        importance: Optional explanation of why this concept matters

    Returns:
        Formatted concept explanation
    """
    concept = f"\n### {concept_name}\n\n"
    concept += f"{explanation}\n"

    if importance:
        concept += f"\n**Why This Matters:** {importance}\n"

    return concept


@function_tool
def add_step_by_step_guide(
    guide_title: str,
    steps: List[Dict[str, str]],
    prerequisites: Optional[List[str]] = None
) -> str:
    """
    Add a step-by-step guide or procedure.

    Args:
        guide_title: Title of the guide
        steps: List of steps, each with 'title' and 'description'
        prerequisites: Optional list of prerequisites

    Returns:
        Formatted step-by-step guide
    """
    guide = f"\n### {guide_title}\n\n"

    if prerequisites:
        guide += "**Prerequisites:**\n"
        for prereq in prerequisites:
            guide += f"- {prereq}\n"
        guide += "\n"

    for i, step in enumerate(steps, 1):
        guide += f"**Step {i}: {step.get('title', '')}**\n\n"
        guide += f"{step.get('description', '')}\n\n"

    return guide


@function_tool
def add_technical_example(
    example_title: str,
    scenario: str,
    implementation: str,
    explanation: str,
    code_snippet: Optional[str] = None
) -> str:
    """
    Add a detailed technical example with scenario, implementation, and explanation.

    Args:
        example_title: Title of the example
        scenario: The scenario or problem being addressed
        implementation: How it's implemented
        explanation: Detailed explanation of the solution
        code_snippet: Optional code example

    Returns:
        Formatted technical example
    """
    example = f"\n#### Example: {example_title}\n\n"
    example += f"**Scenario:** {scenario}\n\n"
    example += f"**Implementation:**\n{implementation}\n\n"

    if code_snippet:
        example += "```\n"
        example += code_snippet
        example += "\n```\n\n"

    example += f"**Explanation:** {explanation}\n"

    return example


@function_tool
def add_comparison_table(
    table_title: str,
    columns: List[str],
    rows: List[List[str]]
) -> str:
    """
    Create a comparison table in markdown format.

    Args:
        table_title: Title of the table
        columns: List of column headers
        rows: List of rows, each row is a list of values

    Returns:
        Formatted markdown table
    """
    table = f"\n**{table_title}**\n\n"

    # Header
    table += "| " + " | ".join(columns) + " |\n"
    table += "|" + "|".join(["---" for _ in columns]) + "|\n"

    # Rows
    for row in rows:
        table += "| " + " | ".join(row) + " |\n"

    table += "\n"

    return table


@function_tool
def add_visual_diagram(
    diagram_title: str,
    diagram_content: str,
    description: str
) -> str:
    """
    Add a visual diagram with description.

    Args:
        diagram_title: Title of the diagram
        diagram_content: ASCII art or text-based diagram
        description: Explanation of what the diagram shows

    Returns:
        Formatted diagram with description
    """
    diagram = f"\n**{diagram_title}**\n\n"
    diagram += "```\n"
    diagram += diagram_content
    diagram += "\n```\n\n"
    diagram += f"*{description}*\n"

    return diagram


@function_tool
def add_best_practices(
    section_title: str,
    practices: List[Dict[str, str]]
) -> str:
    """
    Add a best practices section.

    Args:
        section_title: Title of the best practices section
        practices: List of practices, each with 'title' and 'description'

    Returns:
        Formatted best practices section
    """
    section = f"\n### {section_title}\n\n"

    for practice in practices:
        title = practice.get('title', '')
        description = practice.get('description', '')
        section += f"**✓ {title}**\n{description}\n\n"

    return section


@function_tool
def add_common_pitfalls(
    pitfalls: List[Dict[str, str]]
) -> str:
    """
    Add a common pitfalls/mistakes section.

    Args:
        pitfalls: List of pitfalls, each with 'mistake' and 'solution'

    Returns:
        Formatted pitfalls section
    """
    section = "\n### Common Pitfalls to Avoid\n\n"

    for pitfall in pitfalls:
        mistake = pitfall.get('mistake', '')
        solution = pitfall.get('solution', '')
        section += f"**❌ Pitfall:** {mistake}\n"
        section += f"**✓ Solution:** {solution}\n\n"

    return section


@function_tool
def add_practical_application(
    application_title: str,
    context: str,
    approach: str,
    benefits: List[str]
) -> str:
    """
    Add a practical application section.

    Args:
        application_title: Title of the application
        context: Context where this is applied
        approach: How to apply the concept
        benefits: List of benefits from this application

    Returns:
        Formatted practical application section
    """
    section = f"\n### Practical Application: {application_title}\n\n"
    section += f"**Context:** {context}\n\n"
    section += f"**Approach:**\n{approach}\n\n"
    section += "**Benefits:**\n"
    for benefit in benefits:
        section += f"- {benefit}\n"
    section += "\n"

    return section


@function_tool
def add_mathematical_formula(
    formula_name: str,
    formula: str,
    variables: Dict[str, str],
    explanation: str
) -> str:
    """
    Add a mathematical formula with variable definitions and explanation.

    Args:
        formula_name: Name of the formula
        formula: The formula itself
        variables: Dictionary mapping variable names to their meanings
        explanation: Explanation of the formula

    Returns:
        Formatted formula section
    """
    section = f"\n**{formula_name}**\n\n"
    section += f"```\n{formula}\n```\n\n"
    section += "**Where:**\n"
    for var, meaning in variables.items():
        section += f"- `{var}` = {meaning}\n"
    section += f"\n{explanation}\n"

    return section


@function_tool
def add_key_points_summary(key_points: List[str]) -> str:
    """
    Add a summary of key points at the end of the sub-chapter.

    Args:
        key_points: List of key points to remember

    Returns:
        Formatted key points section
    """
    section = "\n### Key Points\n\n"

    for point in key_points:
        section += f"• {point}\n"

    return section


@function_tool
def validate_subchapter_content(subchapter_markdown: str) -> Dict[str, any]:
    """
    Validate sub-chapter content for completeness and quality.

    Args:
        subchapter_markdown: The complete sub-chapter content

    Returns:
        Validation results with quality metrics
    """
    validation = {
        "valid": True,
        "warnings": [],
        "quality_metrics": {},
        "recommendations": []
    }

    # Count words
    words = len(subchapter_markdown.split())

    # Count educational elements
    definitions = subchapter_markdown.count("**Definition")
    examples = subchapter_markdown.count("Example:")
    diagrams = subchapter_markdown.count("```")
    steps = subchapter_markdown.count("**Step")

    # Count sections
    h2_sections = subchapter_markdown.count("\n## ")
    h3_sections = subchapter_markdown.count("\n### ")

    validation["quality_metrics"] = {
        "word_count": words,
        "definitions": definitions,
        "examples": examples,
        "diagrams": diagrams // 2,  # Each diagram has opening and closing
        "step_by_step_guides": steps,
        "main_sections": h2_sections,
        "subsections": h3_sections,
        "has_key_points": "Key Points" in subchapter_markdown
    }

    # Quality checks
    if words < 500:
        validation["warnings"].append(f"Sub-chapter may be too short ({words} words)")
        validation["recommendations"].append("Add more detailed explanations or examples")

    if words > 3000:
        validation["warnings"].append(f"Sub-chapter may be too long ({words} words)")
        validation["recommendations"].append("Consider breaking into smaller sub-sections")

    if examples == 0:
        validation["recommendations"].append("Add at least one practical example")

    if "TODO" in subchapter_markdown or "TBD" in subchapter_markdown:
        validation["valid"] = False
        validation["warnings"].append("Contains placeholders (TODO/TBD)")

    if "..." in subchapter_markdown and subchapter_markdown.count("...") > 2:
        validation["warnings"].append("Multiple ellipses detected - ensure content is complete")

    if not validation["quality_metrics"]["has_key_points"]:
        validation["recommendations"].append("Consider adding a Key Points summary")

    return validation
