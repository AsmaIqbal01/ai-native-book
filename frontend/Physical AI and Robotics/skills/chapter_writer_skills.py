"""Chapter writing skills for creating complete long-form book content."""

from typing import Dict, List, Optional
from agents import function_tool
import json


@function_tool
def write_chapter_section(
    section_title: str,
    section_content: str,
    section_type: str = "content"
) -> str:
    """
    Write a section of a chapter with specific content.

    Args:
        section_title: The title/heading of this section
        section_content: The complete content for this section
        section_type: Type of section (content, example, exercise, case_study, summary)

    Returns:
        Formatted section as markdown string
    """
    section_markers = {
        "content": "",
        "example": "📖 **Example:** ",
        "exercise": "✏️ **Exercise:** ",
        "case_study": "🔍 **Case Study:** ",
        "summary": "📝 **Key Takeaways:** ",
        "diagram": "📊 **Diagram:** "
    }

    marker = section_markers.get(section_type, "")

    formatted_section = f"\n### {section_title}\n\n"
    if marker:
        formatted_section += f"{marker}\n\n"
    formatted_section += f"{section_content}\n"

    return formatted_section


@function_tool
def create_textual_diagram(
    diagram_title: str,
    diagram_content: str,
    diagram_type: str = "flowchart"
) -> str:
    """
    Create a textual diagram representation.

    Args:
        diagram_title: Title of the diagram
        diagram_content: The ASCII/text representation of the diagram
        diagram_type: Type of diagram (flowchart, hierarchy, comparison, process)

    Returns:
        Formatted diagram as markdown code block
    """
    diagram = f"\n**{diagram_title}**\n\n"
    diagram += f"```{diagram_type}\n"
    diagram += diagram_content
    diagram += "\n```\n"

    return diagram


@function_tool
def add_chapter_example(
    example_title: str,
    example_description: str,
    example_code: Optional[str] = None
) -> str:
    """
    Add a practical example to the chapter.

    Args:
        example_title: Title of the example
        example_description: Explanation of the example
        example_code: Optional code snippet or detailed example

    Returns:
        Formatted example as markdown
    """
    example = f"\n#### 📖 Example: {example_title}\n\n"
    example += f"{example_description}\n\n"

    if example_code:
        example += "```\n"
        example += example_code
        example += "\n```\n"

    return example


@function_tool
def add_exercise(
    exercise_number: int,
    exercise_prompt: str,
    difficulty: str = "medium",
    hints: Optional[List[str]] = None
) -> str:
    """
    Add an exercise for the reader to complete.

    Args:
        exercise_number: The exercise number
        exercise_prompt: The exercise description/prompt
        difficulty: Difficulty level (easy, medium, hard)
        hints: Optional list of hints

    Returns:
        Formatted exercise as markdown
    """
    difficulty_emoji = {
        "easy": "🟢",
        "medium": "🟡",
        "hard": "🔴"
    }

    exercise = f"\n#### ✏️ Exercise {exercise_number} {difficulty_emoji.get(difficulty, '🟡')}\n\n"
    exercise += f"{exercise_prompt}\n"

    if hints:
        exercise += f"\n**Hints:**\n"
        for i, hint in enumerate(hints, 1):
            exercise += f"{i}. {hint}\n"

    return exercise


@function_tool
def add_case_study(
    case_study_title: str,
    background: str,
    challenge: str,
    solution: str,
    outcome: str,
    lessons_learned: Optional[List[str]] = None
) -> str:
    """
    Add a real-world case study to the chapter.

    Args:
        case_study_title: Title of the case study
        background: Background context
        challenge: The problem or challenge faced
        solution: The solution implemented
        outcome: Results and outcomes
        lessons_learned: Key lessons from the case study

    Returns:
        Formatted case study as markdown
    """
    case_study = f"\n#### 🔍 Case Study: {case_study_title}\n\n"
    case_study += f"**Background**\n{background}\n\n"
    case_study += f"**Challenge**\n{challenge}\n\n"
    case_study += f"**Solution**\n{solution}\n\n"
    case_study += f"**Outcome**\n{outcome}\n\n"

    if lessons_learned:
        case_study += "**Key Lessons**\n"
        for lesson in lessons_learned:
            case_study += f"- {lesson}\n"

    return case_study


@function_tool
def create_chapter_structure(
    chapter_number: str,
    chapter_title: str,
    learning_objectives: List[str],
    introduction: str
) -> str:
    """
    Create the initial structure for a chapter with front matter.

    Args:
        chapter_number: The chapter number (e.g., "1", "2.1")
        chapter_title: The title of the chapter
        learning_objectives: List of learning objectives for this chapter
        introduction: Opening paragraph(s) for the chapter

    Returns:
        Chapter header and introduction as markdown
    """
    chapter = f"# Chapter {chapter_number}: {chapter_title}\n\n"

    chapter += "## Learning Objectives\n\n"
    chapter += "By the end of this chapter, you will be able to:\n\n"
    for objective in learning_objectives:
        chapter += f"- {objective}\n"

    chapter += f"\n---\n\n"
    chapter += f"## Introduction\n\n{introduction}\n\n"

    return chapter


@function_tool
def add_key_takeaways(takeaways: List[str]) -> str:
    """
    Add a key takeaways section at the end of a chapter.

    Args:
        takeaways: List of key points from the chapter

    Returns:
        Formatted key takeaways section
    """
    section = "\n## 📝 Key Takeaways\n\n"

    for i, takeaway in enumerate(takeaways, 1):
        section += f"{i}. {takeaway}\n"

    return section


@function_tool
def add_further_reading(resources: List[Dict[str, str]]) -> str:
    """
    Add a further reading/resources section.

    Args:
        resources: List of resources, each with 'title', 'author', and 'description'

    Returns:
        Formatted resources section
    """
    section = "\n## 📚 Further Reading\n\n"

    for resource in resources:
        title = resource.get("title", "")
        author = resource.get("author", "")
        description = resource.get("description", "")

        section += f"- **{title}**"
        if author:
            section += f" by {author}"
        section += f"\n  {description}\n\n"

    return section


@function_tool
def validate_chapter_content(chapter_markdown: str) -> Dict[str, any]:
    """
    Validate chapter content for completeness and quality.

    Args:
        chapter_markdown: The complete chapter content in markdown format

    Returns:
        Validation results with statistics and recommendations
    """
    validation = {
        "valid": True,
        "warnings": [],
        "statistics": {},
        "recommendations": []
    }

    # Count sections
    sections = chapter_markdown.count("###")
    subsections = chapter_markdown.count("####")

    # Count educational elements
    examples = chapter_markdown.count("📖 Example:")
    exercises = chapter_markdown.count("✏️ Exercise")
    case_studies = chapter_markdown.count("🔍 Case Study:")
    diagrams = chapter_markdown.count("```")

    # Count words (rough estimate)
    words = len(chapter_markdown.split())

    validation["statistics"] = {
        "word_count": words,
        "sections": sections,
        "subsections": subsections,
        "examples": examples,
        "exercises": exercises,
        "case_studies": case_studies,
        "diagrams": diagrams // 2  # Each diagram has opening and closing ```
    }

    # Check for completeness
    if "# Chapter" not in chapter_markdown:
        validation["valid"] = False
        validation["warnings"].append("Missing chapter title")

    if "Learning Objectives" not in chapter_markdown:
        validation["warnings"].append("Missing learning objectives section")

    if "Introduction" not in chapter_markdown:
        validation["warnings"].append("Missing introduction section")

    if words < 2000:
        validation["warnings"].append(f"Chapter may be too short ({words} words)")
        validation["recommendations"].append("Consider adding more detailed explanations")

    if examples == 0:
        validation["recommendations"].append("Consider adding practical examples")

    if exercises == 0:
        validation["recommendations"].append("Consider adding exercises for practice")

    if sections < 3:
        validation["recommendations"].append("Consider breaking content into more sections")

    # Check for summary or key takeaways
    if "Key Takeaways" not in chapter_markdown and "Summary" not in chapter_markdown:
        validation["recommendations"].append("Consider adding a summary or key takeaways section")

    return validation
