"""Book outline generation and management skills."""

from typing import Dict, List, Optional
from agents import function_tool
import json


@function_tool
def generate_book_outline(title: str, description: str, num_chapters: Optional[int] = None) -> str:
    """
    Generate a complete hierarchical book outline from a title and description.

    This is a professional book outline strategist that produces structured outlines
    with chapters and sub-chapters, ensuring clear narrative flow and progression.

    Args:
        title: The book title
        description: Description of the book's content, themes, and target audience
        num_chapters: Optional number of chapters to generate (if not specified, determined automatically)

    Returns:
        JSON string containing the structured outline with chapters and sub-chapters
    """
    # This is a tool that returns a structured format
    # The actual intelligent outline generation would be done by the LLM using this tool
    outline_structure = {
        "title": title,
        "description": description,
        "chapters": [],
        "metadata": {
            "total_chapters": 0,
            "total_subchapters": 0,
            "outline_version": "1.0"
        }
    }

    return json.dumps(outline_structure, indent=2)


@function_tool
def add_chapter(outline_json: str, chapter_number: int, chapter_title: str, chapter_description: str) -> str:
    """
    Add a new chapter to the book outline.

    Args:
        outline_json: The current outline in JSON format
        chapter_number: The chapter number (1, 2, 3, etc.)
        chapter_title: The title of the chapter
        chapter_description: Brief description of what the chapter covers

    Returns:
        Updated outline JSON with the new chapter added
    """
    try:
        outline = json.loads(outline_json)

        chapter = {
            "number": chapter_number,
            "title": chapter_title,
            "description": chapter_description,
            "subchapters": []
        }

        outline["chapters"].append(chapter)
        outline["metadata"]["total_chapters"] = len(outline["chapters"])

        return json.dumps(outline, indent=2)
    except Exception as e:
        return f"Error adding chapter: {str(e)}"


@function_tool
def add_subchapter(
    outline_json: str,
    chapter_number: int,
    subchapter_number: int,
    subchapter_title: str,
    subchapter_description: str
) -> str:
    """
    Add a sub-chapter to a specific chapter in the outline.

    Args:
        outline_json: The current outline in JSON format
        chapter_number: The parent chapter number (1, 2, 3, etc.)
        subchapter_number: The sub-chapter number within the chapter (1, 2, 3, etc.)
        subchapter_title: The title of the sub-chapter
        subchapter_description: Brief description of the sub-chapter content

    Returns:
        Updated outline JSON with the new sub-chapter added
    """
    try:
        outline = json.loads(outline_json)

        # Find the chapter
        chapter_index = chapter_number - 1
        if chapter_index >= len(outline["chapters"]):
            return f"Error: Chapter {chapter_number} does not exist"

        subchapter = {
            "number": f"{chapter_number}.{subchapter_number}",
            "title": subchapter_title,
            "description": subchapter_description
        }

        outline["chapters"][chapter_index]["subchapters"].append(subchapter)

        # Update total subchapter count
        total_subchapters = sum(len(ch["subchapters"]) for ch in outline["chapters"])
        outline["metadata"]["total_subchapters"] = total_subchapters

        return json.dumps(outline, indent=2)
    except Exception as e:
        return f"Error adding subchapter: {str(e)}"


@function_tool
def format_outline_markdown(outline_json: str) -> str:
    """
    Convert the JSON outline into a formatted Markdown document.

    Args:
        outline_json: The outline in JSON format

    Returns:
        Markdown-formatted outline as a string
    """
    try:
        outline = json.loads(outline_json)

        md_lines = []
        md_lines.append(f"# {outline['title']}\n")
        md_lines.append(f"_{outline['description']}_\n")
        md_lines.append("---\n")

        for chapter in outline["chapters"]:
            md_lines.append(f"\n## Chapter {chapter['number']}: {chapter['title']}\n")
            md_lines.append(f"{chapter['description']}\n")

            for subchapter in chapter["subchapters"]:
                md_lines.append(f"\n### {subchapter['number']} {subchapter['title']}\n")
                md_lines.append(f"{subchapter['description']}\n")

        md_lines.append("\n---\n")
        md_lines.append(f"**Total Chapters:** {outline['metadata']['total_chapters']}  \n")
        md_lines.append(f"**Total Sub-chapters:** {outline['metadata']['total_subchapters']}\n")

        return "".join(md_lines)
    except Exception as e:
        return f"Error formatting outline: {str(e)}"


@function_tool
def validate_outline_structure(outline_json: str) -> Dict[str, any]:
    """
    Validate the outline structure for completeness and consistency.

    Args:
        outline_json: The outline in JSON format

    Returns:
        Dictionary containing validation results and any issues found
    """
    try:
        outline = json.loads(outline_json)

        validation = {
            "valid": True,
            "issues": [],
            "warnings": [],
            "statistics": {}
        }

        # Check required fields
        if not outline.get("title"):
            validation["valid"] = False
            validation["issues"].append("Missing book title")

        if not outline.get("description"):
            validation["warnings"].append("Missing book description")

        # Check chapters
        chapters = outline.get("chapters", [])
        if len(chapters) == 0:
            validation["warnings"].append("No chapters defined")

        # Check chapter numbering
        for i, chapter in enumerate(chapters):
            expected_num = i + 1
            if chapter.get("number") != expected_num:
                validation["issues"].append(
                    f"Chapter numbering issue: expected {expected_num}, found {chapter.get('number')}"
                )
                validation["valid"] = False

        # Check subchapter numbering
        for chapter in chapters:
            subchapters = chapter.get("subchapters", [])
            chapter_num = chapter.get("number")

            for i, subchapter in enumerate(subchapters):
                expected_num = f"{chapter_num}.{i + 1}"
                if subchapter.get("number") != expected_num:
                    validation["issues"].append(
                        f"Subchapter numbering issue in Chapter {chapter_num}: " +
                        f"expected {expected_num}, found {subchapter.get('number')}"
                    )
                    validation["valid"] = False

        # Gather statistics
        validation["statistics"] = {
            "total_chapters": len(chapters),
            "total_subchapters": sum(len(ch.get("subchapters", [])) for ch in chapters),
            "chapters_with_subchapters": sum(1 for ch in chapters if ch.get("subchapters")),
            "average_subchapters_per_chapter": (
                sum(len(ch.get("subchapters", [])) for ch in chapters) / len(chapters)
                if chapters else 0
            )
        }

        return validation
    except json.JSONDecodeError:
        return {
            "valid": False,
            "issues": ["Invalid JSON format"],
            "warnings": [],
            "statistics": {}
        }
    except Exception as e:
        return {
            "valid": False,
            "issues": [f"Validation error: {str(e)}"],
            "warnings": [],
            "statistics": {}
        }
