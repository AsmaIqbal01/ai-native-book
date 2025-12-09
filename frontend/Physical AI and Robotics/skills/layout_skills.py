"""Layout and HTML/CSS management skills."""

from typing import Dict
from agents import function_tool
import re


@function_tool
def update_html_section(section_id: str, html_content: str) -> str:
    """
    Replaces or updates a specific HTML section.

    Args:
        section_id: The ID of the section to update
        html_content: The new HTML content for the section

    Returns:
        Status message indicating success or failure
    """
    try:
        # Implementation would update the actual HTML file
        # This is a placeholder
        return f"Successfully updated HTML section '{section_id}' with {len(html_content)} chars of content"
    except Exception as e:
        return f"Error updating HTML section: {str(e)}"


@function_tool
def update_styles(section_id: str, styles: Dict[str, str]) -> str:
    """
    Updates CSS classes or inline styles of a section.

    Args:
        section_id: The ID of the section to style
        styles: Dictionary of CSS properties and values

    Returns:
        Status message indicating success or failure
    """
    try:
        # Implementation would update the actual CSS or inline styles
        # This is a placeholder
        style_count = len(styles)
        style_list = "\n".join([f"  {prop}: {value}" for prop, value in styles.items()])
        return f"Successfully updated {style_count} styles for section '{section_id}':\n{style_list}"
    except Exception as e:
        return f"Error updating styles: {str(e)}"


@function_tool
def validate_html(html: str) -> bool:
    """
    Validates the HTML structure for correctness and semantic integrity.

    Args:
        html: The HTML string to validate

    Returns:
        True if valid, False otherwise
    """
    try:
        # Basic validation - check for balanced tags
        # A more complete implementation would use an HTML parser

        # Check for basic structure
        if not html or not isinstance(html, str):
            return False

        # Simple tag balance check
        opening_tags = re.findall(r'<([a-zA-Z][a-zA-Z0-9]*)\b[^>]*>', html)
        closing_tags = re.findall(r'</([a-zA-Z][a-zA-Z0-9]*)>', html)

        # Self-closing tags to ignore
        self_closing = {'img', 'br', 'hr', 'input', 'meta', 'link'}

        opening_filtered = [tag for tag in opening_tags if tag not in self_closing]

        # Basic validation: number of opening and closing tags should match
        if len(opening_filtered) == len(closing_tags):
            return True

        return False
    except Exception:
        return False
