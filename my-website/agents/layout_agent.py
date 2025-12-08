"""
LayoutAgent - Updates website layout, sections, grids, and HTML/CSS.
"""

from typing import Any
from agents import Agent, Runner

# Import skills
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))

from skills.layout_skills import update_html_section, update_styles, validate_html


# Agent configuration
layout_agent = Agent(
    name="LayoutAgent",
    instructions="""You are a LayoutAgent specialized in managing website layout, structure, and styling.

Your responsibilities:
- Update website layout and section structure
- Manage HTML sections and their organization
- Update CSS styles and classes
- Maintain responsive grid layouts
- Ensure proper semantic HTML structure
- Handle visual design updates

Guidelines:
- Always validate HTML before and after changes
- Maintain responsive design across all screen sizes
- Use semantic HTML5 elements appropriately
- Follow CSS best practices (avoid inline styles when possible)
- Ensure accessibility standards (WCAG 2.1)
- Keep layouts consistent with the overall design system
- Consider performance implications of layout changes

When updating layouts:
1. Review the current structure and styles
2. Plan changes to maintain visual hierarchy
3. Update HTML sections as needed
4. Apply CSS changes following design system
5. Validate HTML structure
6. Test responsiveness considerations
7. Provide detailed summary of changes

Common tasks:
- Restructuring page sections
- Updating grid layouts
- Modifying spacing and alignment
- Changing color schemes and themes
- Updating component styles
""",
    model="claude-sonnet-4-5-20250929",
    model_settings={
        "temperature": 0.5,
        "max_tokens": 4096,
    },
    tools=[
        update_html_section,
        update_styles,
        validate_html,
    ],
)


def run_agent(prompt: str) -> str:
    """
    Run the LayoutAgent with the given prompt.

    Args:
        prompt: The user's request for layout/styling updates

    Returns:
        The agent's final output as a string
    """
    result = Runner.run_sync(
        starting_agent=layout_agent,
        input=prompt,
    )
    return result.final_output
