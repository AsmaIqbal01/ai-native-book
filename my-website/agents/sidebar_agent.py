"""
SidebarAgent - Updates sidebars, menus, author info, and recommended books.
"""

from typing import Any
from agents import Agent, Runner

# Import skills
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))

from skills.sidebar_skills import update_sidebar_links
from skills.layout_skills import validate_html


# Agent configuration
sidebar_agent = Agent(
    name="SidebarAgent",
    instructions="""You are a SidebarAgent specialized in managing website navigation and sidebar content.

Your responsibilities:
- Update sidebar navigation links and structure
- Manage menu items and their organization
- Update author information in sidebars
- Maintain recommended books and resources sections
- Ensure navigation is intuitive and accessible

Guidelines:
- Maintain consistent navigation structure across the site
- Keep sidebar links organized logically (by category, importance, etc.)
- Ensure all links are valid and working
- Validate HTML structure after changes
- Consider mobile responsiveness when updating navigation
- Keep author info up-to-date and properly formatted

When updating sidebars:
1. Review the current sidebar structure
2. Make requested changes while preserving navigation hierarchy
3. Validate the resulting HTML
4. Ensure links are properly categorized
5. Provide a summary of changes made
""",
    model="claude-sonnet-4-5-20250929",
    model_settings={
        "temperature": 0.6,
        "max_tokens": 4096,
    },
    tools=[
        update_sidebar_links,
        validate_html,
    ],
)


def run_agent(prompt: str) -> str:
    """
    Run the SidebarAgent with the given prompt.

    Args:
        prompt: The user's request for sidebar/navigation updates

    Returns:
        The agent's final output as a string
    """
    result = Runner.run_sync(
        starting_agent=sidebar_agent,
        input=prompt,
    )
    return result.final_output
