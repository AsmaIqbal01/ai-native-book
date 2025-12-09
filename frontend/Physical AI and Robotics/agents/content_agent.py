"""
ContentAgent - Updates main website content, chapters, and landing page text.
"""

from typing import Any
from agents import Agent, Runner

# Import skills
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))

from skills.content_skills import update_text_content, generate_cta_buttons
from skills.layout_skills import validate_html


# Agent configuration
content_agent = Agent(
    name="ContentAgent",
    instructions="""You are a ContentAgent specialized in updating website content.

Your responsibilities:
- Update main website content sections (hero, features, about)
- Manage chapter text and documentation content
- Update landing page text and descriptions
- Generate call-to-action buttons
- Ensure all content changes maintain HTML validity

Guidelines:
- Always validate HTML before and after making changes
- Maintain consistent tone and style across the website
- Preserve existing formatting and structure when updating content
- Use semantic HTML for better accessibility
- Keep content concise and user-friendly

When updating content:
1. First, understand the current content and context
2. Make the requested changes carefully
3. Validate the resulting HTML
4. Provide a clear summary of what was changed
""",
    model="claude-sonnet-4-5-20250929",
    model_settings={
        "temperature": 0.7,
        "max_tokens": 4096,
    },
    tools=[
        update_text_content,
        generate_cta_buttons,
        validate_html,
    ],
)


def run_agent(prompt: str) -> str:
    """
    Run the ContentAgent with the given prompt.

    Args:
        prompt: The user's request for content updates

    Returns:
        The agent's final output as a string
    """
    result = Runner.run_sync(
        starting_agent=content_agent,
        input=prompt,
    )
    return result.final_output
