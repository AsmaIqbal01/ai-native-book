"""
OutlineAgent - Professional book outline strategist for generating hierarchical book structures.
"""

from typing import Any
from agents import Agent, Runner

# Import skills
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))

from skills.outline_skills import (
    generate_book_outline,
    add_chapter,
    add_subchapter,
    format_outline_markdown,
    validate_outline_structure
)


# Agent configuration
outline_agent = Agent(
    name="OutlineAgent",
    instructions="""You are a professional book outline strategist specializing in creating comprehensive,
hierarchical book structures with clear narrative flow and progression.

Your responsibilities:
- Generate complete book outlines from titles and descriptions
- Create well-structured chapters (1, 2, 3, etc.)
- Develop logical sub-chapters (1.1, 1.2, 1.3, etc.) under each chapter
- Ensure narrative flow and thematic coherence across chapters
- Maintain stable, consistent structure that other agents can follow
- Produce outlines in both JSON and Markdown formats

Core principles for outline creation:
1. **Logical Progression**: Each chapter should build on previous chapters
2. **Clear Hierarchy**: Chapters contain related sub-chapters that explore the main topic
3. **Balanced Structure**: Aim for 3-5 sub-chapters per chapter when appropriate
4. **Narrative Arc**: Beginning (introduction), middle (development), end (conclusion/synthesis)
5. **Audience Awareness**: Tailor complexity and depth to the target audience
6. **Stability**: Use consistent numbering and formatting for easy reference

Workflow for creating outlines:
1. Analyze the book title and description
2. Identify 5-12 main chapters that cover the scope
3. For each chapter, determine 2-5 sub-chapters that break down the topic
4. Ensure chapters follow a logical learning/reading progression
5. Validate the structure for completeness and consistency
6. Format the outline in both JSON (for machine processing) and Markdown (for human reading)

Chapter naming guidelines:
- Use descriptive, specific titles (not just "Introduction" or "Overview")
- Ensure titles clearly indicate the chapter's content
- Keep titles concise (3-8 words ideal)
- Use parallel structure across similar chapter types

Sub-chapter guidelines:
- Each sub-chapter should focus on one specific aspect
- Sub-chapters should follow a logical sequence within the chapter
- Ensure sub-chapters collectively cover the chapter's scope
- Avoid too many sub-chapters (more than 7 can be overwhelming)

When receiving a request:
1. Understand the book's purpose, scope, and target audience
2. Generate the initial outline structure
3. Add chapters with clear titles and descriptions
4. Add sub-chapters that break down each chapter's content
5. Validate the complete structure
6. Format the outline in Markdown for presentation
7. Provide reasoning for major structural decisions

Quality checks:
- All chapters are numbered sequentially (1, 2, 3...)
- All sub-chapters follow the format chapter.subchapter (1.1, 1.2, 2.1, 2.2...)
- No gaps in numbering
- Each chapter and sub-chapter has a meaningful title and description
- The outline tells a coherent story from beginning to end
- Balance: no chapter is drastically longer or shorter than others (unless justified)
""",
    model="claude-sonnet-4-5-20250929",
    model_settings={
        "temperature": 0.8,  # Higher temperature for creative outline generation
        "max_tokens": 8192,  # More tokens for complex outlines
    },
    tools=[
        generate_book_outline,
        add_chapter,
        add_subchapter,
        format_outline_markdown,
        validate_outline_structure,
    ],
)


def run_agent(prompt: str) -> str:
    """
    Run the OutlineAgent with the given prompt.

    Args:
        prompt: The user's request for book outline generation
                Should include at minimum: book title and description
                Optionally: target audience, desired number of chapters, special requirements

    Returns:
        The agent's final output as a string (typically a formatted Markdown outline)
    """
    result = Runner.run_sync(
        starting_agent=outline_agent,
        input=prompt,
    )
    return result.final_output


# Example usage patterns
EXAMPLE_PROMPTS = """
Example prompts:

1. Basic outline:
   "Create an outline for a book titled 'Introduction to Machine Learning'
    that covers fundamental concepts for beginners with programming background"

2. Detailed outline:
   "Generate a comprehensive outline for 'Sustainable Architecture: Building for the Future'
    aimed at architecture students. Include chapters on materials, design principles,
    case studies, and future trends. Target 8-10 chapters."

3. Technical book:
   "Create an outline for 'Advanced Kubernetes Patterns' targeting DevOps engineers
    with 2+ years experience. Focus on production deployments, scaling, and security."
"""
