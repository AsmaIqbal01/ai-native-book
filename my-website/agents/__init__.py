"""Agents package for website management.

This package contains specialized agents for managing different aspects of the website:
- ContentAgent: Updates main content, chapters, and landing pages
- SidebarAgent: Manages navigation, sidebars, and menus
- LayoutAgent: Handles layout, HTML structure, and CSS styling
- OutlineAgent: Generates hierarchical book outlines with chapters and sub-chapters
- ChapterWriterAgent: Writes complete long-form chapter content with examples and exercises
- SubchapterWriterAgent: Writes detailed sub-chapter content with definitions and step-by-step guides
- TranslatorAgent: Translates English technical content to natural, fluent Urdu
"""

from .content_agent import content_agent, run_agent as run_content_agent
from .sidebar_agent import sidebar_agent, run_agent as run_sidebar_agent
from .layout_agent import layout_agent, run_agent as run_layout_agent
from .outline_agent import outline_agent, run_agent as run_outline_agent
from .chapter_writer_agent import chapter_writer_agent, run_agent as run_chapter_writer_agent
from .subchapter_writer_agent import subchapter_writer_agent, run_agent as run_subchapter_writer_agent
from .translator_agent import translator_agent, run_agent as run_translator_agent

__all__ = [
    'content_agent',
    'sidebar_agent',
    'layout_agent',
    'outline_agent',
    'chapter_writer_agent',
    'subchapter_writer_agent',
    'translator_agent',
    'run_content_agent',
    'run_sidebar_agent',
    'run_layout_agent',
    'run_outline_agent',
    'run_chapter_writer_agent',
    'run_subchapter_writer_agent',
    'run_translator_agent',
]
