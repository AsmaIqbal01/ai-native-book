"""Skills package for website management agents."""

from .content_skills import update_text_content, generate_cta_buttons
from .sidebar_skills import update_sidebar_links
from .layout_skills import update_html_section, update_styles, validate_html
from .outline_skills import (
    generate_book_outline,
    add_chapter,
    add_subchapter,
    format_outline_markdown,
    validate_outline_structure
)
from .chapter_writer_skills import (
    write_chapter_section,
    create_textual_diagram,
    add_chapter_example,
    add_exercise,
    add_case_study,
    create_chapter_structure,
    add_key_takeaways,
    add_further_reading,
    validate_chapter_content
)
from .subchapter_writer_skills import (
    create_subchapter_header,
    add_definition,
    add_concept_explanation,
    add_step_by_step_guide,
    add_technical_example,
    add_comparison_table,
    add_visual_diagram,
    add_best_practices,
    add_common_pitfalls,
    add_practical_application,
    add_mathematical_formula,
    add_key_points_summary,
    validate_subchapter_content
)
from .translator_skills import (
    translate_section,
    preserve_markdown_structure,
    validate_translation_completeness,
    identify_technical_terms,
    format_urdu_heading,
    create_glossary_entry
)

__all__ = [
    'update_text_content',
    'generate_cta_buttons',
    'update_sidebar_links',
    'update_html_section',
    'update_styles',
    'validate_html',
    'generate_book_outline',
    'add_chapter',
    'add_subchapter',
    'format_outline_markdown',
    'validate_outline_structure',
    'write_chapter_section',
    'create_textual_diagram',
    'add_chapter_example',
    'add_exercise',
    'add_case_study',
    'create_chapter_structure',
    'add_key_takeaways',
    'add_further_reading',
    'validate_chapter_content',
    'create_subchapter_header',
    'add_definition',
    'add_concept_explanation',
    'add_step_by_step_guide',
    'add_technical_example',
    'add_comparison_table',
    'add_visual_diagram',
    'add_best_practices',
    'add_common_pitfalls',
    'add_practical_application',
    'add_mathematical_formula',
    'add_key_points_summary',
    'validate_subchapter_content',
    'translate_section',
    'preserve_markdown_structure',
    'validate_translation_completeness',
    'identify_technical_terms',
    'format_urdu_heading',
    'create_glossary_entry',
]
