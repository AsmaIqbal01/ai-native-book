#!/usr/bin/env python3
"""
Docusaurus Sidebar Automation Script

This script automatically generates a sidebars.ts file based on folder structure
with numeric prefixes representing chapter/module order.

Folder Naming Convention:
- Chapters: 001-chaptername, 002-chaptername, etc.
- Sub-chapters/modules: 001-01-subchaptername, 001-02-subchaptername, etc.

Example directory structure:
docs/
├── 001-introduction/
│   ├── introduction-to-ai.mdx
│   ├── history-of-ai.mdx
│   └── 001-01-foundations/
│       ├── basic-concepts.mdx
│       └── fundamental-principles.mdx
├── 002-advanced-topics/
│   ├── neural-networks.mdx
│   └── 002-01-deep-learning/
│       └── convolutional-networks.mdx
└── resources/
    ├── setup-guide.mdx
    └── glossary.mdx

The script will automatically sort and organize the sidebar based on numeric prefixes,
ensuring the correct order regardless of filesystem order.
"""
import os
import re
from pathlib import Path


def extract_prefix_and_name(name):
    """
    Extract numeric prefix and name from folder/file names.
    Expected formats: 001-chaptername, 001-01-subchaptername
    Returns (major, minor, name) tuple or (None, None, None) if no match
    """
    # For folders like 001-chaptername or 001-01-subchaptername
    match = re.match(r'^(\d+)(?:-(\d+))?-(.+)$', name)
    if match:
        major = int(match.group(1))
        minor = int(match.group(2)) if match.group(2) else None
        actual_name = match.group(3)
        return major, minor, actual_name
    return None, None, None


def format_label(prefix, name, is_sub=False):
    """Format labels by capitalizing and converting hyphens to spaces."""
    capitalized = ' '.join(word.capitalize() for word in name.split('-'))
    if is_sub:
        return f'{prefix}. {capitalized}'
    else:
        return f'{prefix}. {capitalized}'


def scan_docs_directory_by_convention(docs_path):
    """
    Scan the docs directory and return structured data based on the naming convention.
    This function specifically looks for folders following the 001-chaptername format.
    """
    if not os.path.exists(docs_path):
        raise FileNotFoundError(f"Docs directory not found: {docs_path}")

    entries = os.listdir(docs_path)

    # Find directories that follow the naming convention
    chapter_dirs = []
    resources_dir = []
    other_entries = []

    for entry in entries:
        full_path = os.path.join(docs_path, entry)
        if os.path.isdir(full_path):
            # Check if this directory follows the naming convention
            major, minor, name = extract_prefix_and_name(entry)
            if major is not None:
                chapter_dirs.append((major, minor or 0, name, entry))
            elif 'resource' in entry.lower() or 'appendix' in entry.lower() or 'reference' in entry.lower():
                resources_dir.append(entry)
        else:
            # Check if this file follows the naming convention
            major, minor, name = extract_prefix_and_name(os.path.splitext(entry)[0])
            if major is not None and entry.endswith(('.mdx', '.md')):
                other_entries.append((major, minor or 0, name, entry))

    # Sort chapter directories by numeric prefix
    chapter_dirs.sort(key=lambda x: (x[0], x[1]))
    other_entries.sort(key=lambda x: (x[0], x[1]))

    # Generate structured sidebar configuration
    sidebar_items = []

    # Add other top-level entries that follow naming convention
    for major, minor, name, file in other_entries:
        prefix = f'{major}-{str(minor).zfill(2)}' if minor else str(major)
        formatted_label = format_label(prefix, name)
        # For top-level files, we still add them to the sidebar
        sidebar_items.append(os.path.splitext(file)[0])

    # Process each chapter directory that follows the convention
    for major, minor, name, dir_name in chapter_dirs:
        chapter_path = os.path.join(docs_path, dir_name)
        if os.path.isdir(chapter_path):
            # Read contents of chapter directory
            chapter_contents = os.listdir(chapter_path)

            # Separate sub-chapter directories from files
            sub_chapters = []
            chapter_files = []

            for content in chapter_contents:
                if content not in ['_category_.json', '_category_.yml']:
                    content_path = os.path.join(chapter_path, content)
                    if os.path.isdir(content_path):
                        # Check if subdirectory follows the naming convention
                        sub_major, sub_minor, sub_name = extract_prefix_and_name(content)
                        if sub_major is not None:
                            sub_chapters.append((sub_major, sub_minor or 0, sub_name, content))
                    elif content.endswith(('.md', '.mdx')):
                        # Check if file follows the naming convention
                        file_major, file_minor, file_name = extract_prefix_and_name(os.path.splitext(content)[0])
                        if file_major is not None:
                            chapter_files.append((file_major, file_minor or 0, file_name, content))

            # Sort sub-chapters by prefix
            sub_chapters.sort(key=lambda x: (x[0], x[1]))

            # Sort chapter files by prefix
            chapter_files.sort(key=lambda x: (x[0], x[1]))
            chapter_files = [f[3] for f in chapter_files]  # Get just the filename

            # Create the chapter prefix (format: "001. Chapter Name" or "001-01. Subchapter Name")
            chapter_prefix = f'{major}-{str(minor).zfill(2)}' if minor else str(major)
            chapter_label = format_label(chapter_prefix, name)

            chapter_items = []

            # Process sub-chapters first
            for sub_major, sub_minor, sub_name, sub_dir_name in sub_chapters:
                sub_path = os.path.join(chapter_path, sub_dir_name)
                sub_contents = [f for f in os.listdir(sub_path)
                               if f not in ['_category_.json', '_category_.yml'] and f.endswith(('.md', '.mdx'))]

                sub_prefix = f'{sub_major}-{str(sub_minor).zfill(2)}'
                sub_label = format_label(sub_prefix, sub_name)

                sub_items = [f'{dir_name}/{sub_dir_name}/{os.path.splitext(f)[0]}' for f in sub_contents]
                sub_items.sort()

                if sub_items:
                    chapter_items.append({
                        'type': 'category',
                        'label': sub_label,
                        'collapsed': True,
                        'items': sub_items
                    })

            # Add individual chapter files
            for file in chapter_files:
                chapter_items.append(f'{dir_name}/{os.path.splitext(file)[0]}')

            if chapter_items:
                sidebar_items.append({
                    'type': 'category',
                    'label': chapter_label,
                    'collapsed': False,
                    'items': chapter_items
                })

    # Add resources directory at the end (if it exists and follows the pattern)
    for res_dir in resources_dir:
        res_path = os.path.join(docs_path, res_dir)
        if os.path.isdir(res_path):
            res_contents = []
            for f in os.listdir(res_path):
                if f not in ['_category_.json', '_category_.yml'] and f.endswith(('.md', '.mdx')):
                    # Check if the resource file follows the naming convention
                    major, minor, name = extract_prefix_and_name(os.path.splitext(f)[0])
                    if major is not None:
                        res_contents.append(f)

            res_items = [f'{res_dir}/{os.path.splitext(f)[0]}' for f in res_contents]
            res_items.sort()

            if res_items:
                sidebar_items.append({
                    'type': 'category',
                    'label': 'Resources',
                    'collapsed': False,
                    'items': res_items
                })

    return sidebar_items


def format_sidebar_items(items, indent=4):
    """Format sidebar items as TypeScript code."""
    result = []
    for item in items:
        indent_str = ' ' * indent
        if isinstance(item, dict):
            result.append(indent_str + '{')
            result.append(indent_str + f"  type: '{item['type']}',")
            result.append(indent_str + f"  label: '{item['label']}',")
            result.append(indent_str + f"  collapsed: {str(item['collapsed']).lower()},")
            result.append(indent_str + '  items: [')
            result.extend(format_sidebar_items(item['items'], indent + 2))
            result.append(indent_str + '  ],')
            result.append(indent_str + '},')
        else:
            result.append(f"{indent_str}'{item}',")
    return result


def generate_sidebar_ts(sidebar_items):
    """Generate the complete sidebars.ts file content."""
    # Create the sidebar content
    sidebar_content = []
    sidebar_content.append('// Generated automatically by Qwen. Do not edit manually.')
    sidebar_content.append("import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';")
    sidebar_content.append('')
    sidebar_content.append('const sidebars: SidebarsConfig = {')
    sidebar_content.append('  tutorialSidebar: [')
    sidebar_content.extend(format_sidebar_items(sidebar_items, 4))
    sidebar_content.append('  ],')
    sidebar_content.append('};')
    sidebar_content.append('')
    sidebar_content.append('export default sidebars;')

    return '\n'.join(sidebar_content)


def main():
    # Define docs path - adjust this path based on your project structure
    docs_path = "C:\\Users\\asmaiqbal\\ai-native-book\\frontend\\Physical AI and Robotics\\docs"

    try:
        sidebar_items = scan_docs_directory_by_convention(docs_path)
        sidebar_ts_content = generate_sidebar_ts(sidebar_items)

        # Write to sidebars.ts file
        output_path = "C:\\Users\\asmaiqbal\\ai-native-book\\sidebars.ts"
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(sidebar_ts_content)

        print(f"Sidebar generated successfully at {output_path}")
        print(f"Found {len(sidebar_items)} top-level sidebar items")

        if len(sidebar_items) == 0:
            print("Note: No directories found that match the naming convention (e.g., 001-chaptername).")
            print("To use this automation, rename your directories to follow the pattern:")
            print("  - Chapters: 001-chaptername, 002-chaptername, etc.")
            print("  - Sub-chapters: 001-01-subchaptername, 001-02-subchaptername, etc.")
            print("")
            print("Example directory structure:")
            print("docs/")
            print("├── 001-introduction/")
            print("│   ├── introduction-to-ai.mdx")
            print("│   ├── history-of-ai.mdx")
            print("│   └── 001-01-foundations/")
            print("│       ├── basic-concepts.mdx")
            print("│       └── fundamental-principles.mdx")
            print("├── 002-advanced-topics/")
            print("│   ├── neural-networks.mdx")
            print("│   └── 002-01-deep-learning/")
            print("│       └── convolutional-networks.mdx")
            print("└── resources/")
            print("    ├── setup-guide.mdx")
            print("    └── glossary.mdx")

    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("Make sure the docs path exists and follows the expected naming convention.")
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    main()