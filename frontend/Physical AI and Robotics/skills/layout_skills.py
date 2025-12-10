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
    import os
    from pathlib import Path

    try:
        # Determine the file to update based on section_id
        # For the main page, we'll update the index.tsx file
        if section_id in ["hero", "tech-stack", "learning-path", "cta", "features"]:
            file_path = Path(__file__).parent.parent / "src" / "pages" / "index.tsx"
        elif section_id.startswith("component-"):
            # For components, find the appropriate file
            component_name = section_id.replace("component-", "")
            file_path = Path(__file__).parent.parent / "src" / "components" / f"{component_name}" / "index.tsx"
        else:
            return f"Error: Unknown section_id '{section_id}'"

        # Read the current file content
        if not file_path.exists():
            return f"Error: File {file_path} does not exist"

        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Determine the section to replace based on section_id
        if section_id == "hero":
            # Replace the entire HomepageHeader function
            start_marker = "function HomepageHeader() {"
            end_marker = "}  // Close of HomepageHeader"
            if end_marker not in content:
                # Add the end marker if not present
                # Find the end of the function by looking for the closing brace after the start
                import re
                pattern = r"function HomepageHeader\(\)[^{]*\{(?:[^{}]|\{[^{}]*\})*\}"
                matches = re.findall(pattern, content)
                if len(matches) > 0:
                    old_content = matches[0]
                    new_content = f"function HomepageHeader() {{\n{html_content}\n}}"
                    updated_content = content.replace(old_content, new_content)
                else:
                    return f"Error: Could not find HomepageHeader function in {file_path}"
            else:
                start_idx = content.find(start_marker)
                end_idx = content.find(end_marker) + len(end_marker)
                if start_idx != -1 and end_idx != -1:
                    old_content = content[start_idx:end_idx]
                    updated_content = content.replace(old_content, f"{start_marker}\n{html_content}\n{end_marker}")
                else:
                    return f"Error: Could not find section {section_id} in {file_path}"
        elif section_id == "tech-stack":
            # Replace the TechnologyStack function
            start_marker = "function TechnologyStack() {"
            end_marker = "}  // Close of TechnologyStack"
            if end_marker not in content:
                # Find the actual function using regex
                import re
                pattern = r"function TechnologyStack\(\)[^{]*\{(?:[^{}]|\{[^{}]*\})*\}"
                matches = re.findall(pattern, content)
                if len(matches) > 0:
                    old_content = matches[0]
                    new_content = f"function TechnologyStack() {{\n{html_content}\n}}"
                    updated_content = content.replace(old_content, new_content)
                else:
                    return f"Error: Could not find TechnologyStack function in {file_path}"
            else:
                start_idx = content.find(start_marker)
                end_idx = content.find(end_marker) + len(end_marker)
                if start_idx != -1 and end_idx != -1:
                    old_content = content[start_idx:end_idx]
                    updated_content = content.replace(old_content, f"{start_marker}\n{html_content}\n{end_marker}")
                else:
                    return f"Error: Could not find section {section_id} in {file_path}"
        elif section_id == "learning-path":
            # Replace the LearningPath function
            start_marker = "function LearningPath() {"
            end_marker = "}  // Close of LearningPath"
            if end_marker not in content:
                # Find the actual function using regex
                import re
                pattern = r"function LearningPath\(\)[^{]*\{(?:[^{}]|\{[^{}]*\})*\}"
                matches = re.findall(pattern, content)
                if len(matches) > 0:
                    old_content = matches[0]
                    new_content = f"function LearningPath() {{\n{html_content}\n}}"
                    updated_content = content.replace(old_content, new_content)
                else:
                    return f"Error: Could not find LearningPath function in {file_path}"
            else:
                start_idx = content.find(start_marker)
                end_idx = content.find(end_marker) + len(end_marker)
                if start_idx != -1 and end_idx != -1:
                    old_content = content[start_idx:end_idx]
                    updated_content = content.replace(old_content, f"{start_marker}\n{html_content}\n{end_marker}")
                else:
                    return f"Error: Could not find section {section_id} in {file_path}"
        elif section_id == "cta":
            # Replace the CallToAction function
            start_marker = "function CallToAction() {"
            end_marker = "}  // Close of CallToAction"
            if end_marker not in content:
                # Find the actual function using regex
                import re
                pattern = r"function CallToAction\(\)[^{]*\{(?:[^{}]|\{[^{}]*\})*\}"
                matches = re.findall(pattern, content)
                if len(matches) > 0:
                    old_content = matches[0]
                    new_content = f"function CallToAction() {{\n{html_content}\n}}"
                    updated_content = content.replace(old_content, new_content)
                else:
                    return f"Error: Could not find CallToAction function in {file_path}"
            else:
                start_idx = content.find(start_marker)
                end_idx = content.find(end_marker) + len(end_marker)
                if start_idx != -1 and end_idx != -1:
                    old_content = content[start_idx:end_idx]
                    updated_content = content.replace(old_content, f"{start_marker}\n{html_content}\n{end_marker}")
                else:
                    return f"Error: Could not find section {section_id} in {file_path}"
        elif section_id == "features":
            # Replace the HomepageFeatures component
            file_path = Path(__file__).parent.parent / "src" / "components" / "HomepageFeatures" / "index.tsx"
            if not file_path.exists():
                return f"Error: File {file_path} does not exist"

            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            start_marker = "export default function HomepageFeatures()"
            # Find the function by regex
            import re
            pattern = r"export default function HomepageFeatures\(\)[^{]*\{(?:[^{}]|\{[^{}]*\})*\}"
            matches = re.findall(pattern, content)
            if len(matches) > 0:
                old_content = matches[0]
                new_content = f"export default function HomepageFeatures(): ReactNode {{\n{html_content}\n}}"
                updated_content = content.replace(old_content, new_content)
            else:
                return f"Error: Could not find HomepageFeatures function in {file_path}"
        else:
            return f"Error: Section {section_id} is not yet implemented"

        # Write the updated content back to the file
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(updated_content)

        return f"Successfully updated HTML section '{section_id}' in {file_path} with {len(html_content)} chars of content"

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
    import os
    from pathlib import Path

    try:
        # Determine the CSS file to update based on section_id
        if section_id in ["hero", "tech-stack", "learning-path", "cta"]:
            css_file = Path(__file__).parent.parent / "src" / "pages" / "index.module.css"
        elif section_id == "features":
            css_file = Path(__file__).parent.parent / "src" / "components" / "HomepageFeatures" / "styles.module.css"
        else:
            return f"Error: Unknown section_id '{section_id}' for CSS update"

        if not css_file.exists():
            return f"Error: CSS file {css_file} does not exist"

        # Read the current CSS content
        with open(css_file, 'r', encoding='utf-8') as f:
            css_content = f.read()

        # Create CSS rule based on section_id
        # Map section_id to corresponding CSS class names
        section_to_class = {
            "hero": [".heroBanner", ".heroContent", ".heroText", ".heroTitle", ".heroSubtitle", ".heroButtons", ".heroStats", ".heroImage", ".heroRobot"],
            "tech-stack": [".techStack", ".techGrid", ".techCard"],
            "learning-path": [".learningPath", ".timeline", ".timelineItem", ".timelineContent"],
            "cta": [".cta", ".ctaContent"],
            "features": [".features", ".featureSvg"]
        }

        if section_id not in section_to_class:
            return f"Error: Section {section_id} not mapped to any CSS classes"

        updated_content = css_content
        for css_class in section_to_class[section_id]:
            # Check if the CSS class already exists in the file
            import re
            pattern = rf"({re.escape(css_class)}\s*\{{(?:[^{{}}]|\{{[^{{}}]*\}})*\}})"
            matches = re.findall(pattern, css_content)

            if matches:
                # Update existing CSS rules
                for match in matches:
                    # Extract the existing properties
                    existing_props_pattern = r"([a-zA-Z\-]+\s*:\s*[^;]+;?)"
                    existing_props = re.findall(existing_props_pattern, match)

                    # Create a list of new properties to add/update
                    updated_props = []
                    # Add existing properties that are not being updated
                    for prop in existing_props:
                        prop_name = prop.split(':')[0].strip()
                        if prop_name not in styles:
                            updated_props.append(prop)

                    # Add new/updated properties from the styles dict
                    for prop_name, prop_value in styles.items():
                        updated_props.append(f"  {prop_name}: {prop_value};")

                    # Replace the old rule with the new one
                    new_rule = f"{css_class} {{\n" + "\n".join(updated_props) + "\n}"
                    updated_content = updated_content.replace(match, new_rule)
            else:
                # Add new CSS rule if class doesn't exist
                new_rule = f"\n/* Updated styles for {section_id} */\n{css_class} {{\n"
                for prop_name, prop_value in styles.items():
                    new_rule += f"  {prop_name}: {prop_value};\n"
                new_rule += "}\n"
                updated_content += new_rule

        # Write the updated CSS back to the file
        with open(css_file, 'w', encoding='utf-8') as f:
            f.write(updated_content)

        style_count = len(styles)
        style_list = "\n".join([f"  {prop}: {value}" for prop, value in styles.items()])
        return f"Successfully updated {style_count} styles for section '{section_id}' in {css_file}:\n{style_list}"

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
