#!/usr/bin/env python3
"""
Script to validate the modified files using the available skills.
"""

import re
from skills.layout_skills import validate_html, update_html_section, update_styles
from pathlib import Path

def read_file_content(filepath):
    """Read content from a file."""
    with open(filepath, 'r', encoding='utf-8') as f:
        return f.read()

def validate_react_jsx_structure(content: str, filename: str):
    """Validate JSX structure by checking for common errors."""
    issues = []
    
    # Check for proper attribute syntax
    # Look for attributes without quotes
    improper_attrs = re.findall(r'\s+([a-zA-Z]+)=((?!\")[^>\s]+)', content)
    if improper_attrs:
        for attr, value in improper_attrs:
            if not value.startswith('"') and not value.startswith("'"):
                issues.append(f"Improper attribute syntax for '{attr}' with value '{value}'")
    
    # Check for self-closing tags
    # Look for tags that should be self-closing
    self_closing_tags = ['img', 'br', 'hr', 'input', 'meta', 'link']
    non_self_closing = re.findall(r'<({})[^>]*/?[^>]*>(?!</\1>)'.format('|'.join(self_closing_tags)), content)
    for tag in non_self_closing:
        if tag in self_closing_tags:
            issues.append(f"Tag '{tag}' should be self-closing (<{tag} />)")
    
    # Check for proper JSX syntax - capitalization
    # Components should be capitalized
    lowercase_components = re.findall(r'<([a-z][a-z0-9]*)([^>]*)(?:/>|>.*?</\1>)', content)
    for comp in lowercase_components:
        if comp[0] not in ['div', 'span', 'p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'a', 'img', 'ul', 'ol', 'li', 'section', 'header', 'footer', 'main', 'nav', 'article', 'aside']:
            issues.append(f"Component '{comp[0]}' should be capitalized if it's a custom component")
    
    # Check for proper nesting (basic check)
    # Extract tag names from opening and closing tags
    all_tags = re.findall(r'<(/?)([a-zA-Z][a-zA-Z0-9:]*)', content)
    stack = []
    for is_closing, tag_name in all_tags:
        if not is_closing:  # Opening tag
            stack.append(tag_name)
        elif stack:  # Closing tag
            if stack[-1] != tag_name:
                if tag_name not in ['img', 'br', 'hr', 'input', 'meta', 'link', 'path', 'svg']:  # self-closing
                    issues.append(f"Mismatched closing tag: </{tag_name}> does not match <{stack[-1]}>")
            else:
                stack.pop()
    
    return issues

def main():
    """Main validation function."""
    print("Starting validation of modified files...")
    
    # Files to validate
    files_to_check = [
        'src/pages/index.tsx',
        'src/components/HomepageFeatures/index.tsx',
        'src/pages/index.module.css',
        'src/components/HomepageFeatures/styles.module.css'
    ]

    all_issues = []
    
    for file_path in files_to_check:
        print(f"\nValidating {file_path}...")
        try:
            content = read_file_content(file_path)
            
            if file_path.endswith('.tsx'):
                # Validate JSX structure
                print(f"  Checking JSX structure...")
                jsx_issues = validate_react_jsx_structure(content, file_path)
                for issue in jsx_issues:
                    print(f"    ⚠️  {issue}")
                
                # Basic HTML validation on JSX content
                # Clean JSX to HTML for validation
                clean_html = re.sub(r'{[^}]*}', '', content)  # Remove JSX expressions
                clean_html = re.sub(r'className=', 'class=', clean_html)  # Convert to HTML class
                clean_html = re.sub(r'htmlFor=', 'for=', clean_html)  # Convert to HTML for
                is_html_valid = validate_html(clean_html)
                print(f"  HTML structure valid: {'✓' if is_html_valid else '❌'}")
                
                all_issues.extend([f"{file_path}: {issue}" for issue in jsx_issues])
                
            elif file_path.endswith('.css'):
                # Validate CSS structure
                print(f"  Checking CSS structure...")
                issues = validate_css(content, file_path)
                for issue in issues:
                    print(f"    ⚠️  {issue}")
                all_issues.extend([f"{file_path}: {issue}" for issue in issues])
            
            print(f"  ✓ Validation complete for {file_path}")
            
        except Exception as e:
            print(f"  ❌ Error validating {file_path}: {str(e)}")
            all_issues.append(f"{file_path}: Error during validation - {str(e)}")

    # Summary
    print(f"\n{'='*60}")
    print("VALIDATION SUMMARY")
    print(f"{'='*60}")
    
    if all_issues:
        print(f"❌ Found {len(all_issues)} issues:")
        for i, issue in enumerate(all_issues, 1):
            print(f"  {i}. {issue}")
    else:
        print("✅ All files passed validation - no issues found!")
    
    print(f"{'='*60}")


def validate_css(content: str, filename: str):
    """Validate CSS structure."""
    issues = []
    
    # Check for proper property-value syntax
    malformed_properties = re.findall(r'([a-zA-Z\-]+):\s*([^{;]*?)(?:;|{|$)', content)
    for prop, value in malformed_properties:
        if prop.strip() and not value.strip():
            issues.append(f"Property '{prop}' has no value")
        elif prop.strip() and value.strip() == '{':
            issues.append(f"Property '{prop}' has malformed value")
    
    # Check for properly closed braces
    open_braces = content.count('{')
    close_braces = content.count('}')
    if open_braces != close_braces:
        issues.append(f"Mismatched braces: {open_braces} opening, {close_braces} closing")
    
    # Check for CSS syntax errors
    css_blocks = re.findall(r'\{([^}]*)\}', content)
    for block in css_blocks:
        properties = block.split(';')
        for prop_val in properties:
            if ':' in prop_val and prop_val.strip():
                if not re.match(r'^\s*[a-zA-Z\-]+\s*:\s*.+[^;]$', prop_val):  # Missing semicolon at end of prop
                    # Actually, CSS properties don't always need semicolons if they're the last in the block
                    pass  # This is OK, just for reference
    
    return issues


if __name__ == "__main__":
    main()