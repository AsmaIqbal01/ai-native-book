#!/usr/bin/env python3
"""
Script to validate the modified files without using the agents package.
"""

import re
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
    
    # Check for proper JSX syntax - common issues
    # Check for potential unbalanced tags (basic check)
    # Extract tag names from opening and closing tags
    all_tags = re.findall(r'<(/?)([a-zA-Z][a-zA-Z0-9:.\-]*)', content)
    stack = []
    for is_closing, tag_name in all_tags:
        if not is_closing and tag_name not in ['img', 'br', 'hr', 'input', 'meta', 'link', 'path', 'svg', 'area', 'base', 'br', 'col', 'embed', 'source', 'track', 'wbr']:  # self-closing tags
            # Opening tag, add to stack
            stack.append(tag_name)
        elif is_closing and stack and tag_name not in ['img', 'br', 'hr', 'input', 'meta', 'link', 'path', 'svg', 'area', 'base', 'br', 'col', 'embed', 'source', 'track', 'wbr']:
            # Closing tag, check if it matches the most recent opening tag
            if stack[-1] != tag_name:
                issues.append(f"Mismatched closing tag: </{tag_name}> does not match <{stack[-1]}>")
            else:
                stack.pop()
    
    # Check if there are unclosed tags
    if stack:
        issues.append(f"Unclosed tags: {', '.join(set(stack))}")
    
    return issues

def validate_html_structure(content: str):
    """Validate HTML-like structure."""
    # Basic HTML validation - check for balanced tags
    # Remove JSX expressions first
    clean_content = re.sub(r'{[^}]*}', '', content)
    
    # Find all tags
    all_tags = re.findall(r'<(/?)([a-zA-Z][a-zA-Z0-9:.\-]*)[^>]*>', clean_content)
    
    stack = []
    for is_closing, tag_name in all_tags:
        if not is_closing and tag_name not in ['img', 'br', 'hr', 'input', 'meta', 'link', 'path', 'svg', 'area', 'base', 'col', 'embed', 'source', 'track', 'wbr']:  # self-closing
            stack.append(tag_name)
        elif is_closing and stack and tag_name not in ['img', 'br', 'hr', 'input', 'meta', 'link', 'path', 'svg', 'area', 'base', 'col', 'embed', 'source', 'track', 'wbr']:
            if stack[-1] != tag_name:
                return False  # Mismatched tags
            else:
                stack.pop()
    
    # If stack is empty, all tags were properly closed
    return len(stack) == 0

def validate_css(content: str, filename: str):
    """Validate CSS structure."""
    issues = []

    # Check for properly closed braces
    open_braces = content.count('{')
    close_braces = content.count('}')
    if open_braces != close_braces:
        issues.append(f"Mismatched braces: {open_braces} opening, {close_braces} closing")

    # Find CSS rule blocks
    css_blocks = re.findall(r'([^{]+)\{([^}]*)\}', content)
    for selector, block in css_blocks:
        # Check if selector is valid
        if not selector.strip():
            issues.append("Empty CSS selector found")

        # Check property-value pairs
        properties = block.split(';')
        for prop_val in properties:
            prop_val = prop_val.strip()
            if prop_val and ':' not in prop_val:
                issues.append(f"Invalid property-value pair in selector '{selector.strip()}': {prop_val}")
            elif prop_val and ':' in prop_val:
                prop, val = prop_val.split(':', 1)
                if not prop.strip():
                    issues.append(f"Empty property name in selector '{selector.strip()}'")
                # Note: empty values are allowed in CSS

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
                    print(f"    [WARN] {issue}")

                # Basic HTML validation on JSX content
                is_html_valid = validate_html_structure(content)
                print(f"  HTML structure valid: {'YES' if is_html_valid else 'NO'}")

                if not is_html_valid:
                    jsx_issues.append("HTML structure validation failed")

                all_issues.extend([f"{file_path}: {issue}" for issue in jsx_issues])

            elif file_path.endswith('.css'):
                # Validate CSS structure
                print(f"  Checking CSS structure...")
                issues = validate_css(content, file_path)
                for issue in issues:
                    print(f"    [WARN] {issue}")
                all_issues.extend([f"{file_path}: {issue}" for issue in issues])

            print(f"  [OK] Validation complete for {file_path}")

        except Exception as e:
            print(f"  [ERROR] Error validating {file_path}: {str(e)}")
            all_issues.append(f"{file_path}: Error during validation - {str(e)}")

    # Summary
    print(f"\n{'='*60}")
    print("VALIDATION SUMMARY")
    print(f"{'='*60}")

    if all_issues:
        print(f"[ERROR] Found {len(all_issues)} issues:")
        for i, issue in enumerate(all_issues, 1):
            print(f"  {i}. {issue}")
    else:
        print("[OK] All files passed validation - no issues found!")

    print(f"{'='*60}")
    
    return len(all_issues) == 0


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)