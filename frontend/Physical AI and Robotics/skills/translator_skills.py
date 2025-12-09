"""Translation skills for converting technical content to Urdu."""

from typing import Dict, List, Optional
from agents import function_tool
import re


@function_tool
def translate_section(
    english_text: str,
    section_type: str = "content",
    preserve_terms: Optional[List[str]] = None
) -> str:
    """
    Translate a section of text from English to Urdu.

    Args:
        english_text: The English text to translate
        section_type: Type of section (content, heading, list, quote, example)
        preserve_terms: List of English terms to keep untranslated

    Returns:
        Translated Urdu text with proper formatting
    """
    # This is a tool that will be used by the LLM to structure translation work
    # The actual translation is done by the LLM's language capabilities
    return f"[Translated: {section_type}]"


@function_tool
def preserve_markdown_structure(markdown_text: str) -> Dict[str, any]:
    """
    Parse markdown structure to identify elements that need special handling.

    Args:
        markdown_text: The markdown text to parse

    Returns:
        Dictionary with structure information
    """
    structure = {
        "headings": [],
        "code_blocks": [],
        "lists": [],
        "tables": [],
        "links": [],
        "emphasis": []
    }

    # Find headings
    headings = re.findall(r'^(#{1,6})\s+(.+)$', markdown_text, re.MULTILINE)
    structure["headings"] = [{"level": len(h[0]), "text": h[1]} for h in headings]

    # Find code blocks
    code_blocks = re.findall(r'```[\s\S]*?```', markdown_text)
    structure["code_blocks"] = code_blocks

    # Find lists
    lists = re.findall(r'^[\*\-\+]\s+(.+)$', markdown_text, re.MULTILINE)
    structure["lists"] = lists

    # Find links
    links = re.findall(r'\[([^\]]+)\]\(([^\)]+)\)', markdown_text)
    structure["links"] = [{"text": l[0], "url": l[1]} for l in links]

    return structure


@function_tool
def validate_translation_completeness(
    original_text: str,
    translated_text: str
) -> Dict[str, any]:
    """
    Validate that translation is complete and preserves structure.

    Args:
        original_text: Original English text
        translated_text: Translated Urdu text

    Returns:
        Validation results
    """
    validation = {
        "complete": True,
        "issues": [],
        "statistics": {}
    }

    # Check for markdown structure preservation
    orig_headings = len(re.findall(r'^#{1,6}\s+', original_text, re.MULTILINE))
    trans_headings = len(re.findall(r'^#{1,6}\s+', translated_text, re.MULTILINE))

    if orig_headings != trans_headings:
        validation["complete"] = False
        validation["issues"].append(
            f"Heading count mismatch: {orig_headings} original vs {trans_headings} translated"
        )

    # Check for code block preservation
    orig_code_blocks = len(re.findall(r'```', original_text))
    trans_code_blocks = len(re.findall(r'```', translated_text))

    if orig_code_blocks != trans_code_blocks:
        validation["complete"] = False
        validation["issues"].append(
            f"Code block marker mismatch: {orig_code_blocks} original vs {trans_code_blocks} translated"
        )

    # Check for list preservation
    orig_lists = len(re.findall(r'^[\*\-\+]\s+', original_text, re.MULTILINE))
    trans_lists = len(re.findall(r'^[\*\-\+]\s+', translated_text, re.MULTILINE))

    if orig_lists != trans_lists:
        validation["issues"].append(
            f"List item count may differ: {orig_lists} original vs {trans_lists} translated"
        )

    validation["statistics"] = {
        "original_length": len(original_text),
        "translated_length": len(translated_text),
        "headings": {"original": orig_headings, "translated": trans_headings},
        "code_blocks": {"original": orig_code_blocks // 2, "translated": trans_code_blocks // 2},
        "lists": {"original": orig_lists, "translated": trans_lists}
    }

    return validation


@function_tool
def identify_technical_terms(text: str) -> List[str]:
    """
    Identify technical terms that should be kept in English.

    Args:
        text: The text to analyze

    Returns:
        List of identified technical terms
    """
    # Common technical terms that are typically kept in English when writing Urdu
    common_tech_terms = [
        "AI", "API", "Algorithm", "Array", "Backend", "Binary", "Bit", "Boolean",
        "Buffer", "Bug", "Byte", "Cache", "Class", "Client", "Cloud", "Code",
        "Compiler", "Computer", "CPU", "Data", "Database", "Debug", "Device",
        "Digital", "Directory", "DNS", "Download", "Email", "Error", "File",
        "Firewall", "Firmware", "Framework", "Frontend", "Function", "GPU",
        "Hardware", "HTTP", "HTTPS", "Input", "Integer", "Interface", "Internet",
        "IP", "Kernel", "Laptop", "Library", "Linux", "Loop", "Machine Learning",
        "Memory", "Method", "Mobile", "Model", "Module", "Network", "Neural Network",
        "Node", "Object", "Operating System", "Output", "Package", "Parameter",
        "Password", "Path", "Plugin", "Pointer", "Port", "Process", "Processor",
        "Program", "Protocol", "Python", "Query", "RAM", "Repository", "REST",
        "Robot", "Robotics", "Router", "Runtime", "Script", "SDK", "Sensor",
        "Server", "Software", "SQL", "SSL", "String", "Syntax", "System",
        "TCP", "Terminal", "Thread", "Token", "TypeScript", "UDP", "UI", "URL",
        "USB", "User", "UX", "Variable", "Vector", "Version", "Virtual", "Web",
        "Website", "Wi-Fi", "Windows", "XML"
    ]

    found_terms = []
    text_upper = text.upper()

    for term in common_tech_terms:
        if term.upper() in text_upper:
            found_terms.append(term)

    return list(set(found_terms))


@function_tool
def format_urdu_heading(
    heading_level: int,
    heading_text: str
) -> str:
    """
    Format a heading in Urdu with proper markdown structure.

    Args:
        heading_level: Level of heading (1-6)
        heading_text: The Urdu heading text

    Returns:
        Formatted markdown heading
    """
    hashes = "#" * heading_level
    return f"{hashes} {heading_text}"


@function_tool
def create_glossary_entry(
    english_term: str,
    urdu_translation: str,
    definition: str
) -> str:
    """
    Create a glossary entry for technical terms.

    Args:
        english_term: The English technical term
        urdu_translation: The Urdu translation (if any)
        definition: Definition in Urdu

    Returns:
        Formatted glossary entry
    """
    if urdu_translation and urdu_translation != english_term:
        entry = f"**{english_term}** ({urdu_translation}): {definition}"
    else:
        entry = f"**{english_term}**: {definition}"

    return entry
