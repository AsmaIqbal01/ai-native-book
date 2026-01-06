"""
Unit tests for mode_detector.py

Tests the detection logic for normal_rag vs selected_text_only modes.
"""

import pytest
from app.utils.mode_detector import detect_mode


def test_detect_normal_rag_when_no_selected_text():
    """Should detect normal_rag when no selected text provided."""
    mode = detect_mode(selected_text=None)
    assert mode == "normal_rag"


def test_detect_normal_rag_when_empty_selected_text():
    """Should detect normal_rag when selected text is empty string."""
    mode = detect_mode(selected_text="")
    assert mode == "normal_rag"


def test_detect_normal_rag_when_whitespace_only():
    """Should detect normal_rag when selected text is only whitespace."""
    mode = detect_mode(selected_text="   \n\t  ")
    assert mode == "normal_rag"


def test_detect_selected_text_only_when_text_provided():
    """Should detect selected_text_only when valid text provided."""
    mode = detect_mode(selected_text="This is some selected text from the book.")
    assert mode == "selected_text_only"


def test_detect_selected_text_only_with_multiline():
    """Should detect selected_text_only with multiline text."""
    selected_text = """
    ROS 2 is a framework for robot software.
    It provides communication infrastructure.
    """
    mode = detect_mode(selected_text=selected_text)
    assert mode == "selected_text_only"


def test_detect_selected_text_only_with_code():
    """Should detect selected_text_only when code snippet provided."""
    code = """
    def example():
        return "hello"
    """
    mode = detect_mode(selected_text=code)
    assert mode == "selected_text_only"
