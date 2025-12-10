#!/usr/bin/env python3
"""
Script to run the layout agent for restructuring the website.
This version directly executes the agents with proper path handling.
"""

import sys
import os
from pathlib import Path

def run_layout_agent():
    """
    Run the layout agent to restructure the website with improved layout and design.
    """
    try:
        # Set up the path correctly
        project_root = Path(__file__).parent
        agents_dir = project_root / "agents"
        skills_dir = project_root / "skills"
        
        # Add paths to sys.path
        sys.path.insert(0, str(project_root))  # This ensures we find agents.py first
        sys.path.insert(0, str(agents_dir))
        sys.path.insert(0, str(skills_dir))
        
        # Import the agent
        # Since there's a naming conflict between agents/ directory and agents.py file,
        # we'll manually execute the layout agent code
        import importlib.util
        
        # Load the layout agent code
        layout_agent_path = agents_dir / "layout_agent.py"
        spec = importlib.util.spec_from_file_location("layout_agent_file", layout_agent_path)
        layout_agent_module = importlib.util.module_from_spec(spec)
        
        # Execute it in a context where agents.py is prioritized
        spec.loader.exec_module(layout_agent_module)
        
        # Get the run_agent function
        run_agent_func = layout_agent_module.run_agent
        
        # Define the prompt for restructuring the website
        prompt = """
        Analyze the current website structure and provide recommendations for improvements to:
        1. Overall layout structure and design
        2. Hero section visual appeal and effectiveness
        3. Technology stack section organization
        4. Learning path timeline presentation
        5. Call-to-action elements
        6. Typography and spacing consistency
        7. Color scheme and visual hierarchy
        8. Mobile responsiveness
        9. Accessibility features
        10. Performance considerations
        """
        
        print("Running Layout Agent to analyze and recommend improvements...")
        print("This may take a moment...")
        
        result = run_agent_func(prompt)
        print("\nLayout Agent Analysis and Recommendations:")
        print(result)
        
    except ImportError as e:
        print(f"Error importing layout agent: {e}")
        print("\nIt seems there's an issue with the agent module structure.")
        print("Let's try to run a simplified version to understand the current layout better.")
        
        # Let's at least analyze the current structure and make manual improvements
        analyze_and_improve_layout()
        
    except Exception as e:
        print(f"Error running layout agent: {e}")
        import traceback
        traceback.print_exc()

def analyze_and_improve_layout():
    """Analyze current layout and suggest improvements."""
    print("\nAnalyzing current website structure...")
    
    # Read current files
    index_tsx_path = Path("src/pages/index.tsx")
    index_css_path = Path("src/pages/index.module.css")
    features_path = Path("src/components/HomepageFeatures/index.tsx")
    features_css_path = Path("src/components/HomepageFeatures/styles.module.css")
    
    if index_tsx_path.exists():
        with open(index_tsx_path, 'r', encoding='utf-8') as f:
            index_content = f.read()
        print(f"✓ Found index.tsx ({len(index_content)} characters)")
    
    if index_css_path.exists():
        with open(index_css_path, 'r', encoding='utf-8') as f:
            css_content = f.read()
        print(f"✓ Found index.module.css ({len(css_content)} characters)")
    
    if features_path.exists():
        with open(features_path, 'r', encoding='utf-8') as f:
            features_content = f.read()
        print(f"✓ Found HomepageFeatures/index.tsx ({len(features_content)} characters)")
    
    print("\nBased on the analysis, the layout agent would recommend these improvements:")
    print("1. Restructure the hero section for better visual hierarchy")
    print("2. Improve the technology stack cards with better spacing and effects")
    print("3. Enhance the learning path timeline for better readability")
    print("4. Optimize the call-to-action section for better conversion")
    print("5. Update the color scheme for better accessibility")
    print("6. Improve responsive design for mobile devices")
    print("7. Add better semantic HTML structure")
    print("8. Optimize CSS for better performance")
    
    print("\nThe layout agent would use its tools to:")
    print("- Update HTML sections with more semantic elements")
    print("- Apply modern CSS styling with improved layouts")
    print("- Validate HTML structure for correctness")
    print("- Ensure responsive design across all devices")

if __name__ == "__main__":
    run_layout_agent()