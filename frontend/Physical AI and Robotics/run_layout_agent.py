#!/usr/bin/env python3
"""
Script to run the layout agent for restructuring the website.
"""

import sys
import os
from pathlib import Path
import shutil

# Add the project root to Python path
project_root = Path(__file__).parent

# Temporarily rename the agents directory to avoid import conflicts
agents_dir = project_root / "agents"
temp_agents_dir = project_root / "agents_temp"

if agents_dir.exists():
    shutil.move(str(agents_dir), str(temp_agents_dir))

# Add project root to path to ensure agents.py is found first
sys.path.insert(0, str(project_root))

# Now import the base Agent and Runner classes from agents.py
from agents import Agent, Runner

# Add original agents directory back
if temp_agents_dir.exists():
    shutil.move(str(temp_agents_dir), str(agents_dir))

# Now import the layout agent using importlib to control the import process
import importlib.util
layout_agent_spec = importlib.util.spec_from_file_location(
    "layout_agent",
    str(project_root / "agents" / "layout_agent.py")
)
layout_agent_module = importlib.util.module_from_spec(layout_agent_spec)

# Execute the module with a modified sys.modules to ensure correct imports
sys.modules["agents"] = __import__("agents")  # Ensure the agents.py file is used
layout_agent_spec.loader.exec_module(layout_agent_module)

# Get the run_agent function
run_agent = layout_agent_module.run_agent

def run_layout_agent():
    """
    Run the layout agent to restructure the website with improved layout and design.
    """
    try:
        # Import the layout agent
        from agents.layout_agent import run_agent
        
        # Define the prompt for restructuring the website
        prompt = """
        Restructure the AI-Native Robotics website with the following improvements:

        1. Enhance the overall layout structure:
           - Create a more modern, clean design
           - Improve visual hierarchy and spacing
           - Ensure responsive design for all devices
           - Add a consistent color scheme that matches the technology theme

        2. Improve the Hero Section:
           - Make it more impactful with better visual elements
           - Enhance the call-to-action buttons for better conversion
           - Improve the statistics section for better visual appeal

        3. Enhance the Technology Stack section:
           - Make cards more interactive with better hover effects
           - Improve the layout for better visual organization
           - Add icons or visual elements that better represent each technology

        4. Improve the Learning Path / Timeline:
           - Make the timeline more visually appealing
           - Enhance the chapter cards with better organization
           - Improve the topic tags for better readability

        5. Overall Improvements:
           - Add consistent spacing and typography across all sections
           - Enhance the color scheme for better readability
           - Improve accessibility with proper contrast ratios
           - Add subtle animations and transitions for better user experience
           - Ensure all sections have consistent padding and margins

        Use semantic HTML5 elements and follow modern CSS best practices.
        Maintain the existing content but improve the structure and visual presentation.
        """
        
        print("Running Layout Agent to restructure the website...")
        print("This may take a moment...")
        
        result = run_agent(prompt)
        print("\nLayout Agent Result:")
        print(result)
        
    except ImportError as e:
        print(f"Error importing layout agent: {e}")
        print("\nIt seems the 'agents' module structure might be incomplete.")
        print("The layout agent requires 'Agent' and 'Runner' classes which appear to be missing.")
        print("\nTo properly run the layout agent, we need to identify:")
        print("1. Where the base 'Agent' and 'Runner' classes are defined")
        print("2. Whether all dependencies are properly set up")
        
    except Exception as e:
        print(f"Error running layout agent: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    run_layout_agent()