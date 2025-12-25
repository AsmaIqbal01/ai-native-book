"""
SDK-native agent definitions using openai.agents.Agent.
Replaces custom BaseAgent architecture with genuine OpenAI Agents SDK.

This module provides:
1. SDK-native Agent instances (not custom classes)
2. Handoff-based orchestration
3. Tool-based agent composition
4. Backward-compatible with existing services
"""

from agents import Agent, handoff
from agents.extensions.handoff_prompt import prompt_with_handoff_instructions

from app.agents.sdk_tools import (
    embed_and_retrieve,
    synthesize_answer,
    validate_query,
)


# ============================================================================
# Specialized Agents (for Handoffs)
# ============================================================================

greeting_agent = Agent(
    name="Greeting Agent",
    instructions=prompt_with_handoff_instructions(
        """You handle greetings politely and warmly.

Respond with:
'Hello! I'm here to help you with questions about the AI-Native Robotics textbook. What would you like to know?'

Keep it friendly and concise."""
    ),
    model="gpt-4o-mini",  # Use cheaper model for simple tasks
)

out_of_scope_agent = Agent(
    name="Out of Scope Agent",
    instructions=prompt_with_handoff_instructions(
        """You politely redirect out-of-scope questions.

Respond with:
'This question appears to be outside the scope of the AI-Native Robotics documentation. I can only help with topics covered in the textbook, such as:
- Robotics and ROS2
- Vision-Language-Action (VLA) models
- Physical AI systems
- AI-native development patterns

Please ask a question related to these topics.'

Be polite but firm about scope boundaries."""
    ),
    model="gpt-4o-mini",
)


# ============================================================================
# Main Orchestrator Agent (Replaces RAGChatAgent)
# ============================================================================

rag_orchestrator_agent = Agent(
    name="RAG Orchestrator",

    instructions=prompt_with_handoff_instructions(
        """You are the orchestrator for a RAG system serving the AI-Native Robotics textbook.

**Your Responsibilities:**

1. **Detect Greetings**: If the user's query is a simple greeting (hi, hello, hey, good morning), use the handoff_to_greeting tool.

2. **Detect Out-of-Scope**: If the query is about topics NOT in the textbook (weather, politics, sports, cooking, movies, news), use the handoff_to_out_of_scope tool.

3. **Process Textbook Questions**: For valid questions about the textbook:
   a. First, call embed_and_retrieve to get relevant context from the vector database
   b. Then, call synthesize_answer with the retrieved context to generate the answer
   c. Return the answer to the user

**Tools Available:**
- embed_and_retrieve(query, chapter, top_k): Retrieves relevant chunks from Qdrant
- synthesize_answer(context, query, mode): Generates context-bound answer
- validate_query(query): Validates and normalizes input (optional)

**Important Guidelines:**
- ALWAYS use embed_and_retrieve FIRST for textbook questions (don't skip retrieval)
- ALWAYS pass the retrieved context to synthesize_answer
- For selected text queries, you may need to use mode='selected_text_only' in synthesize_answer
- Keep responses focused and helpful
- If retrieval returns no chunks, acknowledge the limitation

**Examples:**

User: "Hi"
→ Use handoff_to_greeting

User: "What's the weather today?"
→ Use handoff_to_out_of_scope

User: "What is ROS2?"
→ 1. Call embed_and_retrieve(query="What is ROS2?", top_k=5)
→ 2. Call synthesize_answer(context=<retrieved_context>, query="What is ROS2?")
→ 3. Return the answer
"""
    ),

    model="gpt-4o",  # Primary model for orchestration

    # Tools for RAG pipeline
    tools=[
        embed_and_retrieve,
        synthesize_answer,
        validate_query,
    ],

    # Handoffs for specialized handling
    handoffs=[
        handoff(
            agent=greeting_agent,
            tool_name_override="handoff_to_greeting",
            tool_description_override="Use when user sends a simple greeting (hi, hello, hey, good morning)"
        ),
        handoff(
            agent=out_of_scope_agent,
            tool_name_override="handoff_to_out_of_scope",
            tool_description_override="Use when query is about weather, politics, sports, cooking, movies, or other non-textbook topics"
        ),
    ],
)


# ============================================================================
# Alternative: Compose Sub-Agents as Tools (Explicit Orchestration Pattern)
# ============================================================================

# This pattern is more explicit and gives finer control over the pipeline.
# Use this if you prefer explicit agent composition over handoffs.

retrieval_agent = Agent(
    name="Retrieval Agent",
    instructions="""You are responsible for retrieving relevant context from the vector database.

Use the embed_and_retrieve tool to search for relevant chunks based on the user's query.
Return the retrieved context in a structured format.""",
    tools=[embed_and_retrieve],
    model="gpt-4o-mini",
)

synthesis_agent = Agent(
    name="Synthesis Agent",
    instructions="""You are responsible for generating context-bound answers.

Use the synthesize_answer tool to generate an answer based on the provided context.
Ensure the answer is grounded in the context and does not hallucinate.""",
    tools=[synthesize_answer],
    model="gpt-4o",
)

# Orchestrator using agents as tools (alternative pattern)
rag_orchestrator_with_agent_tools = Agent(
    name="RAG Orchestrator (Agent Tools Pattern)",

    instructions="""You orchestrate the RAG pipeline by calling sub-agents.

**Pipeline:**
1. For textbook questions, call retrieve_context to get relevant information
2. Then call generate_answer with the retrieved context
3. Return the answer to the user

For greetings or out-of-scope queries, use the appropriate handoffs.
""",

    model="gpt-4o",

    tools=[
        retrieval_agent.as_tool(
            tool_name="retrieve_context",
            tool_description="Retrieve relevant context from the vector database for the user's query"
        ),
        synthesis_agent.as_tool(
            tool_name="generate_answer",
            tool_description="Generate a context-bound answer using the retrieved context"
        ),
    ],

    handoffs=[greeting_agent, out_of_scope_agent],
)


# ============================================================================
# Export the default orchestrator for use in FastAPI
# ============================================================================

# By default, use the handoff-based orchestrator
# To use the agent-tools pattern, change this to rag_orchestrator_with_agent_tools
default_orchestrator = rag_orchestrator_agent
