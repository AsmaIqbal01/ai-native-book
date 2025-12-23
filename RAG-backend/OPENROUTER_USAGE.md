# OpenRouter Integration Guide

This guide explains how to use OpenRouter as your LLM provider for the RAG backend.

## What is OpenRouter?

OpenRouter is a unified API for accessing multiple LLM providers, offering free-tier models and a simple OpenAI-compatible interface.

## Configuration

### 1. Environment Variables (Already Configured)

Your `.env` file has been configured with OpenRouter:

```env
# Primary LLM Provider (OpenRouter)
LLM_PROVIDER=openrouter
LLM_API_KEY=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
LLM_BASE_URL=https://openrouter.ai/api/v1
LLM_MODEL=mistralai/devstral-2512:free
```

**Security Note**: Your API key is already saved in `.env`. Never commit this file to git!

### 2. Automatic Integration

The existing RAG agent (`app/agents/rag_agent.py`) automatically uses the configuration from `.env`. No code changes needed!

When you start the server, it will:
1. Load OpenRouter as the primary LLM provider
2. Fall back to OpenAI if OpenRouter fails
3. Fall back to Gemini if both fail

## Using the Router Module

The `app/services/router.py` module provides convenient access to OpenRouter API.

### Basic Usage (Synchronous)

```python
from app.services.router import create_completion

# Simple completion
response = create_completion(
    messages=[
        {"role": "user", "content": "What is the meaning of life?"}
    ],
    temperature=0.7,
    max_tokens=500
)

print(response.choices[0].message.content)
```

### Basic Usage (Asynchronous)

```python
from app.services.router import acreate_completion

# Async completion (recommended for FastAPI)
response = await acreate_completion(
    messages=[
        {"role": "user", "content": "Explain quantum computing"}
    ],
    temperature=0.0,  # Deterministic
    max_tokens=1000
)

print(response.choices[0].message.content)
```

### Advanced Usage with Custom Configuration

```python
from app.services.router import OpenRouterClient

# Create custom client instance
router = OpenRouterClient(
    api_key="your-custom-key",
    model="anthropic/claude-3.5-sonnet",
    site_url="https://yoursite.com",
    site_name="Your App Name"
)

# Synchronous
response = router.create_completion(
    messages=[{"role": "user", "content": "Hello!"}],
    temperature=0.5
)

# Asynchronous
response = await router.acreate_completion(
    messages=[{"role": "user", "content": "Hello!"}],
    temperature=0.5
)
```

### Using with Existing RAG Agent

The RAG agent automatically uses OpenRouter. No changes needed:

```python
from app.agents.rag_agent import get_agent

agent = get_agent()

# This will use OpenRouter automatically
result = await agent.run_query(
    user_question="What are humanoid robots?",
    context="[Retrieved context here...]",
    mode="normal_rag"
)

print(result["answer"])
print(f"Provider used: {result['provider_used']}")  # Should show "openrouter"
```

## Available Models

OpenRouter provides access to many models. Free tier models include:

- `mistralai/devstral-2512:free` (Currently configured)
- `meta-llama/llama-3.2-11b-vision-instruct:free`
- `google/gemini-flash-1.5:free`

To change models, update `LLM_MODEL` in `.env`:

```env
LLM_MODEL=meta-llama/llama-3.2-11b-vision-instruct:free
```

See [OpenRouter Models](https://openrouter.ai/models) for full list.

## Optional: Site Rankings

For better rankings on openrouter.ai, add these to your `.env`:

```env
OPENROUTER_SITE_URL=https://yoursite.com
OPENROUTER_SITE_NAME=AI Native Book RAG
```

The router will automatically include these as headers in requests.

## Testing the Integration

### 1. Start the Backend Server

```bash
cd RAG-backend
python -m uvicorn app.main:app --reload
```

### 2. Test via API

```bash
curl -X POST "http://localhost:8000/api/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is Physical AI?",
    "context": "Physical AI refers to AI systems that interact with the physical world.",
    "mode": "normal_rag"
  }'
```

### 3. Check Logs

You should see:

```
INFO: Initialized RAG agent with 3 LLM provider(s)
INFO: Attempting LLM call with provider: openrouter (1/3)
INFO: Successfully generated response using openrouter
```

## Troubleshooting

### Error: "Invalid API key"

- Verify your API key in `.env`
- Check if the key is active on [OpenRouter Dashboard](https://openrouter.ai/keys)

### Error: "Model not found"

- Verify the model name in `LLM_MODEL`
- Check [OpenRouter Models](https://openrouter.ai/models) for correct names

### Error: "Rate limit exceeded"

- Free tier models have rate limits
- The system will automatically fail over to fallback providers
- Consider upgrading or using multiple providers

### No Response from Chatbot

1. Check backend logs for errors
2. Verify `.env` file is in `RAG-backend/` directory
3. Restart the backend server
4. Check if CORS is properly configured

## Migration from Other Providers

To switch back to another provider, update `.env`:

```env
# Switch to X.AI Grok
LLM_PROVIDER=xai
LLM_API_KEY=your_xai_key
LLM_BASE_URL=https://api.x.ai/v1
LLM_MODEL=grok-beta

# Or switch to OpenAI
LLM_PROVIDER=openai
LLM_API_KEY=your_openai_key
LLM_BASE_URL=https://api.openai.com/v1
LLM_MODEL=gpt-4o-mini
```

Then restart the server.

## Best Practices

1. **Use Async**: Always use async methods (`acreate_completion`) in FastAPI endpoints
2. **Handle Errors**: The RAG agent has automatic failover, but add try-catch for custom implementations
3. **Set Temperature**: Use `temperature=0.0` for factual Q&A, `0.7` for creative tasks
4. **Monitor Usage**: Check OpenRouter dashboard for usage and costs
5. **Secure Keys**: Never commit `.env` to version control (already in `.gitignore`)

## Additional Resources

- [OpenRouter Documentation](https://openrouter.ai/docs)
- [OpenRouter API Reference](https://openrouter.ai/docs/api-reference)
- [Model Comparison](https://openrouter.ai/models)
- [Pricing](https://openrouter.ai/docs/pricing)
