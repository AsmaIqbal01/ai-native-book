# LLM API Comparison Matrix

**Date**: 2025-12-07
**Module**: Module 4 - Vision-Language-Action Systems
**Purpose**: Compare LLM API options for VLA planning component

## Research Task: T004 - Investigate LLM API Options

---

## Comparison Matrix

| Feature | OpenAI GPT-4 | Anthropic Claude 3 | Google Gemini | Open Source (LLaMA/Mistral) |
|---------|--------------|-------------------|---------------|---------------------------|
| **Vision Capabilities** | ✅ GPT-4 Vision | ✅ Claude 3 Opus/Sonnet | ✅ Gemini Pro Vision | ⚠️ Limited (LLaVA) |
| **Structured Output** | ✅ JSON mode | ✅ Strong JSON adherence | ✅ JSON mode | ⚠️ Variable quality |
| **API Cost (per 1M tokens)** | $10-30 (input/output) | $15-75 (input/output) | $7-21 (input/output) | 💰 Free (self-hosted) |
| **Latency (p95)** | 2-5s | 3-6s | 2-4s | Variable (depends on hardware) |
| **Rate Limits (Free Tier)** | Limited | Limited | Limited | N/A |
| **Context Window** | 128K tokens | 200K tokens | 1M tokens | 4K-32K tokens |
| **Safety Features** | ✅ Built-in | ✅ Constitutional AI | ✅ Built-in | ❌ Manual filtering needed |
| **Ease of Setup** | ⭐⭐⭐⭐⭐ Easy API | ⭐⭐⭐⭐⭐ Easy API | ⭐⭐⭐⭐⭐ Easy API | ⭐⭐ Complex (model download, GPU) |
| **Student Accessibility** | 💳 Requires payment | 💳 Requires payment | 💳 Requires payment | ✅ Free but needs GPU |

---

## Detailed Analysis

### OpenAI GPT-4 Vision

**Pros**:
- Excellent vision understanding capabilities
- Strong JSON mode for structured outputs
- Well-documented API
- Large developer community
- Good balance of cost and performance

**Cons**:
- Requires API key and payment
- Rate limits on free tier
- No offline mode

**Cost Estimate for Module 4**:
- Per student exercise: ~$0.01-0.05 (assuming 1K-5K tokens per request)
- Full mini-project: ~$0.50-1.00
- **Recommended**: Use caching and response optimization

**Integration Complexity**: ⭐⭐⭐⭐⭐ (Very Easy)

**Recommended for Module 4**: ✅ **Primary option** for students with API access

---

### Anthropic Claude 3 (Opus/Sonnet)

**Pros**:
- Excellent reasoning capabilities
- Very strong at following JSON schema instructions
- Constitutional AI for safety
- Large context window (200K tokens)
- Good vision capabilities (Claude 3 Opus/Sonnet)

**Cons**:
- Higher cost than GPT-4
- Requires API key and payment
- Slightly higher latency

**Cost Estimate for Module 4**:
- Per student exercise: ~$0.02-0.08 (depending on Opus vs Sonnet)
- Full mini-project: ~$1.00-2.00
- **Note**: Sonnet is more cost-effective than Opus

**Integration Complexity**: ⭐⭐⭐⭐⭐ (Very Easy)

**Recommended for Module 4**: ✅ **Alternative option** for students preferring Claude

---

### Google Gemini Pro Vision

**Pros**:
- Competitive pricing
- Good vision capabilities
- Massive context window (1M tokens)
- JSON mode support
- Free tier available

**Cons**:
- Newer API, less community resources
- Requires Google Cloud account
- Variable availability by region

**Cost Estimate for Module 4**:
- Per student exercise: ~$0.01-0.04
- Full mini-project: ~$0.40-0.80
- **Note**: Most cost-effective commercial option

**Integration Complexity**: ⭐⭐⭐⭐ (Easy)

**Recommended for Module 4**: ✅ **Budget-friendly option**

---

### Open Source (LLaMA 3, Mistral, LLaVA)

**Pros**:
- Completely free (no API costs)
- Full control over deployment
- Offline capability
- No rate limits

**Cons**:
- Requires GPU for reasonable performance (8GB+ VRAM)
- Complex setup (model download, quantization, serving)
- Variable output quality
- Limited vision capabilities (need LLaVA or similar)
- Requires manual safety filtering

**Resource Requirements**:
- GPU: 8GB+ VRAM (for 7B models), 16GB+ (for 13B models)
- Storage: 10-20GB per model
- RAM: 16GB+ system memory

**Integration Complexity**: ⭐⭐ (Complex - requires local inference setup)

**Recommended for Module 4**: ⚠️ **Optional advanced topic** (not primary path)

---

## Recommendation for Module 4

### Primary Implementation: **Hybrid Approach**

**Real API (Primary)**:
- Use OpenAI GPT-4 Vision, Anthropic Claude 3, or Google Gemini
- Provide setup guides for all three options
- Students choose based on preference and budget
- Examples use whichever API is available

**Mock Planner (Fallback)**:
- Scripted responses with JSON templates
- Covers common scenarios from mini-project
- No API key required
- Enables offline learning and debugging

**Implementation Strategy**:
```python
# Configuration-based switching
if config.use_mock_planner:
    planner = MockVLAPlanner()
else:
    if config.llm_provider == "openai":
        planner = OpenAIVLAPlanner(api_key=config.api_key)
    elif config.llm_provider == "anthropic":
        planner = AnthropicVLAPlanner(api_key=config.api_key)
    elif config.llm_provider == "google":
        planner = GoogleVLAPlanner(api_key=config.api_key)
```

---

## Cost Management Strategies

### For Students

1. **Response Caching**:
   - Cache LLM responses for identical visual scenes
   - Reduce redundant API calls during debugging

2. **Prompt Optimization**:
   - Use concise prompts with clear structure
   - Minimize token usage while maintaining quality

3. **Model Selection**:
   - Use smaller models for simple tasks (Gemini Flash, GPT-3.5)
   - Reserve GPT-4/Claude Opus for complex scenarios

4. **Free Tier Usage**:
   - Start with free tier limits
   - Most providers offer free credits for new users
   - Google Gemini has generous free tier

5. **Mock Planner for Development**:
   - Use mock planner for code development and testing
   - Switch to real API only for final validation

### Cost Estimates Per Student

**Conservative Usage** (Mock planner for development, real API for validation):
- Total API cost: **$0.50 - $2.00** for entire module

**Regular Usage** (Real API throughout):
- Total API cost: **$2.00 - $5.00** for entire module

**Heavy Usage** (Extensive experimentation):
- Total API cost: **$5.00 - $10.00** for entire module

---

## API Setup Difficulty

### OpenAI
```
1. Create account at platform.openai.com
2. Add payment method
3. Generate API key
4. Set environment variable: OPENAI_API_KEY
5. Install library: pip install openai
```
**Time**: 10-15 minutes

### Anthropic
```
1. Create account at console.anthropic.com
2. Add payment method
3. Generate API key
4. Set environment variable: ANTHROPIC_API_KEY
5. Install library: pip install anthropic
```
**Time**: 10-15 minutes

### Google Gemini
```
1. Create Google Cloud account
2. Enable Gemini API
3. Generate API key (or use OAuth)
4. Set environment variable: GOOGLE_API_KEY
5. Install library: pip install google-generativeai
```
**Time**: 15-20 minutes

---

## Final Recommendation

**Module 4 Implementation**:

✅ **Primary**: OpenAI GPT-4 Vision
- Best balance of cost, performance, ease of use
- Extensive documentation and community support
- Reliable JSON mode

✅ **Alternative 1**: Google Gemini Pro Vision
- Most cost-effective
- Good for budget-conscious students

✅ **Alternative 2**: Anthropic Claude 3 Sonnet
- Best reasoning capabilities
- Excellent safety features

✅ **Fallback**: Mock Planner
- For offline work
- For students without API access
- For debugging and development

---

## Implementation Notes

### JSON Schema Validation

All LLM providers support structured output, but quality varies:
- **OpenAI JSON mode**: Most reliable, enforces valid JSON
- **Anthropic**: Very good at following schema in prompts
- **Google Gemini**: Good JSON mode support
- **Open Source**: Variable, requires strong prompting

**Module 4 Approach**:
- Define strict JSON schema for action plans
- Use JSON mode where available
- Implement robust validation regardless of provider
- Retry logic for malformed responses

### Safety Filtering

- All providers have built-in safety features
- Module 4 adds **additional safety layer**: JSON schema validation + constraint checking
- Safety validator runs **before** plan execution
- Multiple safety checkpoints prevent unsafe actions

---

## References

- OpenAI API Pricing: https://openai.com/pricing
- Anthropic API Pricing: https://www.anthropic.com/pricing
- Google Gemini Pricing: https://ai.google.dev/pricing
- LLaMA Models: https://github.com/meta-llama/llama
- LLaVA (Vision-Language): https://llava-vl.github.io/
