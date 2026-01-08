# 🔒 Security Guide - Environment Variables & API Keys

**CRITICAL**: This project uses sensitive API keys that must NEVER be committed to version control.

---

## ✅ Current Security Status

### Good News
- ✅ `.env` is properly listed in `.gitignore`
- ✅ `.env` was never committed to git history
- ✅ Environment variables are loaded via `pydantic-settings` (secure pattern)

### ⚠️ Security Alert
**Your `.env` file currently contains exposed API keys.**

If you've shared this file, shared your screen, or copied these keys anywhere, **rotate them immediately**.

---

## 🚨 Immediate Action Required

### 1. Rotate All API Keys (DO THIS NOW)

Replace the following keys in your `.env` file:

#### OpenRouter
- **Current**: `sk-or-v1-a75f154ef...` (redacted - check your local .env file)
- **Action**: Visit https://openrouter.ai/keys → Delete old key → Generate new key

#### OpenAI
- **Current**: `sk-proj-s33WDm8AMr...` (redacted - check your local .env file)
- **Action**: Visit https://platform.openai.com/api-keys → Revoke old key → Create new key

#### Qdrant
- **Current**: `eyJhbGciOiJIUzI1...` (redacted - check your local .env file)
- **Action**: Visit Qdrant Cloud → API Keys → Regenerate

---

## 📋 .gitignore Configuration (Already Secure)

Your `.gitignore` already contains:
```gitignore
.env
.env.local
.env.*.local
```

**DO NOT remove these entries.**

---

## 🛡️ Best Practices

### For Local Development

1. **Never hardcode secrets in code**
   ```python
   # ❌ BAD
   api_key = "sk-or-v1-abc123..."

   # ✅ GOOD
   from app.config import settings
   api_key = settings.llm_api_key
   ```

2. **Use .env.example as a template**
   ```bash
   # Copy example and fill in your keys
   cp .env.example .env
   # Edit .env with your actual keys (NEVER commit this file)
   ```

3. **Verify .env is ignored before committing**
   ```bash
   git status
   # .env should NOT appear in the list
   ```

### For Production Deployment

**DO NOT use `.env` files in production.** Use platform-specific secrets management:

#### Render (Recommended)
```
Dashboard → Environment → Environment Variables
- Add each variable individually
- Never expose in logs or screenshots
```

#### Railway
```
Project Settings → Variables
- Add as key-value pairs
- Use Railway CLI for bulk import
```

#### Hugging Face Spaces
```
Settings → Repository Secrets
- Add secrets in dashboard
- Reference in Dockerfile as ARG
```

#### Docker
```bash
# Pass environment variables at runtime
docker run -e LLM_API_KEY=$LLM_API_KEY ...

# Or use Docker secrets (production)
echo "your-api-key" | docker secret create llm_api_key -
```

---

## 🔍 How to Check for Accidental Exposure

### Check Git History
```bash
# Search for .env in entire history
git log --all --full-history -- .env

# If results appear, .env was committed - IMMEDIATE ACTION REQUIRED
```

### Check Staged Files Before Commit
```bash
git status
# Ensure .env is NOT listed under "Changes to be committed"
```

### Scan for Hardcoded Keys
```bash
# Search for potential API keys in code
grep -r "sk-" app/ --include="*.py"
grep -r "api_key.*=" app/ --include="*.py"

# Should only find: settings.llm_api_key (safe)
# Should NOT find: api_key = "sk-..." (unsafe)
```

---

## 🚀 Safe Environment Variable Loading (Current Pattern)

Your project already uses the secure pattern via `pydantic-settings`:

```python
# app/config/config.py
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    llm_api_key: str
    openai_api_key: str | None = None
    qdrant_api_key: str

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
    )

settings = Settings()  # Automatically loads from .env
```

**Usage in code**:
```python
from app.config import settings

# ✅ Safe
api_key = settings.llm_api_key

# ❌ Never do this
print(f"API Key: {settings.llm_api_key}")  # Logs expose secrets
```

---

## 🎯 Pre-Commit Checklist

Before every commit, verify:
- [ ] `.env` is NOT in `git status` output
- [ ] No hardcoded API keys in code (search for `"sk-`, `"ey`, `"api_key =`)
- [ ] No secrets in print/log statements
- [ ] `.env.example` contains only placeholders
- [ ] README mentions "Copy .env.example to .env"

---

## 🔔 Warning for Contributors

**⚠️ NEVER commit the `.env` file**

If you see `.env` in your git status:
```bash
# Remove from staging
git restore --staged .env

# If accidentally committed
git reset --soft HEAD~1  # Undo last commit (keeps changes)
```

**⚠️ NEVER share screenshots containing**:
- Terminal output with API keys visible
- `.env` file contents
- Environment variable listings

**⚠️ NEVER paste keys in**:
- GitHub Issues
- Pull Request descriptions
- Slack/Discord messages
- Documentation files committed to git

---

## 🆘 Emergency Response: Key Leaked

If you accidentally expose an API key:

1. **Immediate**: Rotate the key at the provider's dashboard
2. **Review**: Check where the key was exposed (PR, issue, screenshot)
3. **Delete**: Remove the exposed content if possible
4. **Update**: Update your local `.env` with the new key
5. **Monitor**: Watch provider billing for unusual activity

---

## 📚 Additional Resources

- [12-Factor App: Config](https://12factor.net/config)
- [OWASP: Protecting Secrets](https://cheatsheetseries.owasp.org/cheatsheets/Secrets_Management_Cheat_Sheet.html)
- [GitHub: Removing Sensitive Data](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/removing-sensitive-data-from-a-repository)

---

## ✅ Summary

**Your current setup is secure** - `.env` is properly ignored and was never committed.

**ACTION REQUIRED**: Rotate the API keys listed in section 1 if you've shared the `.env` file or this repository with anyone.

**Going Forward**:
- Keep `.env` in `.gitignore` (already done ✅)
- Use platform secrets for production (see deployment section ✅)
- Never hardcode keys in code (already following ✅)
- Rotate keys if accidentally exposed

---

**Last Updated**: 2026-01-08
**Security Audit**: Claude Sonnet 4.5
