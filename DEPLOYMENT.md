# Deployment Guide

This project is configured for dual deployment:

## 🚀 Vercel Deployment

The project includes `vercel.json` configuration for automatic Vercel deployment.

**Configuration:**
- **Root Directory:** `frontend/Physical AI and Robotics`
- **Build Command:** `npm run build`
- **Output Directory:** `./build`
- **Framework:** Docusaurus (static site)

**Environment Detection:**
The `docusaurus.config.ts` automatically detects the deployment platform:
- On Vercel: Uses `baseUrl: '/'`
- On GitHub Pages: Uses `baseUrl: '/ai-native-book/'`

**Quick Deploy:**
1. Import repository to Vercel: https://vercel.com/new
2. Vercel auto-detects `vercel.json` settings
3. Click "Deploy"

## 📄 GitHub Pages Deployment

**URL:** https://asmaiqbal01.github.io/ai-native-book/

**Workflow:** `.github/workflows/deploy.yml`
- Triggers on push to `main` branch
- Builds from `frontend/Physical AI and Robotics/`
- Deploys to GitHub Pages automatically

## 🔧 Local Development

```bash
cd "frontend/Physical AI and Robotics"
npm install
npm start
```

## 📝 Notes

- Both deployments work simultaneously
- Configuration is environment-aware
- GitHub Pages uses subpath `/ai-native-book/`
- Vercel uses root path `/`

---

**Last Updated:** December 11, 2024
