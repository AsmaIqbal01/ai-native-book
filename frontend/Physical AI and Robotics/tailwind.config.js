/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./docs/**/*.mdx",
    "./docs/**/*.md",
    "./docusaurus.config.ts",
  ],
  darkMode: ['class', '[data-theme="dark"]'], // Match Docusaurus dark mode
  theme: {
    extend: {},
  },
  plugins: [],
  corePlugins: {
    preflight: false, // Disable Tailwind's base styles to avoid conflicts with Docusaurus
  },
}