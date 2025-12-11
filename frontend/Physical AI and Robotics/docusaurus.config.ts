import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'AI-Native Robotics',
  tagline: 'Building the Future of Physical AI - From ROS2 to Digital Twins',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  // Use VERCEL_URL for Vercel deployments, fallback to GitHub Pages
  url: process.env.VERCEL_URL
    ? `https://${process.env.VERCEL_URL}`
    : 'https://asmaiqbal01.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // Vercel uses '/', GitHub Pages uses '/ai-native-book/'
  baseUrl: process.env.VERCEL ? '/' : (process.env.NODE_ENV === 'production' ? '/ai-native-book/' : '/'),

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'AsmaIqbal01', // Usually your GitHub org/user name.
  projectName: 'ai-native-book', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: 'docs',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: false, // Disable blog plugin
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
  title: 'AI-Native Robotics',
  logo: {
    alt: 'AI-Native Robotics Logo',
    src: 'img/logo.svg',
  },
  items: [
    {
      type: 'docSidebar',
      sidebarId: 'docsSidebar',
      position: 'left',
      label: 'Textbook',
    },
    {
      to: '/translator',
      label: 'Urdu Translator',
      position: 'left',
    },
    {
      to: '/login',
      label: 'Login',
      position: 'right',
    },
    {
      href: 'https://github.com/AsmaIqbal01/ai-native-book',
      label: 'GitHub',
      position: 'right',
    },
  ],
},
footer: {
  style: 'dark',
  links: [
    {
      title: 'Learn',
      items: [
        {
          label: 'Get Started',
          to: '/docs/introduction',
        },
        {
          label: 'Physical AI Fundamentals',
          to: '/docs/chapter1/physical-ai',
        },
        {
          label: 'ROS2 Basics',
          to: '/docs/chapter2/ros2-intro',
        },
      ],
    },
    {
      title: 'Community',
      items: [
        {
          label: 'GitHub',
          href: 'https://github.com/AsmaIqbal01/ai-native-book',
        },
        {
          label: 'Discord',
          href: 'https://discordapp.com/invite/docusaurus',
        },
        {
          label: 'Twitter',
          href: 'https://twitter.com/docusaurus',
        },
      ],
    },
    {
      title: 'More',
      items: [
        {
          label: 'Resources',
          to: '/docs/resources/references',
        },
        {
          label: 'GitHub',
          href: 'https://github.com/AsmaIqbal01/ai-native-book',
        },
      ],
    },
  ],
  copyright: `Copyright © ${new Date().getFullYear()} AI-Native Robotics Book. Built with Docusaurus.`,
},
  prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'markup', 'cpp', 'cmake'],
    },
    /* algolia: {
      appId: 'YOUR_APP_ID',
      apiKey: 'YOUR_SEARCH_API_KEY',
      indexName: 'ai-native-robotics',
      contextualSearch: true,
    }, */
  } satisfies Preset.ThemeConfig,
};

export default config;
