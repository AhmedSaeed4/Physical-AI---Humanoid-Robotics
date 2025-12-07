# Docusaurus Skill

## Overview
This skill provides comprehensive knowledge and capabilities for working with Docusaurus, a static-site generator for building beautiful documentation websites. It enables Claude to help users create, configure, customize, and deploy Docusaurus sites with features like versioning, internationalization, and search.

## Purpose
This skill allows Claude to:
- Explain Docusaurus concepts and architecture
- Help create and configure Docusaurus sites
- Customize themes, layouts, and styling
- Implement versioning and internationalization
- Deploy and maintain Docusaurus sites
- Troubleshoot common Docusaurus issues

## Quick Start
When a user needs help with Docusaurus:

1. First, understand their specific documentation needs
2. Suggest appropriate Docusaurus template (classic, blog, custom)
3. Provide setup instructions using recommended package manager
4. Guide them through configuration and customization

## Core Capabilities

### Site Creation
- Initialize new Docusaurus sites with various templates
- Set up project structure and dependencies
- Configure basic site metadata and settings

### Configuration Management
- Configure site metadata (title, tagline, favicon)
- Set up navigation and sidebar organization
- Configure plugins (search, versioning, i18n)

### Content Organization
- Organize documentation hierarchically (pages, sidebars, versions, plugin instances)
- Create and structure markdown content
- Implement versioning for documentation

### Customization
- Customize themes and styling with CSS/SCSS
- Create custom components and layouts
- Extend functionality with plugins

### Internationalization
- Set up multi-language support
- Configure translation workflows
- Manage locale-specific content

### Deployment
- Build optimized production sites
- Deploy to various platforms (GitHub Pages, Vercel, Netlify, etc.)
- Configure custom domains and hosting

## Installation & Setup
Docusaurus can be installed and set up using various package managers:

```bash
# Create a new Docusaurus site with the classic template
npx create-docusaurus@latest my-website classic

# Navigate to the project directory
cd my-website

# Start the development server
npm run start
# or with other package managers:
yarn run start
pnpm run start
bun run start
```

## Usage Examples
- "How do I create a new Docusaurus site?"
- "Show me how to add versioning to my documentation"
- "Help me set up internationalization for multiple languages"
- "I need to customize the theme and styling of my site"
- "How do I add a search functionality to my Docusaurus site?"

## Advanced Features
- Plugin system for extending functionality
- MDX support for interactive content
- Static site generation with React
- Built-in optimization and performance features