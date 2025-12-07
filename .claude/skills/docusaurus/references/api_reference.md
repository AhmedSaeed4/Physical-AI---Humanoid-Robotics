# Docusaurus API Reference

## Core Configuration

### docusaurus.config.js
The main configuration file for your Docusaurus site. Export a JavaScript object with site settings.

**Required Fields:**
- `title` (string): Site title for SEO and UI
- `tagline` (string): Brief description of the site
- `url` (string): URL of your site
- `baseUrl` (string): Base URL of your site

**Common Optional Fields:**
- `favicon` (string): Path to favicon
- `organizationName` (string): GitHub organization name (for deployment)
- `projectName` (string): GitHub project name (for deployment)
- `onBrokenLinks` (string): Action for broken links ('throw', 'warn', 'ignore')
- `onBrokenMarkdownLinks` (string): Action for broken Markdown links
- `trailingSlash` (boolean): Whether to add trailing slashes to URLs
- `i18n` (object): Internationalization configuration
- `presets` (array): Docusaurus presets configuration
- `plugins` (array): Additional plugins configuration
- `themeConfig` (object): Theme-specific configuration

### Example Configuration:
```javascript
module.exports = {
  title: 'My Site',
  tagline: 'A website built with Docusaurus',
  favicon: 'img/favicon.ico',

  url: 'https://my-docusaurus-site.com',
  baseUrl: '/',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'fr', 'es'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/facebook/docusaurus/edit/main/website/',
        },
        blog: {
          showReadingTime: true,
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
};
```

## Presets

### Classic Preset
The most commonly used preset that includes docs, blog, pages, and theme.

**Configuration Options:**
- `docs` (object): Documentation plugin configuration
  - `sidebarPath` (string): Path to sidebar configuration
  - `editUrl` (string): Base URL for edit links
  - `showLastUpdateTime` (boolean): Show last update time
  - `showLastUpdateAuthor` (boolean): Show last update author
- `blog` (object): Blog plugin configuration
  - `path` (string): Path to blog directory
  - `routeBasePath` (string): URL route for blog
  - `showReadingTime` (boolean): Show reading time estimates
- `theme` (object): Theme plugin configuration
  - `customCss` (string): Path to custom CSS file

## Plugin System

### Content Plugins
- `@docusaurus/plugin-content-docs`: Documentation content
- `@docusaurus/plugin-content-blog`: Blog content
- `@docusaurus/plugin-content-pages`: Static pages
- `@docusaurus/plugin-debug`: Debug information
- `@docusaurus/plugin-sitemap`: Sitemap generation

### External Plugins
- `@docusaurus/plugin-google-gtag`: Google Analytics
- `@docusaurus/plugin-google-analytics`: Legacy Google Analytics
- `@docusaurus/plugin-sentry`: Error tracking
- `@docusaurus/plugin-ideal-image`: Optimized image loading

### Example Plugin Configuration:
```javascript
plugins: [
  [
    '@docusaurus/plugin-content-docs',
    {
      id: 'community',
      path: 'community',
      routeBasePath: 'community',
      sidebarPath: require.resolve('./sidebarsCommunity.js'),
    },
  ],
  [
    '@docusaurus/plugin-google-gtag',
    {
      trackingID: 'GA_TRACKING_ID',
      anonymizeIP: true,
    },
  ],
],
```

## Sidebar Configuration

### sidebars.js
Defines the navigation structure for documentation.

**Types of Sidebar Items:**
- `doc`: Link to a specific document
- `category`: Collapsible category with sub-items
- `link`: External or custom link
- `html`: Raw HTML content

### Example Sidebar Configuration:
```javascript
// sidebars.js
module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Docusaurus Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
    {
      type: 'category',
      label: 'Advanced',
      items: [
        'advanced/category-metadata',
        {
          type: 'category',
          label: 'Subcategory',
          items: ['advanced/subcategory/item'],
        },
      ],
    },
  ],
};
```

## Markdown Features

### Front Matter
Metadata at the beginning of Markdown files:

```markdown
---
title: My Doc Title
description: A brief description
slug: /my-custom-url
---

# Content starts here
```

**Common Front Matter Fields:**
- `title` (string): Page title (overrides first heading)
- `description` (string): SEO description
- `slug` (string): Custom URL path
- `sidebar_label` (string): Label in sidebar navigation
- `hide_table_of_contents` (boolean): Hide TOC on this page
- `draft` (boolean): Mark page as draft (not included in production)

### MDX Components
Docusaurus supports JSX in Markdown files (MDX):

```mdx
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
  <TabItem value="npm" label="npm">
    ```bash
    npm install docusaurus
    ```
  </TabItem>
  <TabItem value="yarn" label="yarn">
    ```bash
    yarn add docusaurus
    ```
  </TabItem>
</Tabs>
```

## Theme Components

### Navigation Components
- `Navbar`: Top navigation bar
- `Footer`: Site footer
- `Sidebar`: Documentation navigation
- `TOC`: Table of contents

### Content Components
- `Heading`: Semantic headings
- `Link`: Internal and external links
- `CodeBlock`: Syntax-highlighted code
- `Details`: Expandable content sections

### Example Theme Component Usage:
```jsx
import Link from '@docusaurus/Link';
import CodeBlock from '@theme/CodeBlock';

function MyComponent() {
  return (
    <div>
      <Link to="/docs/intro">Get Started</Link>
      <CodeBlock language="js">
        {`console.log('Hello, Docusaurus!');`}
      </CodeBlock>
    </div>
  );
}
```

## Deployment Commands

### Development
```bash
npm run start          # Start development server
npm run start -- --port 8080  # Start on specific port
npm run start -- --host 0.0.0.0  # Make available on network
```

### Production
```bash
npm run build          # Build static files to build/ directory
npm run serve          # Serve built site locally for testing
npm run deploy         # Deploy to GitHub Pages (if configured)
```

## Internationalization (i18n)

### Configuration
```javascript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'fr', 'es', 'zh-CN'],
  localeConfigs: {
    en: {
      label: 'English',
      direction: 'ltr',
    },
    'zh-CN': {
      label: '简体中文',
      direction: 'ltr',
    },
  },
},
```

### Translation File Structure
```
website/
├── i18n/
│   ├── en/
│   │   ├── docusaurus-plugin-content-docs/
│   │   │   └── current/
│   │   │       ├── intro.md
│   │   │       └── tutorial-basics/
│   │   └── docusaurus-theme-classic/
│   │       └── navbar.json
│   └── fr/
│       └── ... (same structure as en)
└── src/
    └── pages/
        └── i18n/
            └── current/
                └── hello.md
```

## Versioning

### Configuration
```javascript
presets: [
  [
    'classic',
    {
      docs: {
        versions: {
          current: {
            label: 'Next',
            path: 'next',
            banner: 'unreleased',
          },
          '1.5': {
            label: '1.5.0',
            path: 'v1.5',
            banner: 'none',
          },
        },
      },
    },
  ],
],
```

## Environment and Build Tools

### Package Management with uv
When working with Docusaurus projects that include Python-based tooling:

```bash
# Install Node.js dependencies using standard tools
npm install

# For Python-based documentation tooling (if needed)
uv add --group docs @docusaurus/python-tools
```

### Environment Variables
- `USE_SSH=true`: Use SSH for GitHub Pages deployment
- `GIT_USER=username`: Git user for deployment
- `DEPLOYMENT_BRANCH=branch`: Branch to deploy to
- `CURRENT_BRANCH=branch`: Branch to deploy from

## Common CLI Commands

### Create New Site
```bash
npx create-docusaurus@latest my-website classic
```

### Generate Components
```bash
# Create new blog post
npm run write-translations  # Generate translation files
```

### Build Optimization
```bash
npm run build -- --out-dir=custom-build  # Custom output directory
npm run build -- --minify  # Explicitly enable minification
```