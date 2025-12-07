# Docusaurus - Real World Scenarios

## Scenario 1: Open Source Project Documentation Site

### Problem
An open source project needs comprehensive documentation with versioning support, search functionality, and multi-language support to serve a global community of developers.

### Solution
```javascript
// docusaurus.config.js
module.exports = {
  title: 'My Open Source Project',
  tagline: 'A powerful library for modern development',
  url: 'https://my-project.github.io',
  baseUrl: '/',
  organizationName: 'my-org',
  projectName: 'my-project',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'zh', 'ja', 'es', 'fr'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/my-org/my-project/edit/main/website/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
          versions: {
            current: {
              label: 'Next',
              path: 'next',
            },
          },
        },
        blog: {
          showReadingTime: true,
          editUrl: 'https://github.com/my-org/my-project/edit/main/website/blog/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

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
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/my-project-social-card.jpg',
      navbar: {
        title: 'My Project',
        logo: {
          alt: 'My Project Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Docs',
          },
          {to: '/blog', label: 'Blog', position: 'left'},
          {
            type: 'doc',
            position: 'left',
            docId: 'community/index',
            label: 'Community',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/my-org/my-project',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Getting Started',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/my-project',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/my-project',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Blog',
                to: '/blog',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/my-org/my-project',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer/themes/github'),
        darkTheme: require('prism-react-renderer/themes/dracula'),
      },
    }),
};
```

## Scenario 2: Company Product Documentation with Versioning

### Problem
A company needs to maintain documentation for multiple versions of their product while providing a consistent user experience and easy navigation between versions.

### Solution
```javascript
// docusaurus.config.js for product documentation
module.exports = {
  title: 'Acme Product Documentation',
  tagline: 'Comprehensive documentation for Acme Product',
  url: 'https://docs.acme.com',
  baseUrl: '/',

  // Versioning configuration
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/acme/docs/edit/main/',
          // Versioning configuration
          versions: {
            current: {
              label: '2.0.0 (Current)',
              path: 'v2.0',
              banner: 'none',
            },
            '1.5': {
              label: '1.5.0',
              path: 'v1.5',
              banner: 'unmaintained',
            },
            '1.0': {
              label: '1.0.0',
              path: 'v1.0',
              banner: 'unmaintained',
            },
          },
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Acme Product Docs',
      logo: {
        alt: 'Acme Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          type: 'docsVersionDropdown',
          position: 'left',
          dropdownActiveClassDisabled: true,
        },
        {
          type: 'docSidebar',
          sidebarId: 'apiSidebar',
          position: 'left',
          label: 'API Reference',
        },
        {
          type: 'docSidebar',
          sidebarId: 'tutorialsSidebar',
          position: 'left',
          label: 'Tutorials',
        },
      ],
    },
  },
};
```

## Scenario 3: Technical Blog with Documentation Integration

### Problem
A technical company wants to combine their developer blog with comprehensive API documentation in a single, cohesive site.

### Solution
```javascript
// docusaurus.config.js for technical blog + docs
module.exports = {
  title: 'TechCorp Developer Portal',
  tagline: 'API Documentation and Developer Resources',
  url: 'https://developer.techcorp.com',
  baseUrl: '/',

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: 'docs',
        },
        blog: {
          routeBasePath: 'blog',
          path: './blog',
          showReadingTime: true,
          blogSidebarTitle: 'All posts',
          blogSidebarCount: 'ALL',
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  plugins: [
    // API reference plugin
    [
      'docusaurus-plugin-openapi',
      {
        id: 'api',
        path: 'openapi.json',
        routeBasePath: 'api',
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'TechCorp Developer',
      logo: {
        alt: 'TechCorp Logo',
        src: 'img/logo.svg',
      },
      items: [
        {to: '/docs', label: 'Docs', position: 'left'},
        {to: '/api', label: 'API', position: 'left'},
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          type: 'search',
          position: 'right',
        },
      ],
    },
  },
};
```

## Scenario 4: Multi-Language Documentation Site

### Problem
A global company needs to provide documentation in multiple languages while maintaining a consistent structure and easy maintenance.

### Solution
```javascript
// docusaurus.config.js for multi-language site
module.exports = {
  title: 'Global Product Documentation',
  tagline: 'Documentation in your language',

  // Internationalization configuration
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'es', 'fr', 'de', 'ja', 'ko', 'zh-CN'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      es: {
        label: 'Español',
        direction: 'ltr',
      },
      fr: {
        label: 'Français',
        direction: 'ltr',
      },
      // ... other languages
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://translate.example.com/projects/product/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Product Docs',
      items: [
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Documentation',
        },
      ],
    },
  },
};

// Translation file structure
// i18n/es/docusaurus-plugin-content-docs/current/intro.md
// i18n/fr/docusaurus-plugin-content-docs/current/intro.md
// etc.
```

## Scenario 5: Custom Themed Documentation Site

### Problem
A company wants to create documentation that matches their brand identity with custom styling and components while maintaining Docusaurus functionality.

### Solution
```javascript
// src/css/custom.css
:root {
  --ifm-color-primary: #25c2a0;
  --ifm-color-primary-dark: rgb(33, 175, 144);
  --ifm-color-primary-darker: rgb(31, 165, 136);
  --ifm-color-primary-darkest: rgb(26, 136, 112);
  --ifm-color-primary-light: rgb(70, 203, 174);
  --ifm-color-primary-lighter: rgb(102, 212, 189);
  --ifm-color-primary-lightest: rgb(146, 224, 208);
  --ifm-code-font-size: 95%;
}

/* Custom component styles */
.hero--primary {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

.custom-card {
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: var(--ifm-global-radius);
  padding: var(--ifm-global-spacing);
  margin-bottom: var(--ifm-global-spacing);
  box-shadow: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
}

// Custom React component in src/components/FeatureCard.js
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import './FeatureCard.css';

export default function FeatureCard({title, description, link, icon}) {
  return (
    <div className={clsx('custom-card')}>
      <div className="feature-card-icon">{icon}</div>
      <h3>{title}</h3>
      <p>{description}</p>
      {link && (
        <Link to={link}>
          Learn more →
        </Link>
      )}
    </div>
  );
}
```

## Key Implementation Patterns

### 1. Progressive Enhancement
- Start with basic documentation structure
- Add advanced features (search, versioning, i18n) as needed
- Ensure core content is accessible without JavaScript

### 2. Consistent Navigation
- Use consistent sidebar structures across sections
- Implement clear breadcrumbs for deep content
- Provide search functionality for large sites

### 3. Performance Optimization
- Optimize images and assets
- Use code splitting for large documentation sets
- Implement proper caching strategies

### 4. Maintenance Strategy
- Set up automated deployment workflows
- Implement content versioning for long-term maintenance
- Create clear documentation contribution guidelines