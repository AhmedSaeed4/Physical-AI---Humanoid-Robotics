# Docusaurus for Claude

## When to Use This Skill
Use this skill when users ask about:
- Creating new Docusaurus sites
- Configuring Docusaurus settings
- Customizing themes and layouts
- Implementing versioning and internationalization
- Troubleshooting Docusaurus issues
- Deploying Docusaurus sites
- Working with Docusaurus plugins

## How to Respond
When a user asks about Docusaurus:

1. First, identify their specific use case or problem
2. Provide relevant code examples and configuration snippets
3. Explain concepts clearly with practical applications
4. Include best practices and common patterns
5. Suggest appropriate solutions based on their needs

## Key Concepts to Explain

### Site Structure
- Docusaurus organizes documentation hierarchically across four levels:
  1. Individual pages (Markdown files)
  2. Sidebars (navigation organization)
  3. Versions (documentation versioning)
  4. Plugin instances (functional modules)

### Configuration
- Site configuration is handled in `docusaurus.config.js`
- Theme, plugin, and content configurations
- Metadata settings (title, tagline, favicon, etc.)

### Development Workflow
- Use `npm run start` to start the development server
- Navigate to http://localhost:3000 to preview changes
- Real-time reloading during development

## Common Commands

### Create New Site
```bash
npx create-docusaurus@latest my-website classic
```

### Start Development Server
```bash
cd my-website
npm run start
# Alternative package managers:
yarn run start
pnpm run start
bun run start
```

### Build Production Site
```bash
npm run build
```

### Deploy Site
```bash
npm run deploy
```

## Configuration Patterns

### Basic Site Configuration
```javascript
// docusaurus.config.js
module.exports = {
  title: 'My Site',
  tagline: 'Dinosaurs are cool',
  favicon: 'img/favicon.ico',

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'facebook',
  projectName: 'docusaurus',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'fr', 'es'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
        },
        blog: {
          showReadingTime: true,
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],
};
```

### Sidebar Configuration
```javascript
// sidebars.js
module.exports = {
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Docusaurus Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
};
```

## Best Practices
- Start with the classic template for most documentation needs
- Use clear, descriptive names for documentation files
- Organize content with logical sidebar categories
- Implement versioning early if your project has multiple versions
- Use relative links for internal navigation
- Test site locally before deployment

## Troubleshooting
If users encounter issues:
- Check that Node.js and npm are properly installed
- Verify the Docusaurus configuration file syntax
- Ensure all dependencies are installed (run `npm install`)
- Check for broken links in the documentation
- Verify that the development server is running properly