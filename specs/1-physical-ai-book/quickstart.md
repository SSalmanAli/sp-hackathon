# Quickstart: Physical AI Book Development

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Git for version control
- A code editor (VS Code recommended)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Docusaurus Dependencies
```bash
cd docusaurus
npm install
```

### 3. Initialize the Docusaurus Project
```bash
# If starting fresh
npx create-docusaurus@latest docusaurus classic --typescript
```

### 4. Start Local Development Server
```bash
npm run start
```

This will start a local development server at `http://localhost:3000` with live reloading.

## Project Structure

The Physical AI book follows this structure:

```
docusaurus/
├── docs/                    # All book content
│   ├── module-1-ros2/       # Module 1 content
│   ├── module-2-simulation/ # Module 2 content
│   ├── module-3-isaac/      # Module 3 content
│   ├── module-4-vla/        # Module 4 content
│   ├── assets/              # Diagrams, images, code examples
│   └── reusable-intelligence/ # Templates and guidelines
├── src/                     # Custom components
├── static/                  # Static assets
├── docusaurus.config.ts     # Site configuration
├── sidebars.ts              # Navigation structure
└── package.json             # Dependencies
```

## Creating New Content

### Adding a New Lesson
1. Create a new markdown file in the appropriate module folder
2. Follow the naming convention: `lesson-X-Y-title.md`
3. Include frontmatter with metadata:
   ```markdown
   ---
   title: Lesson Title
   sidebar_position: Y
   description: Brief description of the lesson
   ---
   ```

### Adding Assets
1. Place diagrams, images, or code examples in the `docs/assets/` folder
2. Organize by type in subfolders (diagrams/, images/, code-examples/)
3. Reference assets using relative paths: `![alt text](/img/diagram-name.svg)`

## Building and Deployment

### Build for Production
```bash
npm run build
```

### Deploy to GitHub Pages
```bash
npm run deploy
```

## Key Configuration Files

- `docusaurus.config.ts`: Site-wide configuration including title, theme, and plugins
- `sidebars.ts`: Navigation structure and content organization
- `src/css/custom.css`: Custom styling overrides

## Next Steps

1. Review the detailed implementation plan in `plan.md`
2. Check the content model in `data-model.md`
3. Begin creating content following the reusable templates
4. Test locally using `npm run start`