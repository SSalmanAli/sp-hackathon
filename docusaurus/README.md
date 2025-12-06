# Physical AI Book

This is a Docusaurus-based documentation site for the Physical AI Book, a comprehensive, hands-on field manual for building and understanding real Physical AI systems.

## Overview

The Physical AI Book teaches readers how to build and understand real Physical AI systems through practical, implementation-ready lessons. The book covers:
- ROS 2 fundamentals and robot modeling
- Digital twin simulation with Gazebo and Unity
- NVIDIA Isaac ecosystem and navigation
- Vision-Language-Action systems and humanoid autonomy

## Prerequisites

- Node.js version 18 or higher
- npm or yarn package manager

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static content hosting service.

## Deployment

The site is configured for deployment to GitHub Pages. Update the `organizationName` and `projectName` in `docusaurus.config.ts` to match your repository before deploying.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## Directory Structure

- `docs/` - All book content organized by modules and lessons
- `src/` - Custom React components and site source code
- `static/` - Static assets like images
- `docusaurus.config.ts` - Site configuration
- `sidebars.ts` - Navigation structure

## License

This project is licensed under the MIT License.