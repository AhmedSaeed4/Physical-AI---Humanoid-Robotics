# Contributing

Thank you for your interest in contributing to the Physical AI & Humanoid Robotics Book project! This document outlines the guidelines for contributing to this educational resource.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Documentation Guidelines](#documentation-guidelines)
- [Technical Requirements](#technical-requirements)
- [Pull Request Process](#pull-request-process)

## Code of Conduct

Please read and follow our [Code of Conduct](CODE_OF_CONDUCT.md) to ensure a welcoming environment for all contributors.

## Getting Started

1. Fork the repository
2. Clone your fork: `git clone https://github.com/your-username/docusaurus-project.git`
3. Navigate to the project: `cd docusaurus-project`
4. Install dependencies: `cd frontend && npm install`
5. Create a new branch: `git checkout -b feature/your-feature-name`

## Development Workflow

### Documentation Structure

The book content is organized in `frontend/docs/` following this structure:

```
frontend/docs/
├── Part_I_Infrastructure/
│   ├── Chapter_1_Hardware_OS_Config/
│   └── Chapter_2_Edge_Ecosystem/
├── Part_II_ROS/
│   ├── Chapter_3_ROS_Architecture/
│   └── Chapter_4_Body/
├── Part_III_Simulation/
│   ├── Chapter_5_Physics_Gazebo/
│   └── Chapter_6_Physically_Realistic_Sim/
├── Part_IV_Perception/
│   ├── Chapter_7_Sensors_VSLAM/
│   └── Chapter_8_Navigation/
├── Part_V_AI_Integration/
│   ├── Chapter_9_Voice_Pipeline/
│   ├── Chapter_10_Brain/
│   └── Chapter_11_VLA/
└── Part_VI_Capstone/
    └── Chapter_12_Autonomous_Humanoid/
```

### Creating New Content

When adding new chapters or sections:

1. Create the appropriate directory structure
2. Follow the existing markdown format and conventions
3. Include code examples where relevant
4. Add appropriate diagrams (Mermaid.js preferred)
5. Update the sidebar configuration in `frontend/sidebars.ts`

### ROS 2 Code Examples

For ROS 2 code examples (in the physical_ai_ws workspace):

1. Follow ROS 2 coding standards
2. Include comprehensive comments explaining the educational purpose
3. Add proper error handling as specified in the requirements
4. Use Python 3.10+ for primary examples, C++17 for performance-critical nodes

## Documentation Guidelines

- Write in clear, accessible language suitable for CS/Robotics Engineering students
- Include practical examples and code snippets
- Use consistent terminology throughout
- Provide links to external resources where appropriate
- Include diagrams to illustrate complex concepts (prefer Mermaid.js diagrams)

## Technical Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA drivers (535+)
- CUDA 12.2
- Node.js 18+ for Docusaurus frontend

## Pull Request Process

1. Ensure your PR addresses a specific issue or adds clear value
2. Update the README.md with details of changes if needed
3. Add yourself to the contributors list if appropriate
4. Ensure all documentation builds correctly
5. Submit your pull request with a clear description of your changes

## Questions?

If you have questions, please open an issue in the repository.