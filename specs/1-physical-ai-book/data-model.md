# Data Model: Physical AI Book in Docusaurus

## Content Entities

### Module
- **name**: String (e.g., "ROS 2 Fundamentals and Robot Modeling")
- **description**: String (overview of the module)
- **learningOutcomes**: Array<String> (specific outcomes for the module)
- **lessons**: Array<Lesson> (ordered list of lessons in the module)
- **order**: Number (module number in sequence 1-4)

### Lesson
- **title**: String (e.g., "ROS 2 Architecture and Communication")
- **description**: String (what the lesson covers)
- **buildOutcome**: String (what students will build)
- **requiredTools**: Array<String> (tools needed for the lesson)
- **focus**: String (primary focus of the lesson)
- **moduleId**: String (reference to parent module)
- **order**: Number (lesson number in module 1-3)
- **prerequisites**: Array<String> (previous lessons or concepts needed)

### Asset
- **type**: String (diagram, code-example, image, video)
- **path**: String (relative path from docs/assets/)
- **description**: String (what the asset represents)
- **usage**: Array<String> (which lessons/modules use this asset)
- **tags**: Array<String> (keywords for search and organization)

### InteractiveComponent
- **type**: String (demo, code-runner, chatbot, quiz)
- **config**: Object (configuration parameters for the component)
- **placement**: String (where in the lesson it appears)
- **lessonId**: String (reference to the lesson that uses it)

## Content Relationships

### Module-Lesson Relationship
- One Module contains many Lessons (1:M)
- Lessons have a required reference to their parent Module
- Lessons are ordered within their Module (order property)

### Lesson-Asset Relationship
- One Lesson uses many Assets (1:M)
- One Asset can be used by many Lessons (M:N through usage property)
- Assets are organized by type and topic

### Content Progression
- Lessons have prerequisites that must be completed before starting
- Modules build on previous modules (except Module 1)
- Each lesson increases in complexity from the previous one

## Validation Rules

### Module Validation
- Must have a name between 5-100 characters
- Must have 3 lessons exactly
- Description must be 20-200 characters
- Learning outcomes must be 3-5 items

### Lesson Validation
- Must have a title between 10-80 characters
- Description must be 50-300 characters
- Must specify what students will build
- Must list required tools
- Must have an order value between 1-3

### Asset Validation
- Path must be valid and point to an existing file
- Type must be one of the defined asset types
- Description must be 10-100 characters

## State Transitions

### Content States
- **Draft**: Content is being written, not ready for review
- **Review**: Content is ready for peer review
- **Approved**: Content has passed review, ready for publication
- **Published**: Content is live on the site

### State Transition Rules
- Draft → Review: When initial writing is complete
- Review → Approved: After successful peer review
- Approved → Published: After site build and deployment
- Published → Review: When updates are needed