# ADR 004: Personalization

## Status
Accepted

## Context
The coursebook could include optional personalization features to enhance the learning experience. There was a decision to be made about whether to implement user accounts and personalized learning paths.

## Decision
We will implement optional Better-Auth signup for user personalization as a bonus feature, including:

- User accounts for tracking progress
- Personalized learning paths
- Saved notes and bookmarks
- Progress tracking
- Customizable settings

This will be optional and not required for core functionality.

## Rationale
Personalization enhances the learning experience by allowing students to track their progress and customize their learning environment. Implementing as an optional feature allows for broader adoption while providing enhanced functionality for interested users.

## Consequences
- Positive: Enhanced user experience for those who enable it
- Positive: Progress tracking and personalization features
- Negative: Additional complexity for optional feature
- Negative: Additional dependencies and setup