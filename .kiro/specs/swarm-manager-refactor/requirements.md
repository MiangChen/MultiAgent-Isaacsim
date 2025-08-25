# Requirements Document

## Introduction

This feature involves refactoring the swarm manager initialization logic to improve code organization and separation of concerns. Currently, the `Env` class in `env.py` is responsible for loading robot swarm configuration and activating robots. This logic should be moved into the `SwarmManager` class itself, and the main application should handle the async initialization directly.

## Requirements

### Requirement 1

**User Story:** As a developer, I want the SwarmManager to handle its own complete initialization, so that the Env class is not responsible for swarm-specific configuration loading.

#### Acceptance Criteria

1. WHEN SwarmManager is created THEN it SHALL have a method to handle complete async initialization
2. WHEN the async initialization method is called THEN it SHALL ensure the scene is properly assigned from env.world.scene
3. WHEN the async initialization method is called THEN it SHALL load robot swarm configuration from a specified file path
4. WHEN the async initialization method is called THEN it SHALL activate robots based on the active flag configuration
5. WHEN the initialization is complete THEN all robots SHALL be properly loaded and activated in the scene

### Requirement 2

**User Story:** As a developer, I want the Env class to be simplified by removing swarm manager initialization logic, so that it focuses only on environment-specific concerns.

#### Acceptance Criteria

1. WHEN Env._async_init is called THEN it SHALL NOT directly load robot swarm configuration
2. WHEN Env._async_init is called THEN it SHALL NOT directly activate robots
3. WHEN Env._async_init is called THEN it SHALL only set the scene reference for the swarm manager
4. WHEN Env is created THEN the swarm manager SHALL be initialized separately before or after Env creation

### Requirement 3

**User Story:** As a developer, I want the main application to control the swarm manager initialization timing, so that I have better control over the initialization sequence.

#### Acceptance Criteria

1. WHEN the main application runs THEN it SHALL call swarm manager initialization asynchronously
2. WHEN swarm manager initialization is called THEN it SHALL accept configuration file paths as parameters
3. WHEN the initialization sequence runs THEN it SHALL maintain the same functionality as the current implementation
4. WHEN the refactoring is complete THEN the application SHALL work exactly the same as before from a user perspective