# Implementation Plan

- [x] 1. Install and setup dependency-injector framework
  - Install dependency-injector package via pip
  - Verify installation and import capabilities
  - Create basic project structure for dependency injection
  - _Requirements: 1.1, 1.2_

- [x] 2. Create AppContainer with core providers
  - [x] 2.1 Create containers.py file with AppContainer class
    - Implement AppContainer extending DeclarativeContainer
    - Add configuration provider for YAML config loading
    - Add basic singleton providers for ViewportManager and MapSemantic
    - _Requirements: 1.1, 1.3_

  - [x] 2.2 Add GridMap provider with configuration injection
    - Create GridMap singleton provider with config-based parameters
    - Map configuration keys to GridMap constructor parameters
    - Test GridMap creation with injected configuration
    - _Requirements: 1.2, 6.1, 6.2_

  - [x] 2.3 Add manager providers with dependency injection
    - Create SceneManager provider with ViewportManager dependency
    - Create SwarmManager provider with GridMap dependency
    - Verify dependency resolution works correctly
    - _Requirements: 1.2, 3.1, 3.2_

- [x] 3. Implement container setup and configuration loading
  - [x] 3.1 Create container initialization function
    - Implement create_container() function
    - Add YAML configuration loading and injection
    - Add module wiring for dependency injection
    - _Requirements: 6.1, 6.2_

  - [x] 3.2 Add error handling for container setup
    - Implement safe container initialization with try-catch
    - Add meaningful error messages for configuration issues
    - Test container setup with invalid configurations
    - _Requirements: 4.4_

- [x] 4. Refactor main.py to use dependency injection
  - [x] 4.1 Extract business logic into injectable functions
    - Create setup_simulation function with @inject decorator
    - Create create_car_objects function with @inject decorator
    - Create process_semantic_detection function with @inject decorator
    - _Requirements: 2.2, 2.3_

  - [x] 4.2 Update main function to use container
    - Replace manual manager instantiation with container setup
    - Update function calls to use injected business logic functions
    - Remove manual dependency passing between functions
    - _Requirements: 2.1, 2.2_

  - [x] 4.3 Update ROS integration to use dependency injection
    - Modify ROS callback functions to access managers through container
    - Update global semantic map reference to use injected instance
    - Ensure ROS skill execution uses injected SwarmManager
    - _Requirements: 5.1, 5.2, 5.3_

- [x] 5. Test dependency injection implementation
  - [x] 5.1 Create unit tests with mocked dependencies
    - Write tests for SwarmManager with mocked GridMap
    - Write tests for SceneManager with mocked ViewportManager
    - Verify provider override functionality works correctly
    - _Requirements: 4.1, 4.2_

  - [x] 5.2 Create integration tests for full dependency chain
    - Test complete container setup and wiring
    - Verify all managers receive correct dependencies
    - Test singleton behavior across multiple requests
    - _Requirements: 4.3, 1.3_

  - [x] 5.3 Test configuration injection and error handling
    - Test container behavior with missing configuration
    - Test container behavior with invalid configuration
    - Verify error messages are clear and actionable
    - _Requirements: 6.3, 4.4_

- [x] 6. Validate backward compatibility and performance
  - [x] 6.1 Verify existing manager functionality unchanged
    - Test that all existing manager methods work as before
    - Verify no breaking changes to manager APIs
    - Test manual manager instantiation still works
    - _Requirements: 1.4, 3.3_

  - [x] 6.2 Performance validation and optimization
    - Measure container setup time and memory usage
    - Compare performance with manual dependency management
    - Optimize any performance bottlenecks found
    - _Requirements: 1.3_

- [x] 7. Update documentation and finalize implementation
  - [x] 7.1 Create usage documentation
    - Document how to use the new dependency injection system
    - Provide examples of adding new services to the container
    - Document testing patterns with dependency injection
    - _Requirements: 4.1, 4.2_

  - [x] 7.2 Clean up and finalize code
    - Remove any unused imports or code
    - Add proper type hints and docstrings
    - Ensure code follows project style guidelines
    - _Requirements: 2.4_