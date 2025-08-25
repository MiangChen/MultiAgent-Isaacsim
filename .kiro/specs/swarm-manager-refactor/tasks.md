# Implementation Plan

- [x] 1. Add new async initialization method to SwarmManager
  - Create `initialize_async()` method in `SwarmManager` class
  - Accept scene, robot_swarm_cfg_path, and robot_active_flag_path parameters
  - Implement scene assignment, config loading, and robot activation logic
  - Add proper error handling and validation
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5_

- [x] 2. Simplify Env class async initialization
  - Remove swarm manager configuration loading from `Env._async_init()`
  - Remove robot activation logic from `Env._async_init()`
  - Keep only essential environment initialization in `_async_init()`
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [ ] 3. Update main application to use new initialization pattern
  - Modify `setup_simulation()` function in `main.py`
  - Call swarm manager initialization after Env creation
  - Pass proper configuration file paths to swarm manager
  - Ensure proper error handling in the initialization sequence
  - _Requirements: 3.1, 3.2, 3.3, 3.4_

- [ ] 4. Test the refactored initialization
  - Verify that robots are still properly loaded and activated
  - Test that the simulation runs exactly as before
  - Check that all robot types (jetbot, h1, cf2x) work correctly
  - Validate error handling for missing configuration files
  - _Requirements: 1.5, 3.4_