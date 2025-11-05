# Cake Code Generator Tests

This directory contains tests for the `generate_node_interface.py` script.

## Running Tests

### Quick Start

```bash
cd cake/tests
./run_tests.sh
```

### With pytest directly

```bash
cd cake/tests
pytest -v test_generate_node_interface.py
```

### Run specific test

```bash
pytest -v test_generate_node_interface.py::test_generate_node_interface[simple_node]
```

## Test Structure

Tests use a fixture-based approach where each test case consists of:
- `input.yaml` - The interface definition (committed)
- `expected_cpp/` - Expected C++ generated files (committed)
  - `{fixture_name}_interface.hpp`
  - `{fixture_name}_interface.params.yaml`
- `expected_python/` - Expected Python generated files (committed, optional)
  - `_interface.py`
  - `_parameters.py`
  - `__init__.py`
- `generated_cpp/` - Temporary C++ outputs created during test run (gitignored)
- `generated_python/` - Temporary Python outputs created during test run (gitignored)

Tests run the actual generator script as a subprocess and compare all files in the generated directories with the expected directories.

### Test Fixtures

Located in `fixtures/`:

**Basic Functionality:**
- **simple_node** - Basic node with both publishers and subscribers
- **publishers_only** - Node with only publishers
- **subscribers_only** - Node with only subscribers
- **empty_node** - Node with no publishers or subscribers
- **manually_created** - Tests the `manually_created: true` flag
- **complex_types** - Various ROS message types (geometry_msgs, sensor_msgs, nav_msgs)

**QoS Configuration:**
- **qos_predefined** - Tests predefined QoS profiles (SensorDataQoS, SystemDefaultsQoS, etc.)
- **qos_custom** - Tests custom QoS parameters and profile overrides
- **qos_backward_compat** - Tests backward compatibility with integer QoS values

## Adding New Test Cases

1. Create a new directory in `fixtures/`:
   ```bash
   mkdir fixtures/my_test_case
   ```

2. Create `input.yaml`:
   ```yaml
   node:
       name: ${THIS_NODE}
       package: ${THIS_PACKAGE}

   publishers:
       - topic: my_topic
         type: std_msgs/msg/String
         qos: 10
   ```

3. Create the expected output directories:
   ```bash
   mkdir -p fixtures/my_test_case/expected_cpp
   # Optional: for Python tests
   mkdir -p fixtures/my_test_case/expected_python
   ```

4. Generate outputs:
   ```bash
   # For C++
   python3 ../scripts/generate_node_interface.py \
       fixtures/my_test_case/input.yaml \
       --language cpp \
       --package test_package \
       --node-name my_test_case \
       --output fixtures/my_test_case/generated_cpp

   # For Python (optional)
   python3 ../scripts/generate_node_interface.py \
       fixtures/my_test_case/input.yaml \
       --language python \
       --package test_package \
       --node-name my_test_case \
       --output fixtures/my_test_case/generated_python
   ```

5. Accept the generated outputs:
   ```bash
   ./accept_outputs.sh
   ```

6. Review the expected files and commit them

7. Run tests to verify:
   ```bash
   ./run_tests.sh
   ```

Note: Tests will create `generated_cpp/` and `generated_python/` directories during execution, which are gitignored and can be used for debugging.

## Accepting Test Outputs

When you make intentional changes to the code generator and need to update all expected outputs:

```bash
cd cake/tests
./run_tests.sh                # Generate new outputs
./accept_outputs.sh           # Copy generated_* dirs to expected_* dirs
./run_tests.sh                # Verify all tests pass
```

The `accept_outputs.sh` script copies all files from `generated_cpp/` and `generated_python/` directories to their corresponding `expected_cpp/` and `expected_python/` directories across all test fixtures. Use this after making generator changes that affect output formatting or functionality.

## Dependencies

- pytest
- pyyaml (python3-yaml)

Install with:
```bash
pip install pytest pyyaml
```

## Notes

- These tests are **not** integrated into the colcon build system
- They are meant to be run manually during development
- Tests run the generator script as a subprocess (testing actual usage)
- Generated directories (`generated_cpp/` and `generated_python/`) are created during tests and gitignored
- Whitespace differences are normalized during comparison
- Tests verify both valid inputs and error cases
- C++ tests now verify both `.hpp` and `.params.yaml` files
- For debugging, you can inspect the generated files in `generated_cpp/` and `generated_python/` directories after running tests
