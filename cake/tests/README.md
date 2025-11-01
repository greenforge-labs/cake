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
- `expected_output.hpp` - The expected generated code (committed)
- `generated_output.hpp` - Temporary file created during test run (gitignored)

Tests run the actual generator script as a subprocess and compare the generated output file with the expected output file.

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
       name: my_node
       package: $THIS_PACKAGE

   publishers:
       - topic: my_topic
         type: std_msgs/msg/String
         qos: 10
   ```

3. Generate the expected output:
   ```bash
   python3 ../scripts/generate_node_interface.py \
       fixtures/my_test_case/expected_output.hpp \
       fixtures/my_test_case/input.yaml \
       --package test_package
   ```

4. Review the generated `expected_output.hpp` file and commit it

5. Run tests to verify:
   ```bash
   ./run_tests.sh
   ```

Note: Tests will create `generated_output.hpp` files during execution, which are gitignored and can be used for debugging.

## Accepting Test Outputs

When you make intentional changes to the code generator and need to update all expected outputs:

```bash
cd cake/tests
./run_tests.sh                # Generate new outputs
./accept_outputs.sh           # Copy generated outputs to expected outputs
./run_tests.sh                # Verify all tests pass
```

The `accept_outputs.sh` script copies all `generated_output.hpp` files to their corresponding `expected_output.hpp` files across all test fixtures. Use this after making generator changes that affect output formatting or functionality.

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
- Generated output files (`generated_output.hpp`) are created during tests and gitignored
- Whitespace differences are normalized during comparison
- Tests verify both valid inputs and error cases
- For debugging, you can inspect the generated files in each fixture directory after running tests
