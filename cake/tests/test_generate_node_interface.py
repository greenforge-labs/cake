#!/usr/bin/env python3

"""
Tests for the cake node interface code generator.
"""

import subprocess
from pathlib import Path

import pytest


FIXTURES_DIR = Path(__file__).parent / "fixtures"
SCRIPT_PATH = Path(__file__).parent.parent / "scripts" / "generate_node_interface.py"


def normalize_whitespace(content: str) -> str:
    """Normalize whitespace for comparison."""
    # Split into lines, strip trailing whitespace, remove empty lines at start/end
    lines = [line.rstrip() for line in content.split('\n')]
    # Remove trailing empty lines
    while lines and not lines[-1]:
        lines.pop()
    # Remove leading empty lines
    while lines and not lines[0]:
        lines.pop(0)
    return '\n'.join(lines)


def get_test_cases():
    """Discover all test fixtures."""
    test_cases = []
    for fixture_dir in sorted(FIXTURES_DIR.iterdir()):
        if fixture_dir.is_dir():
            input_file = fixture_dir / "input.yaml"
            expected_file = fixture_dir / "expected_output.hpp"
            if input_file.exists() and expected_file.exists():
                generated_file = fixture_dir / "generated_output.hpp"
                test_cases.append((fixture_dir.name, input_file, expected_file, generated_file))
    return test_cases


@pytest.mark.parametrize("test_name,input_file,expected_file,generated_file", get_test_cases())
def test_generate_node_interface(test_name, input_file, expected_file, generated_file):
    """Test code generation for each fixture."""
    # Run the generator script
    result = subprocess.run(
        ["python3", str(SCRIPT_PATH), str(generated_file), str(input_file), "--package", "test_package"],
        capture_output=True,
        text=True
    )

    # Check that script ran successfully
    assert result.returncode == 0, f"Generator script failed for {test_name}:\n{result.stderr}"

    # Read expected output
    with open(expected_file, 'r') as f:
        expected_output = f.read()

    # Read generated output
    with open(generated_file, 'r') as f:
        generated_output = f.read()

    # Normalize whitespace for comparison
    expected_normalized = normalize_whitespace(expected_output)
    generated_normalized = normalize_whitespace(generated_output)

    # Compare
    if expected_normalized != generated_normalized:
        # Print diff for debugging
        print(f"\n{'='*60}")
        print(f"Test: {test_name}")
        print(f"{'='*60}")
        print("EXPECTED:")
        print(expected_normalized)
        print(f"\n{'-'*60}\n")
        print("GENERATED:")
        print(generated_normalized)
        print(f"{'='*60}\n")

    assert expected_normalized == generated_normalized, f"Generated output differs for {test_name}"


def test_missing_node_name(tmp_path):
    """Test that missing node.name causes script to fail."""
    # Create a YAML file with missing node.name
    yaml_file = tmp_path / "invalid.yaml"
    yaml_file.write_text("""node:
    package: ${THIS_PACKAGE}
""")
    output_file = tmp_path / "output.hpp"

    # Run the generator script
    result = subprocess.run(
        ["python3", str(SCRIPT_PATH), str(output_file), str(yaml_file), "--package", "test_package"],
        capture_output=True,
        text=True
    )

    # Should fail with non-zero exit code
    assert result.returncode != 0, "Script should fail for missing node.name"
    assert "node.name" in result.stderr or "KeyError" in result.stderr


def test_missing_node_section(tmp_path):
    """Test that missing node section causes script to fail."""
    # Create a YAML file with missing node section
    yaml_file = tmp_path / "invalid.yaml"
    yaml_file.write_text("""publishers: []
""")
    output_file = tmp_path / "output.hpp"

    # Run the generator script
    result = subprocess.run(
        ["python3", str(SCRIPT_PATH), str(output_file), str(yaml_file), "--package", "test_package"],
        capture_output=True,
        text=True
    )

    # Should fail with non-zero exit code
    assert result.returncode != 0, "Script should fail for missing node section"
    assert "node" in result.stderr or "KeyError" in result.stderr


def test_empty_publishers_and_subscribers(tmp_path):
    """Test that empty lists are handled correctly."""
    # Create a YAML file with empty publishers and subscribers
    yaml_file = tmp_path / "empty.yaml"
    yaml_file.write_text("""node:
    name: test_node
    package: ${THIS_PACKAGE}
publishers: []
subscribers: []
""")
    output_file = tmp_path / "output.hpp"

    # Run the generator script
    result = subprocess.run(
        ["python3", str(SCRIPT_PATH), str(output_file), str(yaml_file), "--package", "test_package"],
        capture_output=True,
        text=True
    )

    # Should succeed
    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Read generated output
    with open(output_file, 'r') as f:
        output = f.read()

    # Should generate empty structs
    assert "struct TestNodePublishers {}" in output
    assert "struct TestNodeSubscribers {}" in output


def test_this_package_without_package_arg(tmp_path):
    """Test that ${THIS_PACKAGE} without --package argument causes script to fail."""
    # Create a YAML file using ${THIS_PACKAGE}
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text("""node:
    name: test_node
    package: ${THIS_PACKAGE}
publishers: []
""")
    output_file = tmp_path / "output.hpp"

    # Run the generator script WITHOUT --package argument
    result = subprocess.run(
        ["python3", str(SCRIPT_PATH), str(output_file), str(yaml_file)],
        capture_output=True,
        text=True
    )

    # Should fail with non-zero exit code
    assert result.returncode != 0, "Script should fail when ${THIS_PACKAGE} is used without --package argument"
    assert "${THIS_PACKAGE}" in result.stderr or "package" in result.stderr.lower()


def test_this_node_substitution(tmp_path):
    """Test that ${THIS_NODE} is substituted correctly when --node-name is provided."""
    # Create a YAML file using ${THIS_NODE}
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text("""node:
    name: ${THIS_NODE}
    package: test_package
publishers: []
subscribers: []
""")
    output_file = tmp_path / "output.hpp"

    # Run the generator script WITH --node-name argument
    result = subprocess.run(
        ["python3", str(SCRIPT_PATH), str(output_file), str(yaml_file), "--package", "test_package", "--node-name", "my_test_node"],
        capture_output=True,
        text=True
    )

    # Should succeed
    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Read generated output
    with open(output_file, 'r') as f:
        output = f.read()

    # Check that node name was substituted correctly
    assert "namespace test_package::my_test_node" in output
    assert "class MyTestNode" in output


def test_this_node_without_node_name_arg(tmp_path):
    """Test that ${THIS_NODE} without --node-name argument causes script to fail."""
    # Create a YAML file using ${THIS_NODE}
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text("""node:
    name: ${THIS_NODE}
    package: test_package
publishers: []
""")
    output_file = tmp_path / "output.hpp"

    # Run the generator script WITHOUT --node-name argument
    result = subprocess.run(
        ["python3", str(SCRIPT_PATH), str(output_file), str(yaml_file), "--package", "test_package"],
        capture_output=True,
        text=True
    )

    # Should fail with non-zero exit code
    assert result.returncode != 0, "Script should fail when ${THIS_NODE} is used without --node-name argument"
    assert "${THIS_NODE}" in result.stderr or "node" in result.stderr.lower()


def test_parameters_generation(tmp_path):
    """Test that parameters section in interface.yaml generates a .params.yaml file."""
    # Create a YAML file with parameters section
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text("""node:
    name: test_node
    package: test_package

parameters:
  update_rate:
    type: double
    default_value: 10.0
    description: "Update rate"
  robot_name:
    type: string
    default_value: "robot1"
    description: "Robot name"

publishers: []
""")
    output_file = tmp_path / "output.hpp"

    # Run the generator script
    result = subprocess.run(
        ["python3", str(SCRIPT_PATH), str(output_file), str(yaml_file), "--package", "test_package"],
        capture_output=True,
        text=True
    )

    # Should succeed
    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Check that parameters file was generated
    params_file = tmp_path / "output.params.yaml"
    assert params_file.exists(), "Parameters file was not generated"

    # Read and verify parameters file content
    with open(params_file, 'r') as f:
        params_content = f.read()

    # Should have correct namespace
    assert "test_package::test_node:" in params_content
    # Should have the parameters
    assert "update_rate:" in params_content
    assert "type: double" in params_content
    assert "robot_name:" in params_content
    assert "type: string" in params_content


def test_no_parameters_generates_dummy(tmp_path):
    """Test that a .params.yaml file with dummy parameter is always generated, even without parameters section."""
    # Create a YAML file WITHOUT parameters section
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text("""node:
    name: test_node
    package: test_package

publishers:
    - topic: /status
      type: std_msgs/msg/String
      qos: 10
""")
    output_file = tmp_path / "output.hpp"

    # Run the generator script
    result = subprocess.run(
        ["python3", str(SCRIPT_PATH), str(output_file), str(yaml_file), "--package", "test_package"],
        capture_output=True,
        text=True
    )

    # Should succeed
    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Check that parameters file WAS generated (always, even with no parameters)
    params_file = tmp_path / "output.params.yaml"
    assert params_file.exists(), "Parameters file should always be generated"

    # Read and verify it has dummy parameter
    with open(params_file, 'r') as f:
        params_content = f.read()

    # Should have namespace and dummy parameter
    assert "test_package::test_node:" in params_content
    assert "__cake_dummy:" in params_content


if __name__ == "__main__":
    # Allow running directly with python
    pytest.main([__file__, "-v"])
