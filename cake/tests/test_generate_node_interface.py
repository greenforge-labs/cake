#!/usr/bin/env python3

"""
Tests for the cake node interface code generator.
"""

from pathlib import Path
import subprocess

import pytest

FIXTURES_DIR = Path(__file__).parent / "fixtures"
SCRIPT_PATH = Path(__file__).parent.parent / "scripts" / "generate_node_interface.py"


def normalize_whitespace(content: str) -> str:
    """Normalize whitespace for comparison."""
    # Split into lines, strip trailing whitespace, remove empty lines at start/end
    lines = [line.rstrip() for line in content.split("\n")]
    # Remove trailing empty lines
    while lines and not lines[-1]:
        lines.pop()
    # Remove leading empty lines
    while lines and not lines[0]:
        lines.pop(0)
    return "\n".join(lines)


def get_test_cases():
    """Discover all test fixtures."""
    test_cases = []
    for fixture_dir in sorted(FIXTURES_DIR.iterdir()):
        if fixture_dir.is_dir():
            input_file = fixture_dir / "input.yaml"
            expected_dir = fixture_dir / "expected_cpp"
            if input_file.exists() and expected_dir.exists():
                generated_dir = fixture_dir / "generated_cpp"
                test_cases.append((fixture_dir.name, input_file, expected_dir, generated_dir))
    return test_cases


@pytest.mark.parametrize("test_name,input_file,expected_dir,generated_dir", get_test_cases())
def test_generate_node_interface(test_name, input_file, expected_dir, generated_dir):
    """Test code generation for each fixture."""
    # Clean and create generated directory
    if generated_dir.exists():
        import shutil

        shutil.rmtree(generated_dir)
    generated_dir.mkdir(parents=True)

    # Run the generator script with new unified argument structure
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(input_file),
            "--language",
            "cpp",
            "--package",
            "test_package",
            "--node-name",
            test_name,
            "--output",
            str(generated_dir),
        ],
        capture_output=True,
        text=True,
    )

    # Check that script ran successfully
    assert result.returncode == 0, f"Generator script failed for {test_name}:\n{result.stderr}"

    # Compare all files in expected vs generated directories
    expected_files = sorted(expected_dir.glob("*"))
    generated_files = sorted(generated_dir.glob("*"))

    # Check same number of files
    assert len(expected_files) == len(generated_files), (
        f"File count mismatch for {test_name}: " f"expected {len(expected_files)} files, got {len(generated_files)}"
    )

    # Compare each file
    for exp_file, gen_file in zip(expected_files, generated_files):
        assert exp_file.name == gen_file.name, f"Filename mismatch for {test_name}: {exp_file.name} != {gen_file.name}"

        with open(exp_file, "r") as f:
            expected_content = f.read()
        with open(gen_file, "r") as f:
            generated_content = f.read()

        # Normalize whitespace for comparison
        expected_normalized = normalize_whitespace(expected_content)
        generated_normalized = normalize_whitespace(generated_content)

        if expected_normalized != generated_normalized:
            # Print diff for debugging
            print(f"\n{'='*60}")
            print(f"Test: {test_name} - File: {exp_file.name}")
            print(f"{'='*60}")
            print("EXPECTED:")
            print(expected_normalized)
            print(f"\n{'-'*60}\n")
            print("GENERATED:")
            print(generated_normalized)
            print(f"{'='*60}\n")

        assert expected_normalized == generated_normalized, f"Content differs for {test_name} in file {exp_file.name}"


def test_missing_node_name(tmp_path):
    """Test that missing node.name causes script to fail."""
    # Create a YAML file with missing node.name
    yaml_file = tmp_path / "invalid.yaml"
    yaml_file.write_text(
        """node:
    package: ${THIS_PACKAGE}
"""
    )

    # Run the generator script
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "cpp",
            "--package",
            "test_package",
            "--node-name",
            "test_node",
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )

    # Should fail with non-zero exit code
    assert result.returncode != 0, "Script should fail for missing node.name"
    assert "node.name" in result.stderr or "KeyError" in result.stderr


def test_missing_node_section(tmp_path):
    """Test that missing node section causes script to fail."""
    # Create a YAML file with missing node section
    yaml_file = tmp_path / "invalid.yaml"
    yaml_file.write_text(
        """publishers: []
"""
    )

    # Run the generator script
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "cpp",
            "--package",
            "test_package",
            "--node-name",
            "test_node",
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )

    # Should fail with non-zero exit code
    assert result.returncode != 0, "Script should fail for missing node section"
    assert "node" in result.stderr or "KeyError" in result.stderr


def test_empty_publishers_and_subscribers(tmp_path):
    """Test that empty lists are handled correctly."""
    # Create a YAML file with empty publishers and subscribers
    yaml_file = tmp_path / "empty.yaml"
    yaml_file.write_text(
        """node:
    name: test_node
    package: ${THIS_PACKAGE}
publishers: []
subscribers: []
"""
    )

    # Run the generator script
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "cpp",
            "--package",
            "test_package",
            "--node-name",
            "test_node",
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )

    # Should succeed
    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Read generated output
    output_file = tmp_path / "test_node_interface.hpp"
    with open(output_file, "r") as f:
        output = f.read()

    # Should generate empty structs
    assert "struct TestNodePublishers {}" in output
    assert "struct TestNodeSubscribers {}" in output


def test_this_package_without_package_arg(tmp_path):
    """Test that ${THIS_PACKAGE} without --package argument causes script to fail."""
    # Create a YAML file using ${THIS_PACKAGE}
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
    name: test_node
    package: ${THIS_PACKAGE}
publishers: []
"""
    )

    # Run the generator script WITHOUT --package argument
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "cpp",
            "--node-name",
            "test_node",
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )

    # Should fail with non-zero exit code
    assert result.returncode != 0, "Script should fail when ${THIS_PACKAGE} is used without --package argument"
    assert "${THIS_PACKAGE}" in result.stderr or "package" in result.stderr.lower()


def test_this_node_substitution(tmp_path):
    """Test that ${THIS_NODE} is substituted correctly when --node-name is provided."""
    # Create a YAML file using ${THIS_NODE}
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
    name: ${THIS_NODE}
    package: test_package
publishers: []
subscribers: []
"""
    )

    # Run the generator script WITH --node-name argument
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "cpp",
            "--package",
            "test_package",
            "--node-name",
            "my_test_node",
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )

    # Should succeed
    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Read generated output
    output_file = tmp_path / "my_test_node_interface.hpp"
    with open(output_file, "r") as f:
        output = f.read()

    # Check that node name was substituted correctly
    assert "namespace test_package::my_test_node" in output
    assert "class MyTestNode" in output


def test_this_node_without_node_name_arg(tmp_path):
    """Test that ${THIS_NODE} without --node-name argument causes script to fail."""
    # Create a YAML file using ${THIS_NODE}
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
    name: ${THIS_NODE}
    package: test_package
publishers: []
"""
    )

    # Run the generator script WITH --node-name missing
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "cpp",
            "--package",
            "test_package",
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )

    # Should fail with non-zero exit code (required argument missing)
    assert result.returncode != 0, "Script should fail when --node-name is not provided"
    assert "node-name" in result.stderr.lower() or "required" in result.stderr.lower()


def test_parameters_generation(tmp_path):
    """Test that parameters section in interface.yaml generates a .params.yaml file."""
    # Create a YAML file with parameters section
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
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
"""
    )

    # Run the generator script
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "cpp",
            "--package",
            "test_package",
            "--node-name",
            "test_node",
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )

    # Should succeed
    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Check that parameters file was generated
    params_file = tmp_path / "test_node_interface.params.yaml"
    assert params_file.exists(), "Parameters file was not generated"

    # Read and verify parameters file content
    with open(params_file, "r") as f:
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
    yaml_file.write_text(
        """node:
    name: test_node
    package: test_package

publishers:
    - topic: /status
      type: std_msgs/msg/String
      qos: 10
"""
    )

    # Run the generator script
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "cpp",
            "--package",
            "test_package",
            "--node-name",
            "test_node",
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )

    # Should succeed
    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Check that parameters file WAS generated (always, even with no parameters)
    params_file = tmp_path / "test_node_interface.params.yaml"
    assert params_file.exists(), "Parameters file should always be generated"

    # Read and verify it has dummy parameter
    with open(params_file, "r") as f:
        params_content = f.read()

    # Should have namespace and dummy parameter
    assert "test_package::test_node:" in params_content
    assert "__cake_dummy:" in params_content


# ============================================================================
# Python code generation tests
# ============================================================================


def get_python_test_cases():
    """Discover all test fixtures with Python expected outputs."""
    test_cases = []
    for fixture_dir in sorted(FIXTURES_DIR.iterdir()):
        if fixture_dir.is_dir():
            input_file = fixture_dir / "input.yaml"
            expected_dir = fixture_dir / "expected_python"

            # Only include if expected_python directory exists
            if input_file.exists() and expected_dir.exists():
                generated_dir = fixture_dir / "generated_python"
                test_cases.append((fixture_dir.name, input_file, expected_dir, generated_dir))
    return test_cases


@pytest.mark.parametrize("test_name,input_file,expected_dir,generated_dir", get_python_test_cases())
def test_generate_python_interface(test_name, input_file, expected_dir, generated_dir):
    """Test Python code generation for each fixture."""
    # Clean and create generated directory
    if generated_dir.exists():
        import shutil

        shutil.rmtree(generated_dir)

    # Run the generator script with Python language
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(input_file),
            "--language",
            "python",
            "--package",
            "test_package",
            "--node-name",
            test_name,
            "--output",
            str(generated_dir),
        ],
        capture_output=True,
        text=True,
    )

    # Check that script ran successfully
    assert result.returncode == 0, f"Generator script failed for {test_name}:\n{result.stderr}"

    # Compare all files in expected vs generated directories
    expected_files = sorted(expected_dir.glob("*.py"))
    generated_files = sorted(generated_dir.glob("*.py"))

    # Check same number of files
    assert len(expected_files) == len(generated_files), (
        f"File count mismatch for {test_name}: " f"expected {len(expected_files)} files, got {len(generated_files)}"
    )

    # Compare each file
    for exp_file, gen_file in zip(expected_files, generated_files):
        assert exp_file.name == gen_file.name, f"Filename mismatch for {test_name}: {exp_file.name} != {gen_file.name}"

        with open(exp_file, "r") as f:
            expected_content = f.read()
        with open(gen_file, "r") as f:
            generated_content = f.read()

        # Normalize whitespace for comparison
        expected_normalized = normalize_whitespace(expected_content)
        generated_normalized = normalize_whitespace(generated_content)

        if expected_normalized != generated_normalized:
            # Print diff for debugging
            print(f"\n{'='*60}")
            print(f"Test: {test_name} - File: {exp_file.name}")
            print(f"{'='*60}")
            print("EXPECTED:")
            # Limit output for parameters files which can be long
            max_len = 1000 if "_parameters" in exp_file.name else None
            print(expected_normalized[:max_len])
            print(f"\n{'-'*60}\n")
            print("GENERATED:")
            print(generated_normalized[:max_len])
            print(f"{'='*60}\n")

        assert expected_normalized == generated_normalized, f"Content differs for {test_name} in file {exp_file.name}"


def test_python_syntax_validation(tmp_path):
    """Test that generated Python code is syntactically valid."""
    import py_compile

    # Create a simple YAML file
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
    name: test_node
    package: test_package

publishers:
    - topic: /status
      type: std_msgs/msg/String
      qos: 10

subscribers:
    - topic: /input
      type: std_msgs/msg/Bool
      qos: 5
"""
    )
    output_dir = tmp_path / "output"

    # Run the generator script
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "python",
            "--package",
            "test_package",
            "--node-name",
            "test_node",
            "--output",
            str(output_dir),
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Verify all generated files compile
    for py_file in ["_interface.py", "_parameters.py", "__init__.py"]:
        file_path = output_dir / py_file
        assert file_path.exists(), f"{py_file} not generated"
        try:
            py_compile.compile(str(file_path), doraise=True)
        except py_compile.PyCompileError as e:
            pytest.fail(f"{py_file} has syntax error: {e}")


def test_python_missing_output_dir(tmp_path):
    """Test that Python generation fails without --output."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
    name: test_node
    package: test_package
publishers: []
"""
    )

    # Run WITHOUT --output
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "python",
            "--package",
            "test_package",
            "--node-name",
            "test_node",
        ],
        capture_output=True,
        text=True,
    )

    # Should fail
    assert result.returncode != 0, "Script should fail without --output for Python"
    assert "output" in result.stderr.lower(), "Error message should mention output"


def test_python_parameters_static_namespace(tmp_path):
    """Test that Python parameters use static 'parameters' namespace."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
    name: my_node
    package: my_package

parameters:
    rate:
        type: double
        default_value: 10.0
        description: "Rate"

publishers: []
"""
    )
    output_dir = tmp_path / "output"

    # Run the generator
    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "python",
            "--package",
            "my_package",
            "--node-name",
            "my_node",
            "--output",
            str(output_dir),
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    # Check _parameters.py uses static namespace
    params_file = output_dir / "_parameters.py"
    with open(params_file, "r") as f:
        params_content = f.read()

    assert "class parameters:" in params_content, "Parameters should use 'parameters' namespace"
    assert "my_package::my_node" not in params_content, "Should NOT use package::node namespace"

    # Check _interface.py imports from static namespace
    interface_file = output_dir / "_interface.py"
    with open(interface_file, "r") as f:
        interface_content = f.read()

    assert (
        "from my_package.my_node._parameters import parameters" in interface_content
    ), "Should import from static namespace"


def test_python_publisher_import_conditional(tmp_path):
    """Test that Publisher import is only included when there are publishers."""
    # Test 1: With publishers
    yaml_with_pubs = tmp_path / "with_pubs.yaml"
    yaml_with_pubs.write_text(
        """node:
    name: pub_node
    package: test_package

publishers:
    - topic: /status
      type: std_msgs/msg/String
      qos: 10
"""
    )
    output_with_pubs = tmp_path / "output_with_pubs"

    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_with_pubs),
            "--language",
            "python",
            "--package",
            "test_package",
            "--node-name",
            "pub_node",
            "--output",
            str(output_with_pubs),
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    with open(output_with_pubs / "_interface.py", "r") as f:
        content = f.read()
    assert "from rclpy.publisher import Publisher" in content, "Should import Publisher when publishers exist"

    # Test 2: Without publishers
    yaml_no_pubs = tmp_path / "no_pubs.yaml"
    yaml_no_pubs.write_text(
        """node:
    name: sub_node
    package: test_package

subscribers:
    - topic: /input
      type: std_msgs/msg/Bool
      qos: 5
"""
    )
    output_no_pubs = tmp_path / "output_no_pubs"

    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_no_pubs),
            "--language",
            "python",
            "--package",
            "test_package",
            "--node-name",
            "sub_node",
            "--output",
            str(output_no_pubs),
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    with open(output_no_pubs / "_interface.py", "r") as f:
        content = f.read()
    assert "from rclpy.publisher import Publisher" not in content, "Should NOT import Publisher when no publishers"


def test_python_qos_imports(tmp_path):
    """Test that QoS imports match usage."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
    name: test_node
    package: test_package

publishers:
    - topic: /status
      type: std_msgs/msg/String
      qos:
        profile: SystemDefaultsQoS

subscribers:
    - topic: /sensor
      type: std_msgs/msg/Bool
      qos:
        profile: SensorDataQoS
"""
    )
    output_dir = tmp_path / "output"

    result = subprocess.run(
        [
            "python3",
            str(SCRIPT_PATH),
            str(yaml_file),
            "--language",
            "python",
            "--package",
            "test_package",
            "--node-name",
            "test_node",
            "--output",
            str(output_dir),
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    with open(output_dir / "_interface.py", "r") as f:
        content = f.read()

    # Check imports
    assert "qos_profile_system_default" in content, "Should import qos_profile_system_default"
    assert "qos_profile_sensor_data" in content, "Should import qos_profile_sensor_data"

    # Check usage
    assert "qos_profile_system_default" in content.split("def run")[1], "Should use qos_profile_system_default"
    assert "qos_profile_sensor_data" in content.split("def run")[1], "Should use qos_profile_sensor_data"


if __name__ == "__main__":
    # Allow running directly with python
    pytest.main([__file__, "-v"])
