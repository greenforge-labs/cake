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
    assert "'name' is a required property" in result.stderr


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
      qos:
        history: 10
        reliability: RELIABLE
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

    # Compare all files in expected vs generated directories (including .py and .yaml files)
    expected_files = sorted(expected_dir.glob("*"))
    expected_files = [f for f in expected_files if f.is_file()]
    generated_files = sorted(generated_dir.glob("*"))
    generated_files = [f for f in generated_files if f.is_file()]

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
      qos:
        history: 10
        reliability: RELIABLE

subscribers:
    - topic: /input
      type: std_msgs/msg/Bool
      qos:
        history: 5
        reliability: BEST_EFFORT
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
    for py_file in ["interface.py", "_parameters.py", "parameters.py", "__init__.py"]:
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

    # Check interface.py uses relative imports
    interface_file = output_dir / "interface.py"
    with open(interface_file, "r") as f:
        interface_content = f.read()

    assert (
        "from .parameters import Params, ParamListener" in interface_content
    ), "Should use relative import from parameters wrapper"


def test_python_publisher_usage_conditional(tmp_path):
    """Test that cake.Publisher is only used when there are publishers."""
    # Test 1: With publishers
    yaml_with_pubs = tmp_path / "with_pubs.yaml"
    yaml_with_pubs.write_text(
        """node:
    name: pub_node
    package: test_package

publishers:
    - topic: /status
      type: std_msgs/msg/String
      qos:
        history: 10
        reliability: RELIABLE
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
    with open(output_with_pubs / "interface.py", "r") as f:
        content = f.read()
    assert "cake.Publisher[" in content, "Should use cake.Publisher when publishers exist"

    # Test 2: Without publishers
    yaml_no_pubs = tmp_path / "no_pubs.yaml"
    yaml_no_pubs.write_text(
        """node:
    name: sub_node
    package: test_package

subscribers:
    - topic: /input
      type: std_msgs/msg/Bool
      qos:
        history: 5
        reliability: BEST_EFFORT
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
    with open(output_no_pubs / "interface.py", "r") as f:
        content = f.read()
    assert "cake.Publisher[" not in content, "Should NOT use cake.Publisher when no publishers"


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
        history: 10
        reliability: RELIABLE
        durability: TRANSIENT_LOCAL

subscribers:
    - topic: /sensor
      type: std_msgs/msg/Bool
      qos:
        history: 5
        reliability: BEST_EFFORT
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
    with open(output_dir / "interface.py", "r") as f:
        content = f.read()

    # Check imports for new QoS format
    assert "QoSProfile" in content, "Should import QoSProfile"
    assert "HistoryPolicy" in content, "Should import HistoryPolicy"
    assert "ReliabilityPolicy" in content, "Should import ReliabilityPolicy"
    assert "DurabilityPolicy" in content, "Should import DurabilityPolicy"

    # Check usage
    assert "QoSProfile(" in content, "Should use QoSProfile"


def test_qos_invalid_reliability_value(tmp_path):
    """Test that invalid QoS reliability value produces clear error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: /test
    type: std_msgs/msg/String
    qos:
      history: 10
      reliability: invalid_value
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0 -> qos -> reliability]" in result.stderr
    assert "invalid_value" in result.stderr


def test_qos_unknown_parameter(tmp_path):
    """Test that unknown QoS parameter (typo) produces helpful error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: /test
    type: std_msgs/msg/String
    qos:
      history: 10
      reliability: RELIABLE
      durabilty: VOLATILE
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0 -> qos]" in result.stderr
    assert "durabilty" in result.stderr  # The typo is shown in the error


def test_qos_zero_history(tmp_path):
    """Test that zero history depth produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: /test
    type: std_msgs/msg/String
    qos:
      history: 0
      reliability: RELIABLE
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0 -> qos -> history]" in result.stderr
    assert "0" in result.stderr


def test_qos_invalid_durability_value(tmp_path):
    """Test that invalid QoS durability value produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: /test
    type: std_msgs/msg/String
    qos:
      history: 10
      reliability: RELIABLE
      durability: INVALID_VALUE
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0 -> qos -> durability]" in result.stderr
    assert "INVALID_VALUE" in result.stderr


def test_ros_type_missing_slash(tmp_path):
    """Test that malformed ROS type (missing slash) produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: /test
    type: std_msgsmsgString
    qos:
      history: 10
      reliability: RELIABLE
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0 -> type]" in result.stderr
    assert "std_msgsmsgString" in result.stderr
    assert "does not match" in result.stderr


def test_ros_type_empty_package(tmp_path):
    """Test that ROS type with empty package name produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: /test
    type: /msg/String
    qos:
      history: 10
      reliability: RELIABLE
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0 -> type]" in result.stderr
    assert "/msg/String" in result.stderr
    assert "does not match" in result.stderr


def test_topic_invalid_characters(tmp_path):
    """Test that topic name with invalid characters produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: /test@topic#
    type: std_msgs/msg/String
    qos:
      history: 10
      reliability: RELIABLE
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0 -> topic]" in result.stderr
    assert "/test@topic#" in result.stderr
    assert "does not match" in result.stderr


def test_topic_valid_without_leading_slash(tmp_path):
    """Test that topic name without leading slash is accepted."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: cmd_vel
    type: geometry_msgs/msg/Twist
    qos:
      history: 10
      reliability: RELIABLE
"""
    )

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

    assert result.returncode == 0, f"Should accept topic without leading slash. stderr: {result.stderr}"


def test_publisher_missing_topic(tmp_path):
    """Test that publisher without topic field produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - type: std_msgs/msg/String
    qos:
      history: 10
      reliability: RELIABLE
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0]" in result.stderr
    assert "'topic' is a required property" in result.stderr


def test_publisher_missing_type(tmp_path):
    """Test that publisher without type field produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: /test
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0]" in result.stderr
    assert "'type' is a required property" in result.stderr


def test_subscriber_missing_fields(tmp_path):
    """Test that subscriber without required fields produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
subscribers:
  - topic: /test
"""
    )

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

    assert result.returncode != 0
    assert "[subscribers -> 0]" in result.stderr
    assert "'type' is a required property" in result.stderr


def test_service_missing_name(tmp_path):
    """Test that service without name field produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
services:
  - type: example_interfaces/srv/AddTwoInts
"""
    )

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

    assert result.returncode != 0
    assert "[services -> 0]" in result.stderr
    assert "'name' is a required property" in result.stderr


def test_service_qos_not_allowed(tmp_path):
    """Test that QoS is not allowed for services."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
services:
  - name: /my_service
    type: std_srvs/srv/Trigger
    qos:
      history: 10
      reliability: RELIABLE
"""
    )

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

    assert result.returncode != 0
    assert "[services -> 0]" in result.stderr
    assert "Additional properties are not allowed" in result.stderr


def test_service_client_qos_not_allowed(tmp_path):
    """Test that QoS is not allowed for service clients."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
service_clients:
  - name: /my_client
    type: std_srvs/srv/Trigger
    qos:
      history: 10
      reliability: RELIABLE
"""
    )

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

    assert result.returncode != 0
    assert "[service_clients -> 0]" in result.stderr
    assert "Additional properties are not allowed" in result.stderr


def test_very_long_topic_name(tmp_path):
    """Test that very long topic name is accepted."""
    long_name = "/" + "a" * 200
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        f"""node:
  name: test_node
  package: ${{THIS_PACKAGE}}
publishers:
  - topic: {long_name}
    type: std_msgs/msg/String
    qos:
      history: 10
      reliability: RELIABLE
"""
    )

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

    assert result.returncode == 0, f"Should accept long topic names. stderr: {result.stderr}"


def test_qos_negative_deadline(tmp_path):
    """Test that negative deadline_ms produces error."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
  name: test_node
  package: ${THIS_PACKAGE}
publishers:
  - topic: /test
    type: std_msgs/msg/String
    qos:
      history: 10
      reliability: RELIABLE
      deadline_ms: -1
"""
    )

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

    assert result.returncode != 0
    assert "[publishers -> 0 -> qos -> deadline_ms]" in result.stderr
    assert "-1" in result.stderr


def test_registration_cpp_generation(tmp_path):
    """Test that registration .cpp file is generated with correct content."""
    yaml_file = tmp_path / "test.yaml"
    yaml_file.write_text(
        """node:
    name: test_node
    package: test_package

publishers:
    - topic: /status
      type: std_msgs/msg/String
      qos:
        history: 10
        reliability: RELIABLE
"""
    )

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

    assert result.returncode == 0, f"Generator script failed:\n{result.stderr}"

    # Check registration file exists
    registration_file = tmp_path / "test_node_registration.cpp"
    assert registration_file.exists(), "Registration file not generated"

    # Verify content
    with open(registration_file, "r") as f:
        content = f.read()

    assert "// auto-generated DO NOT EDIT" in content
    assert '#include "nodes/test_node/test_node.hpp"' in content
    assert "#include <rclcpp_components/register_node_macro.hpp>" in content
    assert "// Type alias to export the component at the package level" in content
    assert "using TestNode = test_package::test_node::TestNode" in content
    assert "RCLCPP_COMPONENTS_REGISTER_NODE(test_package::TestNode)" in content


if __name__ == "__main__":
    # Allow running directly with python
    pytest.main([__file__, "-v"])
