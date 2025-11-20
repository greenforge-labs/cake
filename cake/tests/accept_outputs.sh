#!/bin/bash

# Script to accept all test outputs by copying generated_* directories to expected_* directories
# This updates the expected outputs for all test fixtures

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIXTURES_DIR="$SCRIPT_DIR/fixtures"

# Check if fixtures directory exists
if [ ! -d "$FIXTURES_DIR" ]; then
    echo "Error: Fixtures directory not found at $FIXTURES_DIR"
    exit 1
fi

echo "Accepting all test outputs..."
echo "================================"

cpp_count=0
python_count=0

for fixture_dir in "$FIXTURES_DIR"/*/; do
    if [ -d "$fixture_dir" ]; then
        fixture_name=$(basename "$fixture_dir")

        # Accept C++ outputs
        generated_cpp="$fixture_dir/generated_cpp"
        expected_cpp="$fixture_dir/expected_cpp"

        if [ -d "$generated_cpp" ]; then
            # Remove old expected_cpp if it exists
            rm -rf "$expected_cpp"
            # Copy entire directory
            cp -r "$generated_cpp" "$expected_cpp"
            echo "✓ Accepted C++: $fixture_name"
            cpp_count=$((cpp_count + 1))
        fi

        # Accept Python outputs
        generated_python="$fixture_dir/generated_python"
        expected_python="$fixture_dir/expected_python"

        if [ -d "$generated_python" ]; then
            # Remove old expected_python if it exists
            rm -rf "$expected_python"
            # Copy entire directory
            cp -r "$generated_python" "$expected_python"
            echo "✓ Accepted Python: $fixture_name"
            python_count=$((python_count + 1))
        fi
    fi
done

echo "================================"
echo "Accepted $cpp_count C++ fixture(s)"
echo "Accepted $python_count Python fixture(s)"
echo ""
echo "Note: Run ./run_tests.sh to verify the changes"
