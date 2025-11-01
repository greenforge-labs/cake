#!/bin/bash

# Script to accept all test outputs by copying generated_output.hpp to expected_output.hpp
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

count=0
for fixture_dir in "$FIXTURES_DIR"/*/; do
    if [ -d "$fixture_dir" ]; then
        fixture_name=$(basename "$fixture_dir")
        generated_file="$fixture_dir/generated_output.hpp"
        expected_file="$fixture_dir/expected_output.hpp"

        if [ -f "$generated_file" ]; then
            cp "$generated_file" "$expected_file"
            echo "✓ Accepted: $fixture_name"
            count=$((count + 1))
        else
            echo "⚠ Skipped: $fixture_name (no generated_output.hpp found)"
        fi
    fi
done

echo "================================"
echo "Accepted $count fixture output(s)"
echo ""
echo "Note: Run ./run_tests.sh to verify the changes"
