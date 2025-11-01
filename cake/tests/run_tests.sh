#!/bin/bash

# Convenience script to run the cake code generator tests

cd "$(dirname "$0")"

echo "Running cake code generator tests..."
echo ""

pytest -v test_generate_node_interface.py "$@"
