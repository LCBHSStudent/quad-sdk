#!/bin/bash

command -v clang-format >/dev/null 2>&1 || { 
    echo "Error: clang-format is not installed" 
    echo "Run: sudo apt-get install clang-format"
    exit 1 
}

for dir in include src; do
    if [[ -d "$dir" ]]; then
        echo "Formatting $dir/..."
        find "$dir" -type f \( -name "*.cpp" -o -name "*.cc" -o -name "*.h" -o -name "*.hpp" \) \
            -exec echo "  {}" \; -exec clang-format -i {} \;
    fi
done

echo "Formatting completed!"
