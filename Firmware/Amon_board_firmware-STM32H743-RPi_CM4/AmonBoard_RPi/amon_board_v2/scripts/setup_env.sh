#!/bin/bash
# save as: setup_vscode.sh
# make executable: chmod +x setup_vscode.sh
# run: ./setup_vscode.sh

echo "=== Setting up VS Code for Amon Lander ==="

# Get project root (where this script is)
PROJECT_ROOT="$(pwd)"
echo "Project root: $PROJECT_ROOT"

# 1. Create .vscode directory
echo "1. Creating .vscode directory..."
mkdir -p "${PROJECT_ROOT}/.vscode"

# 2. Create c_cpp_properties.json
echo "2. Creating IntelliSense configuration..."
cat > "${PROJECT_ROOT}/.vscode/c_cpp_properties.json" << 'EOF'
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/include",
                "${workspaceFolder}/modules/gnss/include",
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "gnu++14",
            "intelliSenseMode": "linux-gcc-x64",
            "configurationProvider": "ms-vscode.cmake-tools"
        }
    ],
    "version": 4
}
EOF

# 3. Create settings.json
echo "3. Creating workspace settings..."
cat > "${PROJECT_ROOT}/.vscode/settings.json" << 'EOF'
{
    "cmake.configureOnOpen": true,
    "cmake.buildDirectory": "${workspaceFolder}/build",
    "cmake.generator": "Unix Makefiles",
    "cmake.buildBeforeRun": true,
    
    "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools",
    "C_Cpp.default.compileCommands": "${workspaceFolder}/build/compile_commands.json",
    
    "files.associations": {
        "*.h": "c"
    },
    
    "editor.formatOnSave": true,
    "editor.formatOnType": false,
    
    "cmake.sourceDirectory": "${workspaceFolder}"
}
EOF

# 4. Create launch.json for debugging
echo "4. Creating debug configuration..."
cat > "${PROJECT_ROOT}/.vscode/launch.json" << 'EOF'
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug Amon Lander",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/amon_lander",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb"
        }
    ]
}
EOF

# 5. Create tasks.json for build commands
echo "5. Creating build tasks..."
cat > "${PROJECT_ROOT}/.vscode/tasks.json" << 'EOF'
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Configure CMake",
            "type": "shell",
            "command": "cmake",
            "args": [
                "-B",
                "${workspaceFolder}/build",
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=1",
                "-S",
                "${workspaceFolder}"
            ],
            "group": {
                "kind": "build",
                "isDefault": false
            },
            "problemMatcher": []
        },
        {
            "label": "Build Project",
            "type": "shell",
            "command": "cmake",
            "args": [
                "--build",
                "${workspaceFolder}/build"
            ],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": [
                "$gcc"
            ]
        },
        {
            "label": "Clean Build",
            "type": "shell",
            "command": "rm",
            "args": [
                "-rf",
                "${workspaceFolder}/build"
            ],
            "group": "build"
        }
    ]
}
EOF

# 6. Update root CMakeLists.txt for IntelliSense
echo "6. Updating CMakeLists.txt for IntelliSense..."
# Backup original
if [ -f "${PROJECT_ROOT}/CMakeLists.txt" ]; then
    cp "${PROJECT_ROOT}/CMakeLists.txt" "${PROJECT_ROOT}/CMakeLists.txt.backup"
fi

# Check if the line already exists
if ! grep -q "CMAKE_EXPORT_COMPILE_COMMANDS" "${PROJECT_ROOT}/CMakeLists.txt"; then
    # Add after project() line
    sed -i '/project(AmonLander C)/a set(CMAKE_EXPORT_COMPILE_COMMANDS ON)' "${PROJECT_ROOT}/CMakeLists.txt"
fi

# 7. Generate compile_commands.json
echo "7. Generating compile_commands.json..."
mkdir -p "${PROJECT_ROOT}/build"
cd "${PROJECT_ROOT}/build"
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..

# 8. Create symbolic link for easier access
echo "8. Creating symlink for compile_commands.json..."
ln -sf "${PROJECT_ROOT}/build/compile_commands.json" "${PROJECT_ROOT}/compile_commands.json"

# 9. Check if extensions are installed
echo "9. Checking VS Code extensions..."
echo "Please install these VS Code extensions:"
echo "  - C/C++ (ms-vscode.cpptools)"
echo "  - CMake Tools (ms-vscode.cmake-tools)"
echo ""
echo "To install from terminal:"
echo "  code --install-extension ms-vscode.cpptools"
echo "  code --install-extension ms-vscode.cmake-tools"

# 10. Create a simple test to verify setup
echo "10. Creating test files..."
cat > "${PROJECT_ROOT}/test_intellisense.c" << 'EOF'
// Test file to check IntelliSense
// Try hovering over functions or Ctrl+Click on gnss_init

#include "gnss.h"

void test_function() {
    GnssData data;
    gnss_init();
    gnss_update(&data);
}
EOF

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "To test IntelliSense:"
echo "1. Open VS Code in project folder:"
echo "   code ${PROJECT_ROOT}"
echo "2. Open test_intellisense.c"
echo "3. Hover over 'GnssData' or 'gnss_init' - you should see info"
echo "4. Try Ctrl+Space after 'gnss_' for autocomplete"
echo ""
echo "To build and run:"
echo "1. Press Ctrl+Shift+B to build"
echo "2. Press F5 to debug"
echo ""
echo "If IntelliSense doesn't work:"
echo "1. Press Ctrl+Shift+P"
echo "2. Type 'C/C++: Reset IntelliSense Database'"
echo "3. Reload VS Code: Ctrl+Shift+P → 'Developer: Reload Window'"