#!/usr/bin/env python3
"""
Extract BuildRequires from ROS-related SRPM packages
Generate buildrequire_list.json for dependency verification

This script supports all ROS distributions (humble, jazzy, rolling, etc.)
and can process any ROS-related SRPM packages.
"""

import os
import re
import json
import subprocess
import tempfile
import argparse
from pathlib import Path
from collections import defaultdict

def extract_package_name_from_srpm(srpm_filename):
    """
    Extract package name from SRPM filename
    Example: ros-humble-turtlesim-1.4.2-1.oe2403.src.rpm -> turtlesim
    Example: ros-jazzy-nav2-1.2.0-1.oe2403.src.rpm -> nav2
    """
    # Remove .src.rpm suffix
    name = srpm_filename.replace('.src.rpm', '')

    # Remove ros-{distro}- prefix if present (e.g., ros-humble-, ros-jazzy-, etc.)
    # Pattern: ros-[distro]-[package]
    if name.startswith('ros-'):
        parts = name.split('-', 2)  # Split into ['ros', 'distro', 'package-version']
        if len(parts) >= 3:
            name = parts[2]  # Take the package part after ros-distro-
        else:
            # Fallback: remove ros- prefix
            name = name[4:]

    # Take the part before the first version number (before first digit)
    # Handle cases like "ament-cmake-1.3.4-1.oe2403"
    parts = name.split('-')
    pkg_name = parts[0]

    return pkg_name

def extract_buildrequires_from_spec(spec_path):
    """
    Extract BuildRequires from spec file, handling multi-line cases
    """
    buildrequires = []

    try:
        with open(spec_path, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()

        i = 0
        while i < len(lines):
            line = lines[i].strip()

            # Check if line starts with BuildRequires:
            if line.startswith('BuildRequires:'):
                # Extract the initial requirement
                req_line = line[len('BuildRequires:'):].strip()

                # Handle multi-line BuildRequires (lines ending with \)
                while req_line.endswith('\\') and i + 1 < len(lines):
                    req_line = req_line[:-1].strip()  # Remove backslash
                    i += 1
                    next_line = lines[i].strip()
                    req_line += ' ' + next_line

                # Parse the requirements from the line
                if req_line:
                    buildrequires.extend(parse_requirements_line(req_line))

            i += 1

    except Exception as e:
        print(f"Error reading spec file {spec_path}: {e}")

    return buildrequires

def parse_requirements_line(req_line):
    """
    Parse a BuildRequires line and extract clean package names
    Handle version constraints, pkgconfig(), etc.
    """
    requirements = []

    # Split by comma first
    parts = req_line.split(',')

    for part in parts:
        part = part.strip()
        if not part:
            continue

        # Handle pkgconfig(package) format
        if part.startswith('pkgconfig(') and part.endswith(')'):
            pkg_name = part[10:-1]  # Extract package name from pkgconfig()
            requirements.append(f"pkgconfig-{pkg_name}")
            continue

        # Handle version constraints (>=, <=, =, >, <)
        # Split by space and take the first part (package name)
        pkg_parts = part.split()
        if pkg_parts:
            pkg_name = pkg_parts[0]

            # Remove any remaining version operators that might be attached
            pkg_name = re.sub(r'[><=!]+.*$', '', pkg_name)

            if pkg_name:
                requirements.append(pkg_name)

    return requirements

def is_ros_dependency(pkg_name):
    """
    Check if a package is a ROS dependency that should be filtered out
    
    Filters all packages starting with 'ros-' including:
    - ros-humble-*
    - ros-jazzy-*
    - ros-%{ros_distro}-*
    - ros-rolling-*
    - etc.
    """
    return pkg_name.startswith('ros-')

def extract_srpm(srpm_path, extract_dir):
    """
    Extract SRPM package to specified directory
    Returns the path to the extracted spec file
    """
    try:
        # Create extraction directory
        os.makedirs(extract_dir, exist_ok=True)

        # Extract SRPM using rpm2cpio and cpio
        with open(os.devnull, 'w') as devnull:
            # Convert rpm to cpio format
            rpm2cpio_proc = subprocess.Popen(
                ['rpm2cpio', srpm_path],
                stdout=subprocess.PIPE,
                stderr=devnull
            )

            # Extract cpio archive
            cpio_proc = subprocess.Popen(
                ['cpio', '-idmv'],
                stdin=rpm2cpio_proc.stdout,
                stdout=devnull,
                stderr=devnull,
                cwd=extract_dir
            )

            rpm2cpio_proc.stdout.close()
            cpio_proc.communicate()

        # Find the spec file
        for root, dirs, files in os.walk(extract_dir):
            for file in files:
                if file.endswith('.spec'):
                    return os.path.join(root, file)

        return None

    except Exception as e:
        print(f"Error extracting {srpm_path}: {e}")
        return None

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Extract BuildRequires from ROS-related SRPM packages',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '--input', '-i', 
        required=True,
        type=str,
        help='Input directory containing SRPM files'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default='./buildrequire_list_raw.json',
        help='Output JSON file path'
    )
    
    args = parser.parse_args()
    
    # Convert paths to Path objects
    input_dir = Path(args.input).resolve()
    output_file = Path(args.output).resolve()
    
    # Validate input directory
    if not input_dir.exists():
        print(f"Error: Input directory {input_dir} does not exist")
        return 1
    
    if not input_dir.is_dir():
        print(f"Error: {input_dir} is not a directory")
        return 1

    # Find all SRPM files
    srpm_files = list(input_dir.glob("*.src.rpm"))

    if not srpm_files:
        print(f"No SRPM files found in {input_dir}")
        return 1

    print(f"Found {len(srpm_files)} SRPM files in {input_dir}")

    # Dictionary to store dependencies and which packages require them
    dependencies = defaultdict(set)

    # Create temporary directory for extraction
    with tempfile.TemporaryDirectory(prefix="srpm_extract_") as temp_dir:

        for srpm_file in srpm_files:
            print(f"Processing {srpm_file.name}...")

            # Extract package name from SRPM filename
            pkg_name = extract_package_name_from_srpm(srpm_file.name)

            # Create extraction directory for this package
            extract_dir = os.path.join(temp_dir, pkg_name)

            # Extract SRPM
            spec_file = extract_srpm(str(srpm_file), extract_dir)

            if spec_file:
                # Extract BuildRequires from spec file
                buildrequires = extract_buildrequires_from_spec(spec_file)

                # Filter out ALL ROS dependencies (ros-*) and collect system dependencies
                system_deps = []
                for req in buildrequires:
                    if not is_ros_dependency(req):
                        dependencies[req].add(pkg_name)
                        system_deps.append(req)

                print(f"  Found {len(buildrequires)} BuildRequires, {len(system_deps)} system deps")
            else:
                print(f"  Warning: Could not find spec file for {srpm_file.name}")

    # Generate JSON data
    json_data = []

    for req_pkg in sorted(dependencies.keys()):
        required_by = sorted(list(dependencies[req_pkg]))

        json_data.append({
            "require_pkg": req_pkg,
            "system_pkg": "unknown",
            "install_verify": "n",
            "search_require": "n",
            "miss": "unknown",
            "required_by": required_by
        })

    # Write JSON file
    try:
        # Ensure output directory exists
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, indent=2, ensure_ascii=False)

        print(f"\nGenerated {output_file}")
        print(f"Total unique system dependencies: {len(json_data)}")
        print(f"Most required dependencies:")

        # Show top dependencies by number of packages that require them
        sorted_deps = sorted(json_data, key=lambda x: len(x['required_by']), reverse=True)
        for dep in sorted_deps[:10]:
            print(f"  {dep['require_pkg']}: required by {len(dep['required_by'])} packages")

        return 0

    except Exception as e:
        print(f"Error writing JSON file: {e}")
        return 1

if __name__ == "__main__":
    exit(main())
