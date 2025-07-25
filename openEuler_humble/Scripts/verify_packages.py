#!/usr/bin/env python3
"""
Verify system packages availability on openEuler RISC-V
Update buildrequire_list_raw.json incrementally
"""

import os
import json
import subprocess
import argparse
from datetime import datetime
from pathlib import Path

def log_info(message, verbose=False):
    """Print info message if verbose mode is enabled"""
    if verbose:
        print(f"[INFO] {message}")

def log_progress(message):
    """Always print progress message"""
    print(f"[PROGRESS] {message}")

def log_error(message):
    """Always print error message"""
    print(f"[ERROR] {message}")

def find_json_file():
    """Find the JSON file in current directory or common locations"""
    possible_names = [
        "buildrequire_list_raw.json",
        "buildrequire_list.json",
        "package_list.json"
    ]
    
    # Check current directory first
    current_dir = Path.cwd()
    for name in possible_names:
        file_path = current_dir / name
        if file_path.exists():
            return file_path
    
    # Check home directory
    home_dir = Path.home()
    for name in possible_names:
        file_path = home_dir / name
        if file_path.exists():
            return file_path
    
    # Check ~/humble_oe/ directory
    humble_oe_dir = home_dir / "humble_oe"
    for name in possible_names:
        file_path = humble_oe_dir / name
        if file_path.exists():
            return file_path
    
    return None

def get_output_filename(input_path):
    """Generate output filename based on input filename"""
    input_path = Path(input_path)
    filename = input_path.stem  # filename without extension
    extension = input_path.suffix  # .json
    
    # Remove 'raw' from filename if present
    if filename.endswith('_raw'):
        output_filename = filename[:-4] + extension  # Remove '_raw'
    elif filename.endswith('raw'):
        output_filename = filename[:-3] + extension  # Remove 'raw'
    else:
        # If no 'raw' in filename, add '_verified' to indicate it's processed
        output_filename = filename + '_verified' + extension
    
    return input_path.parent / output_filename

def verify_package_with_dnf(package_name, verbose=False):
    """
    Verify if package can be installed using dnf
    Returns tuple: (success, already_installed, error_message)
    """
    try:
        cmd = ['sudo', 'dnf', 'install', package_name, '-y']

        log_info(f"Running: {' '.join(cmd)}", verbose)

        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=300  # 5 minutes timeout
        )

        stdout = result.stdout.lower()
        stderr = result.stderr.lower()

        log_info(f"Return code: {result.returncode}", verbose)
        if verbose:
            log_info(f"STDOUT:\n{result.stdout}", verbose)
            log_info(f"STDERR:\n{result.stderr}", verbose)

        # Check if package is already installed
        if "already installed" in stdout or "nothing to do" in stdout:
            log_info(f"Package {package_name} is already installed", verbose)
            return True, True, None

        # Check if installation was successful
        if result.returncode == 0 and "complete!" in stdout:
            log_info(f"Package {package_name} installed successfully", verbose)
            return True, False, None

        # Check for common error patterns
        error_msg = result.stderr if result.stderr else result.stdout

        if "no match for argument" in stdout or "no match for argument" in stderr:
            return False, False, f"Package not found: {error_msg.strip()}"
        elif result.returncode != 0:
            return False, False, f"Installation failed (code {result.returncode}): {error_msg.strip()}"
        else:
            return False, False, f"Unknown error: {error_msg.strip()}"

    except subprocess.TimeoutExpired:
        return False, False, "Installation timeout (>5 minutes)"
    except Exception as e:
        return False, False, f"Exception during installation: {str(e)}"

def update_package_entry(json_data, package_name, success, already_installed, error_msg, verbose=False):
    """Update a single package entry in the JSON data"""
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    for entry in json_data:
        if entry["require_pkg"] == package_name:
            # Update fields based on verification result
            entry["install_verify"] = "y"
            entry["last_check"] = current_time

            if success:
                entry["miss"] = "n"
                entry["system_pkg"] = package_name  # Same as require_pkg if successful
                log_info(f"Updated {package_name}: miss=n, system_pkg={package_name}", verbose)
            else:
                entry["miss"] = "unknown"
                # Keep system_pkg as "unknown" if not successful
                log_info(f"Updated {package_name}: miss=unknown, error={error_msg}", verbose)

            return True

    return False

def save_json_data(json_data, json_file_path, verbose=False):
    """Save JSON data to file"""
    try:
        with open(json_file_path, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, indent=2, ensure_ascii=False)
        log_info(f"JSON file saved to {json_file_path}", verbose)
        return True
    except Exception as e:
        log_error(f"Failed to save JSON file: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Verify system packages on openEuler RISC-V')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='Enable verbose output with detailed information')
    parser.add_argument('-i', '--input', '--json-file', dest='json_file',
                       help='Path to the buildrequire raw JSON file (default: search in current directory)')
    parser.add_argument('--skip-update', action='store_true',
                       help='Skip dnf update at the beginning')

    args = parser.parse_args()

    # Determine JSON file path
    if args.json_file:
        # User specified a file
        json_file_path = Path(args.json_file).expanduser()
        if not json_file_path.exists():
            log_error(f"Specified JSON file not found: {json_file_path}")
            return 1
    else:
        # Auto-detect JSON file
        json_file_path = find_json_file()
        if json_file_path is None:
            log_error("No JSON file found. Please specify one using --input parameter.")
            log_error("Searched for files: buildrequire_list_raw.json, buildrequire_list.json, package_list.json")
            log_error("In locations: current directory, home directory, ~/humble_oe/")
            log_error("Usage: python3 verify_packages.py --input /path/to/your/file.json")
            return 1
        else:
            log_progress(f"Auto-detected JSON file: {json_file_path}")

    # Load JSON data
    try:
        with open(json_file_path, 'r', encoding='utf-8') as f:
            json_data = json.load(f)
        log_progress(f"Loaded {len(json_data)} packages from {json_file_path}")
    except Exception as e:
        log_error(f"Failed to load JSON file: {e}")
        return 1

    # Determine output file path
    output_file_path = get_output_filename(json_file_path)
    log_progress(f"Output will be saved to: {output_file_path}")

    # Run dnf update first (unless skipped)
    if not args.skip_update:
        log_progress("Running dnf update...")
        try:
            result = subprocess.run(['sudo', 'dnf', 'update', '-y'],
                                  capture_output=True, text=True, timeout=600)
            if result.returncode == 0:
                log_progress("dnf update completed successfully")
            else:
                log_error(f"dnf update failed: {result.stderr}")
                return 1
        except Exception as e:
            log_error(f"Failed to run dnf update: {e}")
            return 1
    else:
        log_progress("Skipping dnf update as requested")

    # Filter packages that haven't been verified yet
    packages_to_verify = [entry for entry in json_data if entry["install_verify"] == "n"]
    total_packages = len(packages_to_verify)

    if total_packages == 0:
        log_progress("All packages have already been verified")
        return 0

    log_progress(f"Starting verification of {total_packages} packages...")

    # Statistics
    success_count = 0
    already_installed_count = 0
    failed_count = 0

    # Verify each package
    for i, entry in enumerate(packages_to_verify, 1):
        package_name = entry["require_pkg"]

        log_progress(f"[{i}/{total_packages}] Verifying {package_name}...")

        # Verify package
        success, already_installed, error_msg = verify_package_with_dnf(package_name, args.verbose)

        # Update statistics
        if success:
            if already_installed:
                already_installed_count += 1
                log_progress(f"  ✓ Already installed")
            else:
                success_count += 1
                log_progress(f"  ✓ Installed successfully")
        else:
            failed_count += 1
            log_progress(f"  ✗ Failed: {error_msg}")

        # Update JSON entry
        update_package_entry(json_data, package_name, success, already_installed, error_msg, args.verbose)

        # Save JSON file after each verification (incremental update)
        if not save_json_data(json_data, output_file_path, args.verbose):
            log_error("Failed to save progress, continuing...")

    # Final statistics
    log_progress("\n" + "="*60)
    log_progress("VERIFICATION SUMMARY")
    log_progress("="*60)
    log_progress(f"Total packages verified: {total_packages}")
    log_progress(f"Successfully installed: {success_count}")
    log_progress(f"Already installed: {already_installed_count}")
    log_progress(f"Failed to install: {failed_count}")
    log_progress(f"Success rate: {((success_count + already_installed_count)/total_packages)*100:.1f}%")

    # Show failed packages
    if failed_count > 0:
        log_progress(f"\nFailed packages:")
        failed_packages = [entry["require_pkg"] for entry in json_data
                          if entry["install_verify"] == "y" and entry["miss"] == "unknown"]
        for pkg in failed_packages:
            log_progress(f"  - {pkg}")

    log_progress(f"\nResults saved to: {output_file_path}")

    return 0

if __name__ == "__main__":
    exit(main())
