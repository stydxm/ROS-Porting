#!/usr/bin/env python3
"""
Verify system packages availability on openKylin
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
    
    # Check ~/verify_json/ directory (based on the extract script)
    verify_json_dir = home_dir / "verify_json"
    for name in possible_names:
        file_path = verify_json_dir / name
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

def check_package_installed(package_name):
    """Check if package is already installed using dpkg"""
    try:
        result = subprocess.run(
            ['dpkg-query', '-W', '-f=${Status}', package_name],
            capture_output=True,
            text=True
        )
        return "install ok installed" in result.stdout
    except:
        return False

def verify_package_with_apt(package_name, verbose=False):
    """
    Verify if package can be installed using apt
    Returns tuple: (success, already_installed, error_message)
    """
    try:
        # First check if already installed
        if check_package_installed(package_name):
            log_info(f"Package {package_name} is already installed", verbose)
            return True, True, None

        # Try to install the package
        cmd = ['sudo', 'apt', 'install', package_name, '-y']

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

        # Check if package is already installed (double check)
        if "already the newest version" in stdout or "is already the newest version" in stdout:
            log_info(f"Package {package_name} is already installed", verbose)
            return True, True, None

        # Check if installation was successful
        if result.returncode == 0:
            # Verify installation by checking with dpkg
            if check_package_installed(package_name):
                log_info(f"Package {package_name} installed successfully", verbose)
                return True, False, None

        # Check for common error patterns
        error_msg = result.stderr if result.stderr else result.stdout

        if "unable to locate package" in stdout or "unable to locate package" in stderr:
            return False, False, f"Package not found: {package_name}"
        elif "e: package" in stderr and "has no installation candidate" in stderr:
            return False, False, f"No installation candidate for {package_name}"
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
                entry["search_require"] = "n"  # No search required if found
                log_info(f"Updated {package_name}: miss=n, system_pkg={package_name}", verbose)
            else:
                entry["miss"] = "unknown"
                entry["search_require"] = "y"  # Need manual search if not found
                # Keep system_pkg as "unknown" if not successful
                log_info(f"Updated {package_name}: miss=unknown, search_require=y, error={error_msg}", verbose)

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
    parser = argparse.ArgumentParser(description='Verify system packages on openKylin using APT')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='Enable verbose output with detailed information')
    parser.add_argument('-i', '--input', '--json-file', dest='json_file',
                       help='Path to the buildrequire raw JSON file (default: search in common locations)')
    parser.add_argument('--skip-update', action='store_true',
                       help='Skip apt update at the beginning')

    args = parser.parse_args()

    # Check if running on a Debian-based system (openKylin should be)
    if not Path('/usr/bin/apt').exists():
        log_error("APT package manager not found. This script requires a Debian-based system.")
        return 1

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
            log_error("In locations: current directory, home directory, ~/verify_json/")
            log_error("Usage: python3 verify_packages_openkylin.py --input /path/to/your/file.json")
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

    # Run apt update first (unless skipped)
    if not args.skip_update:
        log_progress("Running apt update...")
        try:
            result = subprocess.run(['sudo', 'apt', 'update'],
                                  capture_output=True, text=True, timeout=600)
            if result.returncode == 0:
                log_progress("apt update completed successfully")
            else:
                log_error(f"apt update failed: {result.stderr}")
                return 1
        except Exception as e:
            log_error(f"Failed to run apt update: {e}")
            return 1
    else:
        log_progress("Skipping apt update as requested")

    # Filter packages that haven't been verified yet
    packages_to_verify = [entry for entry in json_data if entry.get("install_verify", "n") == "n"]
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
        success, already_installed, error_msg = verify_package_with_apt(package_name, args.verbose)

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
    
    success_rate = 0.0
    if total_packages > 0:
        success_rate = ((success_count + already_installed_count)/total_packages)*100
    log_progress(f"Success rate: {success_rate:.1f}%")

    # Show failed packages that need manual search
    failed_packages = [entry for entry in json_data
                      if entry.get("install_verify", "n") == "y" and entry.get("miss", "n") == "unknown"]
    
    if failed_packages:
        log_progress(f"\nPackages requiring manual search ({len(failed_packages)}):")
        for entry in failed_packages:
            log_progress(f"  - {entry['require_pkg']} (search_require: {entry.get('search_require', 'n')})")

    log_progress(f"\nResults saved to: {output_file_path}")

    return 0

if __name__ == "__main__":
    exit(main())
