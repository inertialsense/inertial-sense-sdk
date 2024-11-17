import os
import sys
import subprocess

def activate_virtualenv():
    # Determine script directory
    script_dir = os.path.dirname(os.path.realpath(__file__))

    # Directory search list used to select virtual environment.
    dir_search_list = [
        os.getcwd(),
        script_dir,
        os.path.join(script_dir, "../../../scripts"),
        os.path.join(script_dir, "../../scripts"),
        os.path.join(script_dir, "../scripts"),
        os.path.join(script_dir, "../is-common/scripts"),
        os.path.join(script_dir, "../is-common/SDK/scripts"),
        os.path.join(script_dir, "../SDK/scripts"),
    ]

    # Check if a virtual environment is already activated
    if os.environ.get("VIRTUAL_ENV"):
        print("Virtual Environment already activated")
        print(f"VENV: {os.environ['VIRTUAL_ENV']}")
        return

    # Search for existing .venv directories and activate the first one found
    for directory in dir_search_list:
        venv_activate_path = os.path.join(directory, ".venv", "bin", "activate")
        if os.path.isfile(venv_activate_path):
            subprocess.run(f"source {venv_activate_path}", shell=True, executable="/bin/bash")
            print(f"Activated Virtual Environment at {os.path.realpath(os.path.join(directory, '.venv'))}")
            return

    # If no virtual environment is found, create one in the script directory
    print(f"No Virtual Environment found, creating one at {os.path.join(script_dir, '.venv')}")
    venv_dir = os.path.join(script_dir, ".venv")
    subprocess.run([sys.executable, "-m", "venv", venv_dir])
    venv_activate_path = os.path.join(venv_dir, "bin", "activate")
    subprocess.run(f"source {venv_activate_path}", shell=True, executable="/bin/bash")
    print(f"Activated Virtual Environment at {venv_dir}")

if __name__ == "__main__":
    activate_virtualenv()
