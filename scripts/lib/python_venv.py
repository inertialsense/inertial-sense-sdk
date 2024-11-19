import os
import sys
import subprocess
import venv

def is_virtual_environment(path):
    # Check for standard virtual environment files and directories
    if os.name == 'nt':  # Windows
        scripts_path = os.path.join(path, 'Scripts')
        return os.path.isdir(scripts_path) and os.path.isfile(os.path.join(scripts_path, 'activate.bat'))
    else:  # Unix-like systems
        bin_path = os.path.join(path, 'bin')
        return os.path.isdir(bin_path) and os.path.isfile(os.path.join(bin_path, 'activate'))

def create_virtual_environment(path):
    if os.path.exists(path):
        print(f"Virtual environment already exists at '{path}'.")
    else:
        venv.create(path, with_pip=True)
        print(f"Virtual environment created at '{path}'.")
        return path

def find_virtualenv():
    # Determine script directory
    script_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # Directory search list used to select virtual environment.
    dir_search_list = [
        os.getcwd(),
        script_dir,
        os.path.join(script_dir, "../../../scripts"),           # find is-gpx/scripts        from is-gpx/is-common/SDK/scripts
        os.path.join(script_dir, "../../scripts"),              # find is-gpx/scripts        from is-gpx/is-common/scripts
        os.path.join(script_dir, "../scripts"),                 # find is-gpx/scripts        from is-gpx/scripts
        os.path.join(script_dir, "../is-common/scripts"),       # find is-common/scripts     from is-gpx/scripts 
        os.path.join(script_dir, "../is-common/SDK/scripts"),   # find is-common/SDK/scripts from is-gpx/scripts 
        os.path.join(script_dir, "../SDK/scripts"),             # find is-common/SDK/scripts from is-gpx/is-common/scripts 
    ]

    # Search for existing .venv directories and return the first one found
    for directory in dir_search_list:
        if is_virtual_environment(directory):
            # print(f"Found virtual environment at {os.path.realpath(os.path.join(directory, '.venv'))}")
            return directory

    return create_virtual_environment(script_dir)

if __name__ == "__main__":
    print(find_virtualenv())
