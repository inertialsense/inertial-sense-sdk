import os
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
        print(f"Virtual environment already exists: '{path}'.")
    else:
        venv.create(path, with_pip=True)
        print(f"New virtual environment created: '{path}'.")
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

    # Search for existing .venv directory and return first one found
    for directory in dir_search_list:
        directory = os.path.realpath(os.path.join(directory, ".venv"))
        if is_virtual_environment(directory):
            print(f"Found virtual environment: {directory}")
            return directory

    # Virtual environment not found.  Create one.
    return create_virtual_environment(os.path.realpath(os.path.join(script_dir, ".venv")))

if __name__ == "__main__":
    print(find_virtualenv())
