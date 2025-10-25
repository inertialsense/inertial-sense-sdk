import platform
import os.path
import sys
import glob

import setuptools.errors
from setuptools import setup, Extension, find_packages
from setuptools.command.build import build as _build
from setuptools.command.build_ext import build_ext as _build_ext

import pybind11
from pybind11.setup_helpers import Pybind11Extension


__version__ = '2.7.0'
# os.environ["CC"] = "g++-4.7" os.environ["CXX"] = "g++-4.7"

with open("README.md", "r") as fh:
    long_description = fh.read()

class get_pybind_include(object):
    """Helper class to determine the pybind11 include path."""

    def __init__(self, user=False):
        self.user = user

    def __str__(self):
        import pybind11
        return pybind11.get_include(self.user)

if platform.system() == 'Windows':
    macros = [("UNICODE", "1")]     # Necessary for ISFileManager.cpp
else:
    macros = []

static_libraries = ['InertialSenseSDK']
static_lib_dir = '..'
libraries = []
library_dirs = []

if sys.platform == 'win32':
    extra_objects = [r'..\build\InertialSenseSDK.lib']
else: # POSIX
    extra_objects = ['{}/build/lib{}.a'.format(static_lib_dir, l) for l in static_libraries]

sdk_path = os.path.abspath(os.path.curdir + '/..')

source_files = ['inertialsense/logs/src/log_reader.cpp' ]  # sorted(glob.glob("inertialsense/logs/src/*.cpp")),  # Sort source files for reproducibility
include_dirs = [
    'inertialsense/logs/include',   # local headers
    pybind11.get_include(),  # Path to pybind11 headers
    os.path.join(sdk_path, "src"),
    os.path.join(sdk_path, "src", "libusb", "libusb"),
]

print(sdk_path)
print(include_dirs)

ext_modules = [
    # Extension('inertialsense.logs',
    #     language='c++',
    #     sources = source_files,
    #     include_dirs = include_dirs,
    #     extra_objects=extra_objects
    #),
    Pybind11Extension(
        "inertialsense.log_reader",
        source_files,
        include_dirs = include_dirs,
        extra_objects = extra_objects
    ),
]

def has_flag(compiler, flagname):
    """Return a boolean indicating whether a flag name is supported on
    the specified compiler."""
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.cpp') as f:
        f.write('int main (int argc, char **argv) { return 0; }')
        try:
            compiler.compile([f.name], extra_postargs=[flagname])
        except setuptools.errors.CompileError:
            return False
    return True

def cpp_flag(compiler):
    """Return the -std=c++[11/17] compiler flag."""
    if has_flag(compiler, '-std=c++17'):
        return '-std=c++17'
    elif has_flag(compiler, '-std=c++11'):
        return '-std=c++11'
    else:
        raise RuntimeError('Unsupported compiler -- at least C++11 support '
                           'is needed!')


# Override build command
class BuildCommand(_build):
    def initialize_options(self):
        _build.initialize_options(self)
        self.build_base = '/tmp/log_inspector-build'


class BuildExt(_build_ext):
    """A custom build extension for adding compiler-specific options."""
    c_opts = {
        'msvc': ['/EHsc'],
        'unix': ['-O0', '-g'],
    }

    if sys.platform == 'darwin':
        c_opts['unix'] += ['-stdlib=libc++', '-mmacosx-version-min=10.7']

    def build_extensions(self):
        ct = self.compiler.compiler_type
        opts = self.c_opts.get(ct, [])
        if ct == 'unix':
            opts.append('-DVERSION_INFO="%s"' % self.distribution.get_version())
            opts.append(cpp_flag(self.compiler))
        elif ct == 'msvc':
            opts.append('/DVERSION_INFO=\\"%s\\"' % self.distribution.get_version())
            opts.append('/std:c++17')
            opts.append('/DYAML_CPP_STATIC_DEFINE')  # if static yaml-cpp
            # opts += ['/wd4251', '/wd4275']  # silence yaml-cpp DLL warnings
        for ext in self.extensions:
            ext.extra_compile_args = opts
        _build_ext.build_extensions(self)

setup(
    name='inertialsense',
    version=__version__,
    description='Python interface to doing Inertial Sense things, like reading logs and doing mathy things.',
    url='https://github.com/InertialSense/inertial-sense-sdk',

    long_description=long_description,
    long_description_content_type='text/markdown',
    package_dir={'': '.'},
#    packages=find_packages(include=['inertialsense', 'inertialsense.logs']),
    packages=find_packages(where='../inertialsense'),

    ext_modules=ext_modules,

    setup_requires=['pybind11>=2.12', 'setuptools', 'wheel'],

    # cmdclass={'build': BuildCommand, 'build_ext': BuildExt},
    cmdclass={'build_ext': BuildExt},
    zip_safe=False,
)
