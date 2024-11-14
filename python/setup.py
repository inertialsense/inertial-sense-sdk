import platform
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import os.path
import sys
import glob

import setuptools
from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext

import pybind11
from pybind11.setup_helpers import Pybind11Extension

import distutils.command.build

__version__ = '2.1.10'
# os.environ["CC"] = "g++-4.7" os.environ["CXX"] = "g++-4.7"

with open("README.md", "r") as fh:
    long_description = fh.read()

class get_pybind_include(object):
    """Helper class to determine the pybind11 include path

    The purpose of this class is to postpone importing pybind11
    until it is actually installed, so that the ``get_include()``
    method can be invoked. """

    def __init__(self, user=False):
        # try:
        #     import pybind11
        # except ImportError:
        #     if subprocess.call([sys.executable, '-m', 'pip', 'install', 'pybind11']):
        #         raise RuntimeError('pybind11 install failed.')

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
    extra_objects = ['..\\build\\Release\\InertialSenseSDK.lib']
else: # POSIX
    extra_objects = ['{}/lib{}.a'.format(static_lib_dir, l) for l in static_libraries]

sdk_path = os.path.abspath(os.path.curdir + '/..')

source_files = ['inertialsense/logs/src/*.cpp' ]
include_dirs = [
    # Path to pybind11 headers
    'inertialsense/logs/include',
    sdk_path + '/src',
    sdk_path + '/src/libusb/libusb',
    # get_pybind_include(),
    # get_pybind_include(user=True)
]

ext_modules = [
    # Extension('logs',
    #     language='c++',
    #     sources = source_files,
    #     include_dirs = include_dirs,
    #     extra_objects=extra_objects
    #),
    Pybind11Extension(
        "inertialsense.logs.log_reader",
        sorted(glob.glob("inertialsense/logs/src/*.cpp")),  # Sort source files for reproducibility
        include_dirs = include_dirs,
        extra_objects=extra_objects
  ),
]


# As of Python 3.6, CCompiler has a `has_flag` method.
# cf http://bugs.python.org/issue26689
def has_flag(compiler, flagname):
    """Return a boolean indicating whether a flag name is supported on
    the specified compiler.
    """
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.cpp') as f:
        f.write('int main (int argc, char **argv) { return 0; }')
        try:
            compiler.compile([f.name], extra_postargs=[flagname])
        except setuptools.distutils.errors.CompileError:
            return False
    return True


def cpp_flag(compiler):
    """Return the -std=c++[11/17] compiler flag.

    The c++17 is prefered over c++11 (when it is available).
    """
    if has_flag(compiler, '-std=c++17'):
        return '-std=c++17'
    elif has_flag(compiler, '-std=c++11'):
        return '-std=c++11'
    else:
        raise RuntimeError('Unsupported compiler -- at least C++11 support '
                           'is needed!')


# Override build command
class BuildCommand(distutils.command.build.build):
    def initialize_options(self):
        distutils.command.build.build.initialize_options(self)
        self.build_base = '/tmp/log_inspector-build'


class BuildExt(build_ext):
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
#            if has_flag(self.compiler, '-fvisibility=hidden'):
#                opts.append('-fvisibility=hidden ')
        elif ct == 'msvc':
            opts.append('/DVERSION_INFO=\\"%s\\"' % self.distribution.get_version())
        for ext in self.extensions:
            ext.extra_compile_args = opts
        build_ext.build_extensions(self)

setup(
    name='inertialsense',
    version=__version__,
    description='Python interface to doing Inertial Sense things, like reading logs and doing mathy things.',
    url='https://github.com/InertialSense/inertial-sense-sdk',
    python_requires='>=3.6',

    author='Inertial Sense Development Team',
    author_email='devteam@inertialsense.com',
    license='MIT',
    long_description=long_description,
    long_description_content_type='text/markdown',
    classifiers=[],
    package_dir={'': '.'},
    packages=find_packages(where='../inertialsense'),

    install_requires=[
        'allantools<=2019.9',
        'matplotlib', 
        'numpy', 
        'pandas',
        'pybind11>=2.12',
        'pyqt5',
        'pyserial',
        'scipy',
        'simplekml',
        'tqdm',
        'pyyaml'],

    ext_modules=ext_modules,
    extras_require={
        "dev": [ "pytest>=7.0", "twine>=4.0.2"],
    },

    setup_requires=['pybind11>=2.12', 'setuptools', 'wheel'],

    # cmdclass={'build': BuildCommand, 'build_ext': BuildExt},
    cmdclass={'build_ext': BuildExt},
    zip_safe=False,
)
