import platform
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import setuptools
import glob

__version__ = '0.0.1'
# os.environ["CC"] = "g++-4.7" os.environ["CXX"] = "g++-4.7"

import distutils.command.build

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
static_lib_dir = '../..'
libraries = []
library_dirs = []

if sys.platform == 'win32':
    libraries.extend(static_libraries)
    library_dirs.append(static_lib_dir)
    extra_objects = []
else: # POSIX
    extra_objects = ['{}/lib{}.a'.format(static_lib_dir, l) for l in static_libraries]

source_files = ['src/log_reader.cpp' ]

ext_modules = [
    Extension('log_reader',
        sources = source_files,
        include_dirs = [
            # Path to pybind11 headers
            'include',
            '../../src',
            '../../src/libusb/libusb',
            get_pybind_include(),
            get_pybind_include(user=True)
        ],
        language='c++',
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
    name='log_reader',
    version=__version__,
    author='Walt Johnson',
    author_email='walt@inertialsense.com',
    description='pybind interface to reading InertialSense Log files',
    long_description='',
    ext_modules=ext_modules,
    install_requires=[
        'allantools<=2019.9',
        'matplotlib', 
        'numpy', 
        'pandas',
        'pybind11>=2.2', 
        'pyqt5', 
        'pyserial', 
        'pyyaml', 
        'scipy',
        'simplekml',
        'tqdm'],
    setup_requires=['pybind11>=2.2'],
    # cmdclass={'build': BuildCommand, 'build_ext': BuildExt},
    cmdclass={'build_ext': BuildExt},
    zip_safe=False,
)
