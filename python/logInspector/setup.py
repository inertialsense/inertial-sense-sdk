import platform
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import setuptools

__version__ = '2.2.1'

class get_pybind_include(object):
    """Helper class to determine the pybind11 include path."""

    def __init__(self, user=False):
        self.user = user

    def __str__(self):
        import pybind11
        return pybind11.get_include(self.user)

ext_modules = [
    Extension(
        'log_reader',
        sources=[
            'src/log_reader.cpp',  # Only include the specific source file
        ],
        include_dirs=[
            'include',
            '../src',
            '../../src',  # Include any necessary headers from the SDK
            get_pybind_include(),
            get_pybind_include(user=True),
        ] + (['../../src/libusb/libusb'] if platform.system() == 'Windows' else ['/usr/include/libusb-1.0']),
        extra_link_args=[] if platform.system() == 'Windows' else ['-lusb-1.0'],
        # Link to the prebuilt static library
        extra_objects=['../../build-release/InertialSenseSDK.lib'] if platform.system() == 'Windows' else ['../../build/libInertialSenseSDK.a'],
        language='c++',
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
        except setuptools.distutils.errors.CompileError:
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

class BuildExt(build_ext):
    """A custom build extension for adding compiler-specific options."""
    c_opts = {
        'msvc': ['/EHsc'],
        'unix': ['-O3'],
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
        'tqdm'
    ],
    setup_requires=['pybind11>=2.2'],
    cmdclass={'build_ext': BuildExt},
    zip_safe=False,
)
