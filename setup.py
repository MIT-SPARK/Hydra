"""Setup script for building against hydra."""
import os
import re
import subprocess
import multiprocessing
import sys
import shutil
import em

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# Convert distutils Windows platform specifiers to CMake -A arguments
PLAT_TO_CMAKE = {
    "win32": "Win32",
    "win-amd64": "x64",
    "win-arm32": "ARM",
    "win-arm64": "ARM64",
}


class CMakeExtension(Extension):
    """Class to handle cmake extension config."""

    def __init__(self, name, sourcedir="", extra_cmake_flags=None):
        """Make a cmake extension config."""
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

        self.extra_cmake_flags = []
        if extra_cmake_flags is not None:
            self.extra_cmake_flags += extra_cmake_flags


class CMakeBuild(build_ext):
    """Class to build cmake extension."""

    def build_extension(self, ext):
        """Build a cmake extension."""
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # required for auto-detection & inclusion of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "RelWithDebInfo"

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
        ]
        build_args = []

        # Adding CMake arguments set as environment variable
        # (needed e.g. to build for ARM OSx on conda-forge)
        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        cmake_args += ext.extra_cmake_flags

        # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
        # across all generators.
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            parallel = multiprocessing.cpu_count()
            build_args += [f"-j{parallel}"]

        if os.path.exists(self.build_temp):
            shutil.rmtree(self.build_temp)

        os.makedirs(self.build_temp)

        subprocess.check_call(
            ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp
        )
        subprocess.check_call(
            ["cmake", "--build", "."] + build_args, cwd=self.build_temp
        )


empy_path = re.compile("/__init__.py.*").sub("", em.__file__)
cmake_flags = [
    f"-DPY_EM={empy_path}",
    "-DHYDRA_ENABLE_EVAL=OFF",
    "-DHYDRA_ENABLE_GNN=OFF",
    "-DHYDRA_ENABLE_TESTS=OFF",
    "-DHYDRA_ENABLE_PYTHON=ON",
    "-DBUILD_SHARED_LIBS=OFF",
    # "-DHYDRA_ENABLE_COVERAGE=ON",
]
ccache = shutil.which("ccache")
if ccache:
    cmake_flags.append(f"-DCMAKE_CXX_COMPILER_LAUNCHER={ccache}")

cmake_modules = [
    CMakeExtension(
        "hydra_python._hydra_bindings", sourcedir=".", extra_cmake_flags=cmake_flags
    )
]

setup(ext_modules=cmake_modules, cmdclass={"build_ext": CMakeBuild})
