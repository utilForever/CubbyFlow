# Original code from https://github.com/pybind/cmake_example/blob/master/setup.py

import os
import sys
import platform
import multiprocessing

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        """Initialize a CMake-backed extension."""
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + extdir,
            "-DPYTHON_EXECUTABLE=" + sys.executable,
            "-DBUILD_FROM_PIP=ON",
        ]

        tasking_sys = os.environ.get("TASKING_SYSTEM", "")
        if tasking_sys:
            cmake_args += ["-DCUBBYFLOW_TASKING_SYSTEM=" + tasking_sys]

        cfg = "Debug" if self.debug else "Release"
        build_args = [
            "--config",
            cfg,
            "--target",
            "pyCubbyFlow",
            "--parallel",
            str(os.environ.get("NUM_JOBS", multiprocessing.cpu_count())),
        ]

        if platform.system() == "Windows":
            cmake_args += [
                "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}".format(cfg.upper(), extdir)
            ]
            if sys.maxsize > 2**32:
                cmake_args += ["-A", "x64"]
        else:
            cmake_args += ["-DCMAKE_BUILD_TYPE=" + cfg]

        os.environ["CXXFLAGS"] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            os.environ.get("CXXFLAGS", ""), self.distribution.get_version()
        )
        self.spawn(
            ["cmake", "-S", ext.sourcedir, "-B", self.build_temp] + cmake_args
        )
        self.spawn(["cmake", "--build", self.build_temp] + build_args)


setup(
    name="pyCubbyFlow",
    version="0.7",
    author="Chris Ohk",
    author_email="utilforever@gmail.com",
    description="Voxel-based fluid simulation engine for computer games",
    long_description="",
    ext_modules=[CMakeExtension("pyCubbyFlow")],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)
