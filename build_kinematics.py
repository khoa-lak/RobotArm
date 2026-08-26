import os
import sys
import subprocess

def compile_extension():
    print("Kiem tra va cai dat pybind11 va setuptools...")
    # Clean check and install pybind11/setuptools
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pybind11", "setuptools"])

    print("Dang bat dau bien dich robot_kinematics.cpp sang file binary .pyd...")
    
    from setuptools import setup, Extension
    from pybind11.setup_helpers import Pybind11Extension, build_ext

    # Set compiler options if on Windows
    extra_compile_args = []
    if sys.platform == "win32":
        extra_compile_args = ["/EHsc"] # Standard C++ Exception Handling for MSVC

    ext_modules = [
        Pybind11Extension(
            "robot_kinematics",
            ["kinematics_src/bindings.cpp"],
            include_dirs=["kinematics_src"],
            extra_compile_args=extra_compile_args,
        ),
    ]

    # Save sys.argv to restore later
    old_argv = sys.argv
    # Force setuptools to build inplace (in current directory)
    sys.argv = [sys.argv[0], "build_ext", "--inplace"]

    try:
        setup(
            name="robot_kinematics",
            ext_modules=ext_modules,
            cmdclass={"build_ext": build_ext},
            script_args=["build_ext", "--inplace"]
        )
        print("\n=======================================================")
        print("BIEN DICH THANH CONG!")
        print("File binary robot_kinematics.pyd da duoc tao o thu muc goc.")
        print("=======================================================")
    except Exception as e:
        print(f"\n[ERROR] Bien dich that bai: {e}")
        print("Hay dam bao ban da cai dat Visual Studio Build Tools hoac MSVC Compiler.")
    finally:
        sys.argv = old_argv

if __name__ == "__main__":
    compile_extension()
