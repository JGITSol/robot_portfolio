from setuptools import setup, find_packages
import os

# Read requirements from requirements-robot.txt
with open('requirements-robot.txt') as f:
    requirements = f.read().splitlines()

# Get long description from README
with open('README.md', 'r', encoding='utf-8') as f:
    long_description = f.read()

setup(
    name="robot_arm_simulator",
    version="1.0.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=requirements,
    python_requires=">=3.8",
    author="Your Name",
    author_email="your.email@example.com",
    description="Advanced robotic arm simulation with PyBullet featuring multi-robot coordination and production line automation",
    long_description=long_description,
    long_description_content_type="text/markdown",
    license="MIT",
    keywords="robotics simulation pybullet automation production-line robot-arm",
    url="https://github.com/yourusername/robot_arm_simulator",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    entry_points={
        'console_scripts': [
            'robot-arm-demo=robot_arm.demo:main',
        ],
    },
    include_package_data=True,
    package_data={
        'robot_arm': ['*.urdf', '*.obj', '*.mtl'],
    },
)
