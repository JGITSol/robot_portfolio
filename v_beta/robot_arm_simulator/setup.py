from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="robot_arm_simulator",
    version="0.1.0",
    author="Your Name",
    author_email="your.email@example.com",
    description="A professional-grade robot arm simulation framework for industrial applications",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/robot-arm-simulator",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "pybullet>=3.2.0",
        "pyyaml>=6.0",
        "matplotlib>=3.4.0",
    ],
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "black>=21.0",
            "isort>=5.0",
            "mypy>=0.9",
            "sphinx>=4.0",
            "sphinx-rtd-theme>=0.5.0",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    entry_points={
        "console_scripts": [
            "robot-sim=robot_arm_simulator.cli:main",
        ],
    },
    include_package_data=True,
    package_data={
        "robot_arm_simulator": ["models/**/*.urdf", "models/**/*.obj", "models/**/*.stl"],
    },
)
