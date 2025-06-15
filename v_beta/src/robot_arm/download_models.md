# Robot Model Downloader

## Overview
This script automates the download and setup of robot models and grippers required for the simulation. It handles downloading, extracting, and organizing the necessary URDF and mesh files.

## Features
- Downloads UR5 robot model from ROS-Industrial
- Fetches multiple gripper models (Robotiq 2F-85, 2F-140, EPick)
- Handles ZIP extraction and file organization
- Skips already downloaded models

## Directory Structure
```
robot_arm/
├── models/
│   ├── ur5/                 # UR5 robot model
│   │   └── ur5_description/  # Extracted URDF and meshes
│   └── grippers/            # Various gripper models
│       ├── robotiq_2f_85/    # 85mm parallel gripper
│       ├── robotiq_2f_140/   # 140mm parallel gripper
│       └── robotiq_epick/    # Vacuum gripper
```

## Models

### UR5 Robot
- **Source**: ROS-Industrial Universal Robot package
- **Version**: 1.3.1
- **Contents**:
  - URDF description
  - Collision meshes
  - Visual meshes
  - Joint limits and dynamics

### Grippers

#### Robotiq 2F-85
- **Type**: Parallel gripper
- **Width**: 85mm
- **Source**: ROS-Industrial Robotiq package

#### Robotiq 2F-140
- **Type**: Parallel gripper
- **Width**: 140mm
- **Source**: ROS-Industrial Robotiq package

#### Robotiq EPick
- **Type**: Vacuum gripper
- **Source**: ROS-Industrial Robotiq EPick package

## Usage
```bash
# Run the download script
python -m robot_arm.download_models
```

## Dependencies
- Python Standard Library:
  - `urllib.request`
  - `zipfile`
  - `os`
  - `pathlib.Path`

## Notes
- Requires internet connection for initial download
- Uses approximately 100MB of disk space
- Models are cached locally after first download
- Safe to run multiple times (skips existing files)

## Troubleshooting
- **Permission Errors**: Ensure write access to the target directory
- **Network Issues**: Check internet connection
- **Extraction Failures**: Verify ZIP file integrity
- **Missing Dependencies**: Ensure Python standard library is complete
