"""Script to download robot arm and gripper models."""
import os
import urllib.request
import zipfile
from pathlib import Path

# Base paths
BASE_DIR = Path(__file__).parent
MODELS_DIR = BASE_DIR / "models"
UR5_DIR = MODELS_DIR / "ur5"
GRIPPERS_DIR = MODELS_DIR / "grippers"

# Create directories
os.makedirs(UR5_DIR, exist_ok=True)
os.makedirs(GRIPPERS_DIR, exist_ok=True)

def download_file(url: str, target_path: Path):
    """Download a file from URL to target path."""
    print(f"Downloading {url} to {target_path}")
    urllib.request.urlretrieve(url, target_path)

def extract_zip(zip_path: Path, extract_to: Path):
    """Extract a zip file to the specified directory."""
    print(f"Extracting {zip_path} to {extract_to}")
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(extract_to)

def setup_ur5():
    """Download and set up the UR5 robot model."""
    ur5_zip = UR5_DIR / "ur5.zip"
    if not (UR5_DIR / "ur5_description").exists():
        download_file(
            "https://github.com/ros-industrial/universal_robot/archive/refs/tags/1.3.1.zip",
            ur5_zip
        )
        extract_zip(ur5_zip, UR5_DIR)
        os.remove(ur5_zip)
        os.rename(UR5_DIR / "universal_robot-1.3.1", UR5_DIR / "ur5_description")

def setup_grippers():
    """Download and set up different gripper models."""
    # Robotiq 2F-85 (parallel gripper)
    if not (GRIPPERS_DIR / "robotiq_2f_85").exists():
        gripper_zip = GRIPPERS_DIR / "robotiq_2f_85.zip"
        download_file(
            "https://github.com/ros-industrial/robotiq/archive/refs/tags/0.1.0.zip",
            gripper_zip
        )
        extract_zip(gripper_zip, GRIPPERS_DIR)
        os.remove(gripper_zip)
        os.rename(GRIPPERS_DIR / "robotiq-0.1.0", GRIPPERS_DIR / "robotiq_2f_85")
    
    # Robotiq 2F-140 (wider parallel gripper)
    if not (GRIPPERS_DIR / "robotiq_2f_140").exists():
        gripper_zip = GRIPPERS_DIR / "robotiq_2f_140.zip"
        download_file(
            "https://github.com/ros-industrial/robotiq/archive/refs/tags/0.1.0.zip",  # Same repo, different config
            gripper_zip
        )
        extract_zip(gripper_zip, GRIPPERS_DIR)
        os.remove(gripper_zip)
        os.rename(GRIPPERS_DIR / "robotiq-0.1.0", GRIPPERS_DIR / "robotiq_2f_140")
    
    # Robotiq EPick (vacuum gripper)
    if not (GRIPPERS_DIR / "robotiq_epick").exists():
        gripper_zip = GRIPPERS_DIR / "robotiq_epick.zip"
        download_file(
            "https://github.com/ros-industrial/robotiq_epick/archive/refs/tags/0.0.1.zip",
            gripper_zip
        )
        extract_zip(gripper_zip, GRIPPERS_DIR)
        os.remove(gripper_zip)
        os.rename(GRIPPERS_DIR / "robotiq_epick-0.0.1", GRIPPERS_DIR / "robotiq_epick")

if __name__ == "__main__":
    print("Setting up robot models...")
    setup_ur5()
    setup_grippers()
    print("Done setting up robot models!")
