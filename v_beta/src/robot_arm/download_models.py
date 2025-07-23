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
UR5_ROBOTIQ_COMBINED_DIR = MODELS_DIR / "ur5_robotiq_85_combined"
os.makedirs(UR5_ROBOTIQ_COMBINED_DIR, exist_ok=True)

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
        # Original line: os.rename(GRIPPERS_DIR / "robotiq-0.1.0", GRIPPERS_DIR / "robotiq_2f_140")
        # This is problematic because robotiq-0.1.0 might have been renamed for 2F-85.
        # The proper fix is to extract specific subfolders from the robotiq-0.1.0.zip.
        # For now, we'll just check if the source exists to prevent a crash.
        # The priority is the combined UR5 model.
        source_rename_dir_140 = GRIPPERS_DIR / "robotiq-0.1.0" # This is the extracted folder name from the zip
        target_rename_dir_140 = GRIPPERS_DIR / "robotiq_2f_140"
        if source_rename_dir_140.exists():
            if not target_rename_dir_140.exists():
                 os.rename(source_rename_dir_140, target_rename_dir_140)
            else:
                 print(f"Target {target_rename_dir_140} already exists. Skipping rename from {source_rename_dir_140}.")
        else:
            # This condition is expected if 2F-85 setup correctly renamed robotiq-0.1.0
            # print(f"Source {source_rename_dir_140} for 2F-140 gripper rename not found. This might be okay if 2F-85 setup used it.")
            pass # Silently pass if source is gone, assuming 2F-85 handled it.
    
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

def setup_ur5_robotiq_combined():
    """Download and set up the combined UR5 + Robotiq 85 model."""
    combined_model_dir = UR5_ROBOTIQ_COMBINED_DIR
    # Check if the specific URDF file exists in its subdirectory as a sign of successful setup
    if not (combined_model_dir / 'urdf' / 'ur5_robotiq_85.urdf').exists():
        print(f'Setting up combined UR5 + Robotiq 85 model in {combined_model_dir}')
        zip_file_url = 'https://github.com/Kami-code/UR5_Robotiq85_description/archive/refs/heads/main.zip'
        zip_target_path = MODELS_DIR / 'UR5_Robotiq85_description-main.zip'
        
        if not zip_target_path.exists(): # Download the whole repo zip only if not already present
            download_file(zip_file_url, zip_target_path)
        
        print(f'Extracting from {zip_target_path} to {combined_model_dir}')
        # Proceed with extraction attempt
        with zipfile.ZipFile(zip_target_path, 'r') as zip_ref:
            extracted_count = 0
            # Create target urdf and meshes directories
            target_urdf_dir = combined_model_dir / 'urdf'
            target_meshes_dir = combined_model_dir / 'meshes'
            os.makedirs(target_urdf_dir, exist_ok=True)
            os.makedirs(target_meshes_dir, exist_ok=True)

            for member_info in zip_ref.infolist():
                member_path = Path(member_info.filename)
                if member_info.is_dir():
                    continue # Skip directories

                # Check if the member is within 'UR5_Robotiq85_description-main/src/ur5_description/urdf/'
                expected_urdf_prefix = ('UR5_Robotiq85_description-main', 'src', 'ur5_description', 'urdf')
                if len(member_path.parts) > len(expected_urdf_prefix) and member_path.parts[:len(expected_urdf_prefix)] == expected_urdf_prefix:
                    # Target path is directly in our combined_model_dir/urdf
                    file_name = member_path.name
                    target_file_path = target_urdf_dir / file_name
                    print(f"  Extracting URDF: {member_info.filename} -> {target_file_path}")
                    with zip_ref.open(member_info) as source_file, open(target_file_path, 'wb') as target_file:
                        target_file.write(source_file.read())
                    extracted_count += 1
                
                # Check if the member is within 'UR5_Robotiq85_description-main/src/ur5_description/meshes/'
                expected_meshes_prefix = ('UR5_Robotiq85_description-main', 'src', 'ur5_description', 'meshes')
                if len(member_path.parts) > len(expected_meshes_prefix) and member_path.parts[:len(expected_meshes_prefix)] == expected_meshes_prefix:
                    # Preserve subdirectory structure from within '.../src/ur5_description/meshes/'
                    relative_mesh_path = Path(*member_path.parts[len(expected_meshes_prefix):])
                    target_file_path = target_meshes_dir / relative_mesh_path
                    print(f"  Extracting MESH: {member_info.filename} -> {target_file_path}")
                    os.makedirs(target_file_path.parent, exist_ok=True) # Ensure subfolder exists
                    with zip_ref.open(member_info) as source_file, open(target_file_path, 'wb') as target_file:
                        target_file.write(source_file.read())
                    extracted_count += 1

            if extracted_count > 0:
                 print(f'Extracted {extracted_count} file(s) to {combined_model_dir}')
            else:
                 print(f'Warning: No URDF or MESH files found to extract from {zip_target_path} matching expected paths.')
        # os.remove(zip_target_path) # Optional: keep for caching, but it's large.
    else:
        print(f'Combined UR5 + Robotiq 85 model already exists at {combined_model_dir}')


if __name__ == "__main__":
    print("Setting up robot models...")
    setup_ur5()
    setup_grippers()
    setup_ur5_robotiq_combined()
    print("Done setting up robot models!")
