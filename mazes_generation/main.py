import os
import shutil

# Directories and XML files
source_dirs = {
    "easy": "./easy",
    "medium": "./medium",
    "hard": "./hard"
}

dest_ranges = {
    "easy": range(100, 110),
    "medium": range(110, 120),
    "hard": range(120, 130)
}

# Function to create directories and copy files
def create_directories_and_copy_files(source_dir, dest_range):
    for i in dest_range:
        dest_dir = f"turtlebot3_drl_stage{i}"
        os.makedirs(dest_dir, exist_ok=True)
        
        for xml_file in os.listdir(source_dir):
            if xml_file.endswith(".xml"):
                new_file_path = os.path.join(dest_dir, "burger.model")
                shutil.copyfile(os.path.join(source_dir, xml_file), new_file_path)

# Create destination directories and copy files
for difficulty, source_dir in source_dirs.items():
    create_directories_and_copy_files(source_dir, dest_ranges[difficulty])

print("All directories and files have been created successfully.")
