import os
import shutil

# Define the root directory
root_dir = r"C:\Users\André's PC\Documents\GitHub\Robotteknologi-4.-semester\RosBags\11test"
# Define the destination directory
all_mcaps_dir = os.path.join(root_dir, "all_mcaps")

# Create the destination directory if it doesn't exist
os.makedirs(all_mcaps_dir, exist_ok=True)

# Walk through all subdirectories and copy .mcap files
for dirpath, dirnames, filenames in os.walk(root_dir):
    for filename in filenames:
        if filename.lower().endswith('.mcap'):
            src_path = os.path.join(dirpath, filename)
            dst_path = os.path.join(all_mcaps_dir, filename)
            # If a file with the same name exists, add a number to the filename
            base, ext = os.path.splitext(filename)
            counter = 1
            while os.path.exists(dst_path):
                dst_path = os.path.join(all_mcaps_dir, f"{base}_{counter}{ext}")
                counter += 1
            shutil.copy2(src_path, dst_path)