import os
import sys
import subprocess

def batch_process_bags(root_folder, document_name, label_suffix="_label"):
    for subdir in os.listdir(root_folder):
        subdir_path = os.path.join(root_folder, subdir)
        if os.path.isdir(subdir_path):
            # Find bag files in subdir (e.g., .db3)
            bag_files = [f for f in os.listdir(subdir_path) if f.endswith(".db3")]
            if not bag_files:
                print(f"No bag file found in {subdir_path}")
                continue

            bag_file_path = os.path.join(subdir_path, bag_files[0])

            # Create final output path: documentname/sub_folder_label
            output_dir = os.path.join(document_name, f"{subdir}{label_suffix}")
            os.makedirs(output_dir, exist_ok=True)

            print(f"➡️ Processing: {bag_file_path}")
            print(f"📂 Output dir: {output_dir}")

            subprocess.run(["python", "file_for_label.py", bag_file_path, output_dir])

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python batch_extract.py <root_folder> <document_name> [optional_label_suffix]")
        sys.exit(1)

    root_folder = sys.argv[1]
    document_name = sys.argv[2]
    label_suffix = sys.argv[3] if len(sys.argv) > 3 else "_label"

    batch_process_bags(root_folder, document_name, label_suffix)

