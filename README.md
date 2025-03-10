# ROS2 Bag Data Extractor

This project provides tools to extract image and LiDAR data from ROS2 `.db3` bag files and save them in a structured format, along with their timestamps.

There are two main Python scripts:

- `extract_data.py`: Extracts images and LiDAR point clouds from a **single ROS2 bag file**.
- `batch_extract.py`: Processes **all subfolders** containing `.db3` bag files within a directory and stores outputs in structured folders.

---

## 📂 Directory Structure (Example)

```
your_dataset_folder/
├── sub_folder_1/
│   └── your_bag_1.db3
├── sub_folder_2/
│   └── your_bag_2.db3
└── ...
```

After processing, outputs will be stored like:
```
output_document/
├── sub_folder_1_label/
│   ├── output_images/
│   │   ├── *.png
│   │   └── timestamps.mat
│   ├── fisheye_images_12/
│   │   ├── *.png
│   │   └── timestamps.mat
│   ├── fisheye_images_13/
│   ├── fisheye_images_14/
│   └── lidar_points/
│       ├── *.pcd
│       └── timestamps.mat
...
```

---

## Requirements

Make sure you have the following installed:

```bash
pip install numpy opencv-python scipy open3d
```

Also ensure that **ROS2 and rosbag2_py** are installed and properly sourced in your environment:
```bash
source /opt/ros/<your_ros2_distro>/setup.bash
```

---

##  `extract_data.py` – Single Bag Extraction

### Description:
Extracts camera images and LiDAR point clouds from a single `.db3` ROS2 bag file and saves:
- Images as `.png` files
- LiDAR point clouds as `.pcd` files
- Timestamps in `.mat` format (MATLAB-compatible)

### Usage:
```bash
python extract_data.py <path_to_bag_file.db3> <output_directory>
```

### Example:
```bash
python extract_data.py sub_folder_1/your_bag_1.db3 output_document/sub_folder_1_label
```

---

## `batch_extract.py` – Batch Process All Bags in Folder

### Description:
Loops through all subfolders in a specified folder, finds `.db3` files, and calls `extract_data.py` for each, saving outputs under `documentname/subfolder_label`.

### Usage:
```bash
python batch_extract.py <root_folder_with_bag_subfolders> <output_document_name> [optional_label_suffix]
```

### Example:
```bash
python batch_extract.py your_dataset_folder output_document _label
```

This will process:
- `your_dataset_folder/sub_folder_1/your_bag_1.db3`
- `your_dataset_folder/sub_folder_2/your_bag_2.db3`

And save to:
- `output_document/sub_folder_1_label/`
- `output_document/sub_folder_2_label/`

---

## Output Explanation

| Folder | Contents |
|--------|----------|
| `output_images/` | Front camera PNG images and `timestamps.mat` |
| `fisheye_images_12/13/14/` | Fisheye camera images and `timestamps.mat` |
| `lidar_points/` | `.pcd` point clouds and `timestamps.mat` |
| `timestamps.mat` | MATLAB-compatible timestamps (per topic) |

---

## 📌 Notes
- The `.pcd` files are saved using Open3D, with optional grayscale coloring by LiDAR intensity.
- You can read `timestamps.mat` in Python using `scipy.io.loadmat`:

```python
from scipy.io import loadmat
timestamps = loadmat("timestamps.mat")["timestamps"].flatten()
```

---

## 🛎 License
MIT License


