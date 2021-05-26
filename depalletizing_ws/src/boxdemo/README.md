# BoxDemo

## Basic requirements
```bash
python == 3.6
```

## Installation
### update pip3 first
pip3 install --upgrade pip
### 1. numpy, scipy, opencv, numba, and open3d
```bash
$pip3 install -r requirements.txt
```
### 2. pcl
```bash
$pip3 install cython==0.26.0
```
```bash
$sudo apt-get install libpcl-dev pcl-tools
```

### 3. python-pcl
```bash
$git clone https://github.com/strawlab/python-pcl.git
$cd python-pcl
```

Before build install python-pcl, replace the "setup.py of python-pcl" whth "python_pcl.setup.py of BoxDemo" to avoid possible conflicts:

1. copy "python_pcl.setup.py" of "BoxDemo" to "python-pcl"

2. delete "setup.py" of "python-pcl" and rename "python_pcl.setup.py" as "setup.py"

Then, install python-pcl
```bash
$python3 setup.py build_ext -i
$python3 setup.py install
```

## Usage
```bash
$python3 example_for_UNSTACK.py
```

## Main Parameters in config.py
```bash
PREPROCESS_THRESHOLD_DEFAULT (default = 30): tuning this value for detecting edges in the image
filter_ground_plane_para: the parameters of the ground plane. We can crop the point cloud of the ground and pre-compute the paras and save it.
dist2plane: the distance threshold for filtering points near a plane (within dist2plane)
camera_intrinsics: intrinsic paras of camera for converting between point cloud and the corresponding detpth/grey image
```

## Device requirements
```bash
>= GeForce RTX 2080 Ti 
```
