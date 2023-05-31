# Normality Detection Code Documentation

This code performs normality detection on point cloud data of a planar surface using some filtering, clustering, and plane fitting algorithms, and it estimates the orientation of the end effector relative to the surface.

## Usage

The development environment used is a docker container based on this image [eslaaam3/aric:0.1](https://hub.docker.com/r/eslaaam3/aric/tags). More on this in the `docker` directory in this repo.

The code can be used in two different modes (`DIR` preprocessor):

1. Single File Mode: Pass a single .pcd file to the executable. If `DIR` preprocessor is ***commented***

2. Directory Mode: Pass a directory containing multiple .pcd files to the executable. If `DIR` preprocessor is ***uncommented***

## Code Structure

The code is organized into the following sections:

1. **Initialization**: The necessary variables and objects are initialized, and command line arguments are checked.

2. **Reading** Input: If in directory mode, the program reads all the `.pcd` files in the given directory.

3. **Preprocessing**: The point cloud data is preprocessed, for example, by specifying a region of interest using pass-through filters.

4. **Filtering Algorithm**: the main purpose is extracting the cluster that best represents the planar surface and any other points/clusters are considered as outliers.
   - **Currently implemented algos**:
     - Region growing clustering algorithm. It clusters points based on their normals and curvature similarity.
     - ...

5. **Plane Fitting Algorithm**: After getting a cluster/point cloud that represents the planar surface, we use a plane fitting algo to get the coefficients of the plane to be fitted to the cloud. Currently, RANSAC algorithm is used to fit a plane to the largest cluster/cloud that we get out of 4. We might or might not need to investigate other plane fitting algos.

6. **Orientation Estimation**: The rotation matrix (and consequently, Euler angles) of the end effector (camera) is calculated relative to the fitted plane.

7. **Output**: The estimated orientation is printed out in the terminal. If in directory mode, the results are also saved to a `.csv` file containing each `.pcd` file and the corresponding euler angles, and the filtered point cloud is saved to disk appended by "`_filtered`".

8. **Visualization** (optional): If enabled, the raw point cloud, normals, region growing clusters, and fitted planes are visualized using the PCLVisualizer.

### Note

***5 & 6 is where all the magic happens***

## Compilation and Execution

### Dependencies

- PCL (Point Cloud Library) library v1.11.
- CMake v3.5.

### Building

In the project directory:

```shell
mkdir build
cd build
cmake ..
make
```

### Execution

To execute the code, run the compiled executable with the appropriate command line arguments:

- Single File Mode: Pass the path to a single .pcd file as a command line argument.

- Directory Mode: Pass the path to a directory containing .pcd files as the first command line argument, and the output .csv file name as the second command line argument.

As follows:

- Single file mode:  

    ```bash
    ./plane_normality <path/to/.pcd/file>
    ```

- Directory mode:

    ```bash
    ./plane_normality <path/to/.pcd/files/directory> <csv file name>` # without `.csv
    ```

For example

- Single file mode:  

    ```bash
    ./plane_normality ../data/point_clouds/0_deg/1_0deg_raw.pcd
    ```

- Directory mode:

    ```bash
    ./plane_normality ../data/point_clouds/0_deg/ 0deg_results
    ```

## Visualization (Optional)

Visualization of the point cloud data can be enabled by defining the `VIS` macro.

This will display:

- The raw point cloud
- Normals cloud
- Region growing clusters
- Fitted planes and normal vectors.
