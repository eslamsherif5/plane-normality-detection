# Plane Normality Detection

This code performs normality detection on point cloud data of a planar surface using some filtering, clustering, and plane fitting algorithms, and it estimates the orientation of the end effector relative to the surface.

The development environment used is a docker container based on this image [eslaaam3/aric:0.1](https://hub.docker.com/r/eslaaam3/aric/tags). More on this in the [`docker`](docker/) directory in this repo.

## Usage

Use the `NormalityDetectionConfig.h` file to configure the executable using preprocessor directives

`NEW_DATA`: Comment to use filters for old data, uncomment to use filters for new data

`PCD_DIR`: Either use single file or director/patch mode

`VERBOSE`: Print all internal functions' messages

The code can be used in two different modes (using `PCD_DIR` preprocessor):

1. Single File Mode: Pass a single `.pcd` file to the executable. Comment `PCD_DIR` preprocessor.

2. Directory/Patch Mode: Pass a directory containing multiple .pcd files to the executable. Uncomment `PCD_DIR` preprocessor.

You need to recompile after changing any of the preprocessor directives.

## Code Structure

The code is organized into the following sections:

1. **Initialization**: The necessary variables and objects are initialized, and command line arguments are checked.

2. **Reading** Input: If in directory/patch mode, the program reads all the `.pcd` files in the given directory.

3. **Preprocessing**: The point cloud data is preprocessed, for example, by specifying a region of interest using pass-through filters.

4. **Filtering Algorithm**: the main purpose is extracting the cluster that best represents the planar surface and any other points/clusters are considered as outliers.
   - **Currently implemented algos**:
     - Region growing clustering algorithm. It clusters points based on their normals and curvature similarity.
     - ...

5. **Plane Fitting Algorithm**: After getting a cluster/point cloud that represents the planar surface, we use a plane fitting algo to get the coefficients of the plane to be fitted to the cloud. Currently, RANSAC algorithm is used to fit a plane to the largest cluster/cloud that we get out of 4. We might or might not need to investigate other plane fitting algos.

6. **Orientation Estimation**: The rotation matrix (and consequently, Euler angles) of the end effector (camera) is calculated relative to the fitted plane.

7. **Output**: The estimated orientation is printed out in the terminal. If in directory/patch mode, the results are also saved to a `.csv` file containing each `.pcd` file and the corresponding euler angles, and the filtered point cloud is saved to disk appended by "`_filtered`".

8. **Visualization** (optional): If enabled, the raw point cloud, normals, region growing clusters, and fitted planes are visualized using the PCLVisualizer.

### Note

***5 & 6 is where all the magic happens***

![dia](diagrams/diagram.svg)

## Compilation and Execution

### Dependencies

- PCL (Point Cloud Library) library v1.11.
- CMake v3.5.

### Building

```bash
git clone https://github.com/eslamsherif5/plane-normality-detection
cd plane-normality-detection
mkdir build
cd build
cmake ..
make
```

### Execution

To execute the code, run the compiled executable with the appropriate command line arguments:

- Single File Mode: Pass the path to a single .pcd file as a command line argument.

- Directory/Patch Mode: Pass the path to a directory containing .pcd files as the first command line argument, and the output .csv file name as the second command line argument.

As follows:

- Single file mode:  

    ```bash
    ./plane_normality <path/to/.pcd/file>
    ```

- Directory/Patch mode:

    ```bash
    ./plane_normality <path/to/.pcd/files/directory> <csv file name>` # without `.csv
    ```

For example

- Single file mode:  

    ```bash
    ./plane_normality ../data/point_clouds/0_deg/1_0deg_raw.pcd
    ```

- Directory/Patch mode:

    ```bash
    ./plane_normality ../data/point_clouds/0_deg/ 0deg_results
    ```

## Setting the Development Environment using Docker

The `docker-launch.sh` script launches a docker container with the desired container name and based on the desired image. It also creates a `~/Docker/<container_name>_workspace` directory that's shared between host and docker container.

### Running a Docker Container

```bash
docker pull eslaaam3/aric:0.1
cd docker/
chmod +x docker-launch/sh
./docker-launch.sh <container_name> <image/name:tag> <username_inside_container>
```

For example:

```bash
./docker-launch.sh plane-normality docker.io/eslaaam3/aric:0.1 aric
```

### Building and execution in Docker

After launching a docker container as mentioned above. Inside the container:

```bash
cd workspace
git clone https://github.com/eslamsherif5/plane-normality-detection
cd plane-normality-detection
chown -Rv $USERNAME .
mkdir build
cd build
cmake ..
make
```

## Known Issues

### GUI

When you stop the container and run `docker start <container_name>` and `docker attach <container_name>`, GUI doesn't work.  
To solve this issue, run:

```bash
docker rm <container_name>
./docker-launch.sh <container_name> <image/name:tag> <username_inside_container>
```

## Visualization (Optional)

Visualization of the point cloud data can be enabled by defining the `VIS` macro.

This will display:

- The raw point cloud
- Normals cloud
- Region growing clusters
- Fitted planes and normal vectors.
