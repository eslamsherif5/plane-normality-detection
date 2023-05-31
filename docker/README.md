# Docker

## Usage

The `docker-launch.sh` script launches a docker container with the desired container name and based on the desired image. It also creates a `~/Docker/<container_name>_workspace` directory that's shared between host and docker container.

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

## Known Issues

### GUI

When you stop the container and run `docker start <container_name>` and `docker attach <container_name>`, GUI doesn't work.  
To solve this issue, run:

```bash
docker rm <container_name>
./docker-launch.sh <container_name> <image/name:tag> <username_inside_container>
```
