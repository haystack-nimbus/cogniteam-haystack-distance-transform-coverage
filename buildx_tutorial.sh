#1. install docker buildx:
# https://docs.docker.com/buildx/working-with-buildx/

#Experimental features are now included in the standard Docker binaries as of version 1.13.0. To enable experimental features, start the Docker daemon with the --experimental flag or enable the daemon flag in the /etc/docker/daemon.json configuration file:
#

#    "experimental": true


#2. add this line to the bashrc 
#export DOCKER_CLI_EXPERIMENTAL=enabled


#
# on the terminal (inside the folder of the Dockerfile
# 
sudo docker buildx create --name nimbus-builder 
sudo docker buildx use nimbus-builder
sudo docker run --privileged --rm tonistiigi/binfmt --install all
sudo docker buildx inspect --bootstrap


#
# on the terminal (inside the folder of the Dockerfile

sudo docker buildx build --platform linux/amd64,linux/arm64 -t cognimbus/cogniteam-mce:latest --push .






