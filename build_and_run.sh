echo "Building the Docker image..."
docker build -t tp_gmm:latest .

echo "Running the Docker container..."
docker run -it --rm --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" tp_gmm:latest
