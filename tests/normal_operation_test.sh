export DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Running normal mission test"
$DIR/start_containers.sh
source $DIR/error_handler.sh

# code to launch and start test

echo "Simulation log:"
docker logs simulation
echo "GCS log:"
docker logs gcs
docker stop simulation gcs
docker rm simulation gcs
