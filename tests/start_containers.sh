export networkname="dronesforbloodnetwork"
export networkgateway="10.40.0.1"
export simulation_ip="10.40.0.2"

export DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

set -e

#docker network create --gateway ${networkgateway} --subnet ${networkgateway}/24 ${networkname}


# enable access to xhost from container
xhost +
docker run -d --privileged \
	--name simulation \
	--network ${networkname} \
	--ip ${simulation_ip} \
	-e DISPLAY=:0 \
	-v $DIR/../simulation/Firmware:/src/firmware/:rw \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:ro" \
	crowdedlight/dronesforblood_gazebo:latest bash
echo "Simulation container started"

sleep 5
docker run -d --cap-add=NET_ADMIN --name gcs --network ${networkname} -e mavroshost=${simulation_ip} crowdedlight/dronesforblood_groundcontrol:latest

echo "Containers started. Waiting 30 seconds for system to stabilize."
sleep 30

# Source error handler
source $DIR/error_handler.sh
