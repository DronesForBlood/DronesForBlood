#!/bin/bash
docker login -u="$DOCKER_USERNAME" -p="$DOCKER_PASSWORD"
docker push crowdedlight/dronesforblood_groundcontrol:latest
docker push crowdedlight/dronesforblood_gazebo:latest
