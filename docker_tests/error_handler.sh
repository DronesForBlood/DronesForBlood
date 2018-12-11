error_handler() {
	echo "Simulation log:"
	docker logs simulation || true
	echo "GCS log:"
	docker logs gcs || true
	docker stop simulation gcs
	docker rm simulation gcs
}
trap 'error_handler $LINENO' ERR
set -e
