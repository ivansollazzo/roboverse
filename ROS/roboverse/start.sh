#!/bin/bash

WORKING_DIR=$(pwd)
INSTALL_DIR="$WORKING_DIR/install"
DATA_DIR="$INSTALL_DIR/unicycle/data"
DOCKER_COMPOSE_FILE="$WORKING_DIR/dashboard/docker-compose.yaml"

# Funzione cleanup per CTRL+C
cleanup() {
    echo -e "\n🛑 Interruzione rilevata. Arresto in corso..."
    echo "📦 Fermiamo i container Docker..."
    docker compose -f "$DOCKER_COMPOSE_FILE" down
    echo "✅ Tutti i servizi sono stati fermati."
    exit 0
}

# Associa la funzione cleanup al segnale CTRL+C
trap cleanup SIGINT

# 1. Verifica install
if [ ! -d "$INSTALL_DIR" ]; then
    echo "❌ ERROR: Install directory not found."
    exit 1
fi

# 2. Source ROS 2
source "$INSTALL_DIR/setup.bash"

# 3. Verifica data
if [ ! -d "$DATA_DIR" ]; then
    echo "⚠️ Data directory not found. Creating..."
    mkdir -p "$DATA_DIR"
    echo "✅ Data directory created."
fi

# Check if Roboverse Data directory is available in www
if [ ! -d "$WORKING_DIR/dashboard/www/roboverse_data" ]; then
    echo "⚠️ Roboverse Data directory not found in www. Creating..."
    mkdir -p "$WORKING_DIR/dashboard/www/roboverse_data"
    echo "✅ Roboverse Data directory created."
fi

# 4. Avvia docker
echo "📦 Avvio Docker Compose..."
docker compose -f "$DOCKER_COMPOSE_FILE" up -d

# 5. Avvia ROS 2
echo "🚀 Avvio ROS 2..."
ros2 launch "$WORKING_DIR/roboverse.launch.py" &

ROS2_PID=$!

# 6. Mantieni script attivo finché ROS 2 è in esecuzione
wait $ROS2_PID
