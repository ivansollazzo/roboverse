#!/bin/bash

# Roboverse Launch File
# This file contains all the launch instructions for the Roboverse system
# Written by: Ivan Sollazzo

# Environment variables
WORKING_DIR=$(pwd)
INSTALL_DIR="$WORKING_DIR/install"
DATA_DIR="$INSTALL_DIR/unicycle/data"
DOCKER_COMPOSE_FILE="$WORKING_DIR/dashboard/docker-compose.yaml"
ROS_TCP_ENDPOINT_LAUNCH="$WORKING_DIR/src/ros_tcp_endpoint/launch/endpoint.py"
ROBOVERSE_LAUNCH="$WORKING_DIR/roboverse_nodes.launch.xml"

# Store process IDs
ROS_TCP_PID=""
ROS_NODES_PID=""

# Task manager crash tracking
TASK_MANAGER_CRASHED=false

# Set colors for output
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m' # means no color

# Function to check if any task manager is running
check_task_manager() {
    if [ ! -z "$ROS_NODES_PID" ] && kill -0 $ROS_NODES_PID 2>/dev/null; then
        # Check if any task manager process is actually running
        if pgrep -f "_task_manager" > /dev/null; then
            return 0  # at least one task manager is running
        else
            return 1  # all task managers have crashed
        fi
    else
        return 1  # ROS nodes process has died
    fi
}

# Function to get count of running task managers
get_task_manager_count() {
    local count=$(pgrep -f "_task_manager" 2>/dev/null | wc -l)
    echo "$count"
}

# Function to list running task managers
list_task_managers() {
    pgrep -af "_task_manager" 2>/dev/null | grep -v "pgrep\|grep" | awk '{for(i=2;i<=NF;i++) printf $i" "; print ""}' | grep -o '[a-zA-Z0-9][a-zA-Z0-9_]*_task_manager' | sort -u
}

# Roboverse cleanup system
cleanup() {
    echo -e "\n${RED}🛑 Roboverse is shutting down...${NC}"
    
    # Shutdown of nodes
    echo "🤖 Shutting down ROS nodes..."
    if [ ! -z "$ROS_NODES_PID" ] && kill -0 $ROS_NODES_PID 2>/dev/null; then
        
        # Let's try with a graceful shutdown, so send a term signal
        kill -INT $ROS_NODES_PID 2>/dev/null
        sleep 3

        # If it's still running, send a kill signal
        if kill -0 $ROS_NODES_PID 2> /dev/null; then
            echo "Force killing ROS launch process..."
            kill -KILL $ROS_NODES_PID 2>/dev/null
        fi
    
    fi
    
    # Shutdown of ROS TCP Endpoint
    echo "🔌 Shutting down ROS TCP Endpoint..."
    if [ ! -z "$ROS_TCP_PID" ] && kill -0 $ROS_TCP_PID 2>/dev/null; then
        
        # Let's try with a graceful shutdown, so send a term signal
        kill -INT $ROS_TCP_PID 2> /dev/null
        sleep 3

        # If it's still running, send a kill signal
        if kill -0 $ROS_TCP_PID 2>/dev/null; then
            kill -KILL $ROS_TCP_PID 2>/dev/null
        fi
    
    fi
    
    # Shutdown of Docker container
    echo "🐳 Stopping Docker container for Roboverse Dashboard..."
    docker compose -f "$DOCKER_COMPOSE_FILE" down 2>/dev/null
    
    echo -e "${GREEN}✅ System has been stopped successfully!${NC}"
    exit 0
}

# Handle all termination signals
trap cleanup SIGINT SIGTERM EXIT

echo -e "${GREEN}🚀 Roboverse is starting...${NC}"

# Check if install directory exists
if [ ! -d "$INSTALL_DIR" ]; then
    echo -e "${RED}❌ ERROR: Install directory not found${NC}"
    exit 1
fi

# Source the ROS install dir
source "$INSTALL_DIR/setup.bash"

# Create directories if needed
mkdir -p "$DATA_DIR"
mkdir -p "$WORKING_DIR/dashboard/www/roboverse_data"

# Start Docker
echo "🐳 Docker is starting..."
docker compose -f "$DOCKER_COMPOSE_FILE" up -d
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠️  Something went wrong with Docker. Maybe the Docker daemon is not running or some files are missing. Roboverse will continue to start without Docker, so the dashboard won't be available.${NC}"
fi

# Start ROS TCP Endpoint for Unity simulation
echo "🔌 Starting ROS TCP Endpoint for Unity simulation..."
ros2 launch "$ROS_TCP_ENDPOINT_LAUNCH" &
ROS_TCP_PID=$!

# Start ROS nodes
echo "🤖 Starting ROS nodes..."
ros2 launch "$ROBOVERSE_LAUNCH" &
ROS_NODES_PID=$!

# We give time to nodes to initialize
sleep 2
echo -e "${GREEN}✅ System started successfully. Press CTRL + C to gently stop at anytime.${NC}"

INITIAL_TASK_MANAGERS=$(list_task_managers)
INITIAL_TM_COUNT=$(get_task_manager_count)

# Now we can monitor task managers
if [ "$INITIAL_TM_COUNT" -gt 0 ]; then
    echo -e "${GREEN}🎯 Detected $INITIAL_TM_COUNT task manager(s):${NC}"
    echo "$INITIAL_TASK_MANAGERS" | while read -r tm; do
        [ ! -z "$tm" ] && echo -e "${GREEN}   - $tm${NC}"
    done
else
    echo -e "${YELLOW}⚠️  No task managers detected yet. Monitoring...${NC}"
fi

# Continuous monitoring
while true; do
    # Check if ROS TCP Endpoint is still running
    if ! kill -0 $ROS_TCP_PID 2>/dev/null; then
        echo -e "${YELLOW}⚠️  ROS TCP Endpoint has crashed. System continues running.${NC}"
        echo -e "${YELLOW}   Consider stopping the system with CTRL + C and restarting if Unity connection is needed.${NC}"
        ROS_TCP_PID=""  # Mark as dead to avoid repeated checks
    fi
    
    # Check task manager
    CURRENT_TM_COUNT=$(get_task_manager_count)
    if [ "$CURRENT_TM_COUNT" -eq 0 ] && [ "$INITIAL_TM_COUNT" -gt 0 ]; then
        if [ "$TASK_MANAGER_CRASHED" = false ]; then
            echo -e "${RED}💥 CRITICAL: All task managers have crashed!${NC}"
            echo -e "${RED}🛑 System cannot continue safely. Initiating shutdown...${NC}"
            TASK_MANAGER_CRASHED=true
            cleanup
        fi
    elif [ "$CURRENT_TM_COUNT" -lt "$INITIAL_TM_COUNT" ]; then
        # Some task managers crashed but not all
        echo -e "${YELLOW}⚠️  Warning: Some task managers have crashed ($CURRENT_TM_COUNT/$INITIAL_TM_COUNT running)${NC}"
        echo -e "${YELLOW}  Consider stopping the system with CTRL + C and restarting if you need all robots operational.${NC}"
    fi
    
    # If the main ROS launch process dies completely, that's critical too
    if [ ! -z "$ROS_NODES_PID" ] && ! kill -0 $ROS_NODES_PID 2>/dev/null; then
        echo -e "${RED}💥 CRITICAL: Main ROS launch process has terminated!${NC}"
        echo -e "${RED}🛑 System cannot continue. Shutting down...${NC}"
        cleanup
    fi
    
    sleep 2
done