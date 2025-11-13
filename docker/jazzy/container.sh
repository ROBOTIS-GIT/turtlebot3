#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="turtlebot3"

# Determine whether to use docker-compose or docker compose
if docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
else
    COMPOSE_CMD="docker-compose"
fi

# Function to display help
show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  help                    Show this help message"
    echo "  start                   Start the container"
    echo "  enter                   Enter the running container"
    echo "  stop                    Stop the container"
    echo ""
    echo "Examples:"
    echo "  $0 start                Start container"
    echo "  $0 enter                Enter the running container"
    echo "  $0 stop                 Stop the container"
}

# Function to start the container
start_container() {
    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    echo "Starting Turtlebot3 container..."

    # Copy udev rule for TurtleBot3
    sudo tee /etc/udev/rules.d/99-tb3.rules > /dev/null <<EOF
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE:="0666"
ATTRS{idVendor}=="fff1", ATTRS{idProduct}=="ff48", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666"
ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666", SYMLINK+="tb3_lidar"
EOF

    # Reload udev rules
    echo "Reloading udev rules..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    # Pull the latest images
    $COMPOSE_CMD -f "${SCRIPT_DIR}/docker-compose.yml" pull

    # Run docker-compose
    $COMPOSE_CMD -f "${SCRIPT_DIR}/docker-compose.yml" up -d
}

# Function to enter the container
enter_container() {
    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi
    docker exec -it "$CONTAINER_NAME" bash
}

# Function to stop the container
stop_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi

    echo "Warning: This will stop and remove the container. All unsaved data in the container will be lost."
    read -p "Are you sure you want to continue? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        $COMPOSE_CMD -f "${SCRIPT_DIR}/docker-compose.yml" down
    else
        echo "Operation cancelled."
        exit 0
    fi
}

# Main command handling
case "$1" in
    "help")
        show_help
        ;;
    "start")
        start_container
        ;;
    "enter")
        enter_container
        ;;
    "stop")
        stop_container
        ;;
    *)
        echo "Error: Unknown command"
        show_help
        exit 1
        ;;
esac
