#!/bin/bash

# Launch GR00T inference client

# Default values
SERVER_HOST="localhost"
SERVER_PORT=5555
CAMERA_TOPICS='["/camera/image_raw"]'
CAMERA_NAMES='["ego_view"]'
JOINT_STATE_TOPIC="/joint_states"
LANGUAGE_INSTRUCTION_TOPIC="/language_instruction"
ACTION_TOPIC="/robot_action"
ACTION_HORIZON=8
CONTROL_FREQUENCY=50.0

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --server-host)
            SERVER_HOST="$2"
            shift 2
            ;;
        --server-port)
            SERVER_PORT="$2"
            shift 2
            ;;
        --camera-topics)
            CAMERA_TOPICS="$2"
            shift 2
            ;;
        --camera-names)
            CAMERA_NAMES="$2"
            shift 2
            ;;
        --joint-state-topic)
            JOINT_STATE_TOPIC="$2"
            shift 2
            ;;
        --language-instruction-topic)
            LANGUAGE_INSTRUCTION_TOPIC="$2"
            shift 2
            ;;
        --action-topic)
            ACTION_TOPIC="$2"
            shift 2
            ;;
        --action-horizon)
            ACTION_HORIZON="$2"
            shift 2
            ;;
        --control-frequency)
            CONTROL_FREQUENCY="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --server-host HOST             Server hostname/IP (default: localhost)"
            echo "  --server-port PORT             Server port (default: 5555)"
            echo "  --camera-topics TOPICS         Camera topics list (default: [\"/camera/image_raw\"])"
            echo "  --camera-names NAMES           Camera names list (default: [\"ego_view\"])"
            echo "  --joint-state-topic TOPIC      Joint state topic (default: /joint_states)"
            echo "  --language-instruction-topic TOPIC  Language instruction topic (default: /language_instruction)"
            echo "  --action-topic TOPIC           Action topic (default: /robot_action)"
            echo "  --action-horizon N             Action horizon (default: 8)"
            echo "  --control-frequency HZ         Control frequency (default: 50.0)"
            echo "  -h, --help                     Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

echo "Starting GR00T inference client with:"
echo "  Server: $SERVER_HOST:$SERVER_PORT"
echo "  Camera topics: $CAMERA_TOPICS"
echo "  Camera names: $CAMERA_NAMES"
echo "  Joint state topic: $JOINT_STATE_TOPIC"
echo "  Language instruction topic: $LANGUAGE_INSTRUCTION_TOPIC"
echo "  Action topic: $ACTION_TOPIC"
echo "  Action horizon: $ACTION_HORIZON"
echo "  Control frequency: $CONTROL_FREQUENCY Hz"
echo ""

# Launch the client
ros2 launch gr00t_inference client.launch.py \
    server_host:="$SERVER_HOST" \
    server_port:="$SERVER_PORT" \
    camera_topics:="$CAMERA_TOPICS" \
    camera_names:="$CAMERA_NAMES" \
    joint_state_topic:="$JOINT_STATE_TOPIC" \
    language_instruction_topic:="$LANGUAGE_INSTRUCTION_TOPIC" \
    action_topic:="$ACTION_TOPIC" \
    action_horizon:="$ACTION_HORIZON" \
    control_frequency:="$CONTROL_FREQUENCY"
