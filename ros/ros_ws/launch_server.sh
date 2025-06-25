#!/bin/bash

# Launch GR00T inference server with custom model

# Default values
MODEL_PATH="nvidia/GR00T-N1.5-3B"
EMBODIMENT_TAG="gr1"
DATA_CONFIG="fourier_gr1_arms_only"
DENOISING_STEPS=4
SERVER_PORT=5555
DEVICE="cuda"
INFERENCE_MODE="pytorch"
TRT_ENGINE_PATH="gr00t_engine"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --model-path)
            MODEL_PATH="$2"
            shift 2
            ;;
        --embodiment-tag)
            EMBODIMENT_TAG="$2"
            shift 2
            ;;
        --data-config)
            DATA_CONFIG="$2"
            shift 2
            ;;
        --denoising-steps)
            DENOISING_STEPS="$2"
            shift 2
            ;;
        --port)
            SERVER_PORT="$2"
            shift 2
            ;;
        --device)
            DEVICE="$2"
            shift 2
            ;;
        --inference-mode)
            INFERENCE_MODE="$2"
            shift 2
            ;;
        --trt-engine-path)
            TRT_ENGINE_PATH="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --model-path PATH      Path to GR00T model (default: nvidia/GR00T-N1.5-3B)"
            echo "  --embodiment-tag TAG   Embodiment tag (default: gr1)"
            echo "  --data-config CONFIG   Data configuration (default: fourier_gr1_arms_only)"
            echo "  --denoising-steps N    Number of denoising steps (default: 4)"
            echo "  --port PORT            Server port (default: 5555)"
            echo "  --device DEVICE        Device (cuda/cpu, default: cuda)"
            echo "  --inference-mode MODE  Inference mode (pytorch/tensorrt, default: pytorch)"
            echo "  --trt-engine-path PATH Path to TensorRT engines (default: gr00t_engine)"
            echo "  -h, --help             Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

echo "Starting GR00T inference server with:"
echo "  Model path: $MODEL_PATH"
echo "  Embodiment tag: $EMBODIMENT_TAG"
echo "  Data config: $DATA_CONFIG"
echo "  Denoising steps: $DENOISING_STEPS"
echo "  Server port: $SERVER_PORT"
echo "  Device: $DEVICE"
echo "  Inference mode: $INFERENCE_MODE"
if [ "$INFERENCE_MODE" = "tensorrt" ]; then
    echo "  TensorRT engine path: $TRT_ENGINE_PATH"
fi
echo ""

# Launch the server
ros2 launch gr00t_inference server.launch.py \
    model_path:="$MODEL_PATH" \
    embodiment_tag:="$EMBODIMENT_TAG" \
    data_config:="$DATA_CONFIG" \
    denoising_steps:="$DENOISING_STEPS" \
    server_port:="$SERVER_PORT" \
    device:="$DEVICE" \
    inference_mode:="$INFERENCE_MODE" \
    trt_engine_path:="$TRT_ENGINE_PATH"
