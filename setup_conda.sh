#!/usr/bin/env bash

# === Configuration ===
ENV_FILE="conda_env.yml"  
ENV_NAME=$(grep -m1 '^name:' "$ENV_FILE" | cut -d' ' -f2)

# === Functions ===

create_env() {
    if conda info --envs | awk '{print $1}' | grep -q "^$ENV_NAME$"; then
        echo "✅ Environment '$ENV_NAME' already exists. Activating it..."
    else
        echo "🚀 Creating conda environment '$ENV_NAME' from $ENV_FILE..."
        conda env create --file "$ENV_FILE"
    fi

    echo "⚠️ Please run the following to activate the environment in your shell:"
    echo "    conda activate $ENV_NAME"
}

prune_env() {
    if conda info --envs | awk '{print $1}' | grep -q "^$ENV_NAME$"; then
        echo "⚠️ Are you sure you want to delete the conda environment '$ENV_NAME'? [y/N]"
        read -r confirm
        if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
            echo "🗑 Removing environment '$ENV_NAME'..."
            conda env remove -n "$ENV_NAME"
            echo "✅ Removed."
        else
            echo "❌ Aborted."
        fi
    else
        echo "❌ Environment '$ENV_NAME' does not exist."
    fi
}

case "$1" in
    create)
        create_env
        ;;
    prune)
        prune_env
        ;;
    *)
        create_env
        ;;
esac
