#!/usr/bin/env bash

pkg_dir="$(dirname "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)")"
shell="$(basename "${SHELL}")"

function download_model() {
    if [[ ! -e $1 ]]; then
        echo "Downloading '$1'..."
        mkdir -p $(dirname $1)
        "${pkg_dir}/env/bin/python3" -m gdown -O $1 $2
    fi
}

install_crisp=true
install_models=true
install_shell=true

while :; do
    case $1 in
        --no-crisp)
            install_crisp=false
            shift
            ;;
        --no-models)
            install_models=false
            shift
            ;;
        --no-shell)
            install_shell=false
            shift
            ;;
        -h|-?|--help)
            echo "Usage: setup.sh [--no-crisp] [--no-models] [--no-shell]"
            exit
            ;;
        *)
            break
    esac
done

if [ "$install_crisp" = true ]; then
    echo "Installing crisp in: ${pkg_dir}"
    python3 -m virtualenv --clear --system-site-packages "${pkg_dir}/env"
    "${pkg_dir}/env/bin/python3" -m pip install -r "${pkg_dir}/install/crisp_requirements.txt"
    "${pkg_dir}/env/bin/python3" -m pip install git+https://github.com/MIT-SPARK/CRISP
fi

if [ "$install_models" = true ]; then
    echo "Downloading models to: ${pkg_dir}/models"
    python3 -m virtualenv --system-site-packages "${pkg_dir}/env"
    "${pkg_dir}/env/bin/python3" -m pip install gdown
    download_model "${pkg_dir}/models/202508042315_P41O3/checkpoint.pth" https://drive.google.com/file/d/1WBXGpNs7nUR4baonH85kP4eoFSeCAb_R/view?usp=sharing
    download_model "${pkg_dir}/models/202601240250_PY271/checkpoint.pth" https://drive.google.com/file/d/1a5oFGUhVpxTc54Ej7TR7AYJQOoUMMWlW/view?usp=sharing
fi

if [ "$install_shell" = true ]; then
    user_config="$HOME/.bashrc"
    case $shell in
        bash)
            echo "Exporting variables to .bashrc"
            user_config="$HOME/.bashrc"
            ;;
        zsh)
            echo "Exporting variables to .zshrc"
            user_config="$HOME/.zshrc"
            ;;
        *)
            echo "Unknown shell '$shell'!"
            exit
    esac

    echo "export PATH="'$PATH:'"${pkg_dir}/bin" >> "$user_config"
    echo "export HYDRAPP_ENV=${pkg_dir}/env" >> "$user_config"
    echo "export HYDRAPP_MODEL_DIR=${pkg_dir}/models/" >> "$user_config"
fi
