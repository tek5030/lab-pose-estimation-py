#!/usr/bin/env bash
setup_jetson() (
  set -eux
  python3.8 -m venv venv
  source venv/bin/activate
  wget -i url-vtk.txt -O "vtk-9.0.1-cp38-cp38-linux_aarch64.whl"
  pip install -U pip wheel
  pip install -r requirements-jetson.txt
)

[[ $_ != $0 ]] || setup_jetson

