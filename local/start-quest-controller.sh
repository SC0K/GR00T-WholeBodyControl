#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/controller-env.sh"
cd "$SONIC_ROOT/gear_sonic_deploy"
exec ./target/release/g1_deploy_onnx_ref lo policy/release/model_decoder.onnx reference/example/ \
 --obs-config policy/release/observation_config.yaml \
 --encoder-file policy/release/model_encoder.onnx \
 --planner-file planner/target_vel/V2/planner_sonic.onnx \
 --disable-crc-check --input-type zmq_manager --output-type all --zmq-host 127.0.0.1
