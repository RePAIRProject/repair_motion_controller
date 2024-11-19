#!/bin/bash
cur_dir="$(dirname "$(readlink -f "$0")")"
config_path="$cur_dir/../config/repair_xbot_config.yaml"
xbot2-core --config $config_path --hw dummy