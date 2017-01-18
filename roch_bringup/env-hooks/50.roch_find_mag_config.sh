ROCH_MAG_CONFIG=$(catkin_find --etc --first-only ROCH_bringup mag_config.yaml 2>/dev/null)
if [ -z "$ROCH_MAG_CONFIG" ]; then
  ROCH_MAG_CONFIG=$(catkin_find --share --first-only ROCH_bringup config/mag_config_default.yaml 2>/dev/null)
fi

export ROCH_MAG_CONFIG
