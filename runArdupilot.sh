# Set the ArduPilot directory
ARDUPILOT_DIR="~/ardupilot"
ardupilot_info_file_path=~/aInfo.inf

# Change to the ArduPilot directory
cd "$ARDUPILOT_DIR" || { echo "Failed to change directory to $ARDUPILOT_DIR"; exit 1; }

# Configure WAF for SITL (Software In The Loop)
echo "Configuring WAF for SITL..."
"$ARDUPILOT_DIR/modules/waf/waf-light" configure --board sitl

# Build ArduPilot
echo "Building ArduPilot..."
"$ARDUPILOT_DIR/modules/waf/waf-light" build --target bin/arducopter

# Run ArduCopter
echo "Running ArduCopter..."
"$ARDUPILOT_DIR/Tools/autotest/run_in_terminal_window.sh" "ArduCopter" \
    "$ARDUPILOT_DIR/build/sitl/bin/arducopter" \
    -S --model gazebo-iris --speedup 1 --slave 0 \
    --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm" \
    -I0 --home "48.15084570555733,17.072729745416012,150.0,0.0" 

# Run MAVProxy
echo "Running MAVProxy..."
mavproxy.py --out 127.0.0.1:14550 --out 127.0.0.1:14551 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --daemon > $ardupilot_info_file_path 
