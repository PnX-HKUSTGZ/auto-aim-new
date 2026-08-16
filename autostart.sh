sleep 5
cd /home/pnx/sp_vision/sp_vision_25/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    ./build/mt_standard ./configs/standard3.yaml
    # bash -c "./watchdog.sh"
