sleep 5
cd /home/pnx/pnx_autoaim_sp/auto-aim-new
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    ./build/sentry ./configs/standard3.yaml
    # bash -c "./watchdog.sh"
