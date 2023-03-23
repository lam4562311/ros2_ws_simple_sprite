#! /usr/bin/bash
sudo cp ros2-sprite-control.service /etc/systemd/system
sudo cp ros2_API.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable ros2-sprite-control.service
# sudo systemctl enable ros2_API.service
sudo systemctl start ros2-sprite-control.service
# sudo systemctl start ros2_API.service