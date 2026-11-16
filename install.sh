#!/bin/bash
set -e

if [ -f "./xbox_driver" ]; then
    BIN="./xbox_driver"
elif [ -f "./vader5s_driver_v5" ]; then
    BIN="./vader5s_driver_v5"
else
    echo "Error: Compiled driver binary not found! Compile xbox_driver.c first."
    exit 1
fi

echo "1. Installing $BIN to /usr/local/bin/xbox_driver..."
install -m 0755 "$BIN" /usr/local/bin/xbox_driver
restorecon -F /usr/local/bin/xbox_driver 2>/dev/null || true

echo "2. Setting up controller permissions (udev rule)..."
echo 'SUBSYSTEM=="input", ATTRS{name}=="Flydigi Vader 5S (userspace driver)", TAG+="uaccess"' > /etc/udev/rules.d/99-vader5s.rules
udevadm control --reload-rules && udevadm trigger

echo "3. Creating automatic startup service..."
cat << 'EOF' > /etc/systemd/system/vader5s.service
[Unit]
Description=Flydigi Vader 5S Userspace Driver
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/local/bin/xbox_driver -t -p -w -R -m 18:0x01:share
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "4. Starting and enabling the service at boot..."
systemctl daemon-reload
systemctl enable --now vader5s.service

echo "Done! The Vader 5S driver is now running in the background and will start on every boot."
