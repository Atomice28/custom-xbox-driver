#!/bin/bash
set -e

echo "1. Installing driver binary to /usr/local/bin..."
install -m 0755 ./vader5s_driver_v5 /usr/local/bin/vader5s_driver_v5
restorecon -F /usr/local/bin/vader5s_driver_v5 2>/dev/null || true

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
ExecStart=/usr/local/bin/vader5s_driver_v5 -t -p -w -R -m 18:0x01:share
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "4. Starting and enabling the service at boot..."
systemctl daemon-reload
systemctl enable --now vader5s.service

echo "Done! The Vader 5S driver is now running in the background and will start on every boot."
