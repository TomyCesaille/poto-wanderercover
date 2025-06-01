# poto-wanderercover

Control an WandererAstro WandererCover without a remote computer nor ZWO ASIAIR.
We're using a tiny onboarded box with physical switches to close/open the lid, control the brighness and heater.

It's a especially made for non-newer IR WandererCover versions, but it will work with newer versions too, with more features than the original remote.

## Getting started

The minimal for this project is to have:

- a WandererCover ¯\\_(ツ)_/¯
- a Raspberry pi (Raspberry Pi Zero W recommended)
- a 4-8GB microSD card
- 3 switches 3 position (ON-OFF-ON)
- a USB-A to micro USB-b cable (for the data)

### Installing on a raspberry pi zero w

1. Install the latest Raspberry Pi OS Lite on the microSD card, then boot it up and connect to your Wi-Fi network. Connect in SSH and check that python is installed:

```bash
python --version
# > 3.11.2
```

2. Setup the python virtual environment:

```bash
sudo apt-get install python3-pip python3-venv

mkdir poto-wanderercover # TODO: change to git clone
cd poto-wanderercover
python -m venv poto-wanderercover
source poto-wanderercover/bin/activate

# And install the requirements
pip install -r requirements.txt
```

3. Create a systemd service file:

For the program to run automatically on boot.

```bash
sudo nano /etc/systemd/system/poto-wanderercover.service
```

4. Add the following content to the file:

```ini
[Unit]
Description=Poto-WandererCover Control Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/poto-wanderercover
ExecStart=/bin/bash -c 'source /home/pi/poto-wanderercover/poto-wanderercover/bin/activate && python main.py'
Restart=always
RestartSec=10
StandardOutput=syslog
StandardError=syslog
SyslogIdentifier=poto-wanderercover

[Install]
WantedBy=multi-user.target
```

The `Restart=always` option ensures that the script will automatically restart if it crashes, and the `RestartSec=10` option sets a 10-second delay before restarting.

5. Enable and start the service:

```bash
sudo systemctl enable poto-wanderercover.service
sudo systemctl start poto-wanderercover.service

# Check the status of the service
sudo systemctl status poto-wanderercover.service

# If you make changes to the service file, reload the daemon
sudo systemctl daemon-reload
```

- To start the service: `sudo systemctl start poto-wanderercover.service`
- To stop the service: `sudo systemctl stop poto-wanderercover.service`
- To restart the service: `sudo systemctl restart poto-wanderercover.service`
- To disable autostart: `sudo systemctl disable poto-wanderercover.service`
- To view logs: `sudo journalctl -u poto-wanderercover.service -f`

## Contributing

### Remote coding (with the raspberry pi zero w)

Since we [can't code remotely with VS Code with the raspberry pi zero ](https://github.com/microsoft/vscode-remote-release/issues/669#issuecomment-640986010), the method is the following:

- Write the code from your dev computer
- Mirror the project to the pi using `rsync` or via git push + pull

```bash
# rsync command (adapt the IP and remove --dry-run when ready)
# The `--exclude='/poto-wanderercover/'` flag prevents syncing the virtual environment folder to the Pi.
# NOTE: Windows users, please use WSL to run the rsync command.
rsync -avzcu --delete --exclude='/poto-wanderercover/' --exclude='/__pycache__/' --exclude='/.mypy_cache/' --dry-run ./ pi@192.168.0.100:/home/pi/poto-wanderercover/
```

### Run the code (from the raspberry pi zero w)

```bash
source poto-wanderercover/bin/activate
pip install -r requirements.txt

python main.py
```

### Type checking & Linter (from the dev computer)

From ubuntu (WSL or native), run the following commands to check the types:

```bash
apt install python3-venv
python3 -m venv poto-wanderercover

# Activate the virtual environment
source poto-wanderercover/bin/activate

# Install dependencies
pip install -r requirements-dev.txt

# Run type checking
mypy .

# Run linter
ruff check .
```
