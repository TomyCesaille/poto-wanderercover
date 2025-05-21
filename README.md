# poto-wanderercover

Control a WandererCover (not the newer IR version) without a remote computer nor ZWO ASIAIR.

We're using a tiny box with physical switches to close/open the lid, control the brighness and heater.

## Initial setup in the raspberry pi zero w

```bash
python --version
# 3.11.2

sudo apt-get install python3-pip python3-venv

python3 -m venv poto-wanderercover
source poto-wanderercover/bin/activate
```

## Remote coding with the raspberry pi zero w

Since we [can't code remotely with VS Code](https://github.com/microsoft/vscode-remote-release/issues/669#issuecomment-640986010), the method is the following:

- Write the code here
- Mirror the project to the pi using `rsync`

### Initial Configuration

Before using either method below, set up your connection details:

1. Copy `.env.example` to `.env` and update it with your Raspberry Pi settings:

   ```env
   PI_USER=your_username
   PI_HOST=your_raspberry_pi_ip
   PI_REMOTE_PATH=/home/your_username/your_project_path
   ```

### Manual rsync command

```bash
# (remove --dry-run when ready)
rsync -avz --delete --exclude='/poto-wanderercover/' --dry-run ./ tom@192.168.10.112:/home/tom/poto-wanderercover/
```

NOTE: Windows users, please use WSL to run the rsync command.

The `--exclude='/poto-wanderercover/'` flag prevents syncing the virtual environment folder to the Pi.

## Install the required packages

From the raspberry pi zero w, run the following command to install the required packages:

```bash
pip install -r requirements.txt
```

## Executing the code in the raspberry pi zero w

From the raspberry pi zero w, run the following command to execute the code:

```bash
python main.py
```
