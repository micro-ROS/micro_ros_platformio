import subprocess

def run_cmd(command):
    return subprocess.run(command,
        capture_output = True,
        shell = True,
    )
