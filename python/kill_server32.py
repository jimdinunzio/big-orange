"""Kill every running server32-windows.exe (msl-loadlib 32-bit server).

    python kill_server32.py                    kill all instances
    python kill_server32.py --install-shortcut put a one-click icon on the desktop
"""
import os
import subprocess
import sys
import time

IMAGE = "server32-windows.exe"
SHORTCUT_NAME = "Kill server32.lnk"


def count_running():
    out = subprocess.run(
        ["tasklist", "/FI", f"IMAGENAME eq {IMAGE}", "/FO", "CSV", "/NH"],
        capture_output=True, text=True,
    ).stdout
    return sum(1 for line in out.splitlines() if line.lower().startswith(f'"{IMAGE}'))


def kill_all():
    before = count_running()
    if before == 0:
        print(f"No {IMAGE} processes running.")
        return 0
    subprocess.run(["taskkill", "/F", "/T", "/IM", IMAGE], capture_output=True, text=True)
    time.sleep(0.5)
    after = count_running()
    print(f"Killed {before - after} of {before} {IMAGE} process(es).")
    if after:
        print(f"{after} still running (may need an elevated prompt).")
    return 1 if after else 0


def install_shortcut():
    desktop = subprocess.run(
        ["powershell", "-NoProfile", "-Command", "[Environment]::GetFolderPath('Desktop')"],
        capture_output=True, text=True,
    ).stdout.strip()
    link = os.path.join(desktop, SHORTCUT_NAME)
    script = os.path.abspath(__file__)
    ps = (
        "$s = (New-Object -ComObject WScript.Shell).CreateShortcut($env:LNK);"
        "$s.TargetPath = $env:PY;"
        "$s.Arguments = '\"' + $env:SCRIPT + '\"';"
        "$s.WorkingDirectory = $env:WORKDIR;"
        "$s.IconLocation = 'shell32.dll,131';"
        "$s.Description = 'Kill all server32-windows.exe processes';"
        "$s.Save()"
    )
    env = dict(os.environ, LNK=link, PY=sys.executable, SCRIPT=script,
               WORKDIR=os.path.dirname(script))
    subprocess.run(["powershell", "-NoProfile", "-Command", ps], env=env, check=True)
    print(f"Created {link}")


if __name__ == "__main__":
    if "--install-shortcut" in sys.argv:
        install_shortcut()
    else:
        code = kill_all()
        time.sleep(2)  # leave the console up long enough to read when launched from the icon
        sys.exit(code)
