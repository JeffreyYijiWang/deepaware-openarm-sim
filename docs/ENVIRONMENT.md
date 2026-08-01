# Development Environment

Recorded on 2026-08-01 in PowerShell on Windows.

## Host inspection

- Operating system: Windows 10 Home 25H2, build 26200.8973, x64
- PowerShell: 5.1.26100.8972
- Git: 2.54.0.windows.1
- Initial current directory: `C:\Users\Jeffr\OneDrive\Documents\GitHub\DeepAwareTakeHome`
- Initial current directory was a Git repository: no
- Project directory was already an independent Git repository: yes (`deepaware-openarm-sim`, empty with no commits)
- Active virtual environment before setup: none
- Python installations reported by `py -0p`:
  - Python 3.13: `C:\Python313\python.exe`
  - Python 3.11: `C:\Program Files\WindowsApps\PythonSoftwareFoundation.Python.3.11_3.11.2544.0_x64__qbz5n2kfra8p0\python3.11.exe`
- Selected Python: 3.11.9. Python 3.12 was not installed, so the requested 3.11 fallback was used.
- Virtual environment: `C:\Users\Jeffr\OneDrive\Documents\GitHub\DeepAwareTakeHome\deepaware-openarm-sim\.venv`
- Python executable: `C:\Users\Jeffr\OneDrive\Documents\GitHub\DeepAwareTakeHome\deepaware-openarm-sim\.venv\Scripts\python.exe`
- PowerShell execution-policy change: not required

## Installed direct dependencies

| Dependency | Version |
| --- | ---: |
| openarm-mujoco | 2.0.1 |
| matplotlib | 3.11.1 |
| pandas | 3.0.5 |
| PyYAML | 6.0.3 |
| pytest | 9.1.1 |
| ruff | 0.16.1 |

Packaging tools were upgraded inside `.venv` to pip 26.2, setuptools 83.0.0,
and wheel 0.47.0. The full resolved environment is captured in
`requirements-lock.txt`.

MuJoCo version: 3.11.0.

The first packaging-tool upgrade attempt encountered an SSL certificate-chain
error. The retry used pip's Windows system trust-store support
(`--use-feature=truststore`) and completed without disabling TLS verification.

## Verification

After activating `.venv`, the successful headless command was:

```powershell
python scripts/smoke_test.py
```

The test resolved both OpenArm v2 model paths, loaded the bimanual model, created
`MjData`, completed `mj_forward`, and completed one `mj_step`. It exited with
status 0.

Viewer command:

```powershell
openarm-mujoco-launch --no-sheet
```

Viewer status: launched successfully. The process remained alive for the
12-second verification window, produced no stdout or stderr, and was then
stopped intentionally after the startup check. No OpenGL or display error was
observed.

