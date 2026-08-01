# Upstream Versions

Recorded on 2026-08-01.

| Item | Recorded value |
| --- | --- |
| OpenArm MuJoCo repository | https://github.com/enactic/openarm_mujoco |
| Installed distribution | `openarm-mujoco==2.0.1` |
| Installation source | PyPI (`pip install openarm-mujoco==2.0.1`) |
| Upstream `HEAD` | `ebc5cd29a957c8253887aab222ff3f7dc5907d4a` |
| Bimanual model | `C:\Users\Jeffr\OneDrive\Documents\GitHub\DeepAwareTakeHome\deepaware-openarm-sim\.venv\share\openarm_mujoco\v2\openarm_bimanual.xml` |
| Cell model | `C:\Users\Jeffr\OneDrive\Documents\GitHub\DeepAwareTakeHome\deepaware-openarm-sim\.venv\share\openarm_mujoco\v2\cell.xml` |

The upstream commit was obtained with `git ls-remote` against the official
repository. The local Git installation initially reported a certificate-chain
error, so the successful query used Git's Windows certificate backend:

```powershell
git -c http.sslBackend=schannel ls-remote https://github.com/enactic/openarm_mujoco.git HEAD
```

The installed artifact came from PyPI, not from the upstream Git repository.
The recorded upstream `HEAD` is provenance information only; no assertion is
made that the PyPI 2.0.1 artifact is identical to that later repository state.

