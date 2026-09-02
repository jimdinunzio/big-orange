# Service Supervisor — Client Integration Guide

This document is for the **remote host** side (the machine that drives the Jetson
over the network). Hand it to Claude Code on the client machine, together with
`jetson_supervisor_client.py`, to integrate GPU-service switching.

## What the supervisor does

The Jetson runs two GPU services that **cannot run at the same time** (both bind
port 8000 and both need the single GPU):

| Logical name | systemd unit | Purpose        | Serves on |
|--------------|--------------|----------------|-----------|
| `owl`        | `nano_owl`   | NanoOWL detect | `:8000`   |
| `vlm`        | `live_vlm`   | Live VLM       | `:8000`   |

The **supervisor** is a separate XML-RPC server that starts/stops those units so
you can switch between them on command. It runs on a dedicated control port and
is always up (even while neither GPU service is running).

- **Supervisor URL:** `http://192.168.55.1:8002/` (`192.168.55.1` = Jetson over
  the USB-device network link; port **8002**).
- The arm service on `:8001` is independent and unaffected by switching.

## The switch is asynchronous + marker-gated + reboots on failure

`switch_to()` **returns immediately** with `{accepted: True, ...}`. In the
background the supervisor:

1. Stops the other GPU service (frees the GPU).
2. Starts the requested unit.
3. Waits for that service's **READY MARKER** to appear in its log before calling
   it "up" — NOT merely when the unit is active or `:8000` answers a ping. This
   is essential: the VLM's port comes up *before* the model finishes loading, and
   it can OOM mid-load. Markers:
   - owl → `[NanoOwl] Predictor loaded.`
   - vlm → `[NanoVlm] Model loaded`
4. On failure (CUDA/OOM in the log, unit exit, or timeout) the supervisor
   **REBOOTS the Jetson automatically** (`phase: "rebooting"`). It does **not**
   retry — the model is loaded exactly once per boot, because repeated
   back-to-back loads brown out the board. After the reboot, boot-restore brings
   the **last-requested** service back up (a reboot clears GPU/memory state, so
   the load usually succeeds the next time).

Learn the outcome two ways:
- **Poll** `get_status()` until `phase` is `up` (on failure the box reboots — the
  connection drops and returns with the service loading again), or
- **Register a callback** and receive pushed events.

> Because model load is memory-heavy on the Jetson, you can fire `switch_to()`
> and **disconnect/quit** to free memory, then reconnect later and read
> `get_status()` once.

## Client library

Use `jetson_supervisor_client.py` (same directory).

```python
from jetson_supervisor_client import SupervisorClient

sup = SupervisorClient("http://192.168.55.1:8002/")
sup.connect()
sup.current()                 # -> "owl" | "vlm" | "none"
sup.switch_to("vlm")          # -> {'accepted': True, 'desired': 'vlm'}
# ...poll or use a callback for the outcome...
```

## Methods

| Method | Returns | Notes |
|--------|---------|-------|
| `ping()` | `"pong"` | Supervisor liveness. |
| `current()` | `"owl"`\|`"vlm"`\|`"none"` | Active GPU service right now. |
| `switch_to(name, callback_url=None)` | `{accepted, desired}` | **Async.** `name` in `owl`,`vlm`,`none`. A failed load auto-reboots the Jetson. A new call preempts an in-progress switch. |
| `stop_all()` | same as above (`none`) | Stops both GPU services. |
| `get_status()` | status dict (below) | Progress + outcome of the current/last switch. |
| `reboot()` | ack string | Reboots the Jetson on demand (also happens automatically on a failed load). |

### `get_status()` dict
```json
{
  "active": "none",           // active GPU service now
  "desired": "vlm",           // last-requested (restored on boot)
  "owl": "inactive",          // raw systemctl is-active
  "vlm": "inactive",
  "app_port_ready": false,    // does :8000 answer ping() (informational)
  "phase": "rebooting",       // idle | switching | up | rebooting | cancelled | failed
  "attempt": 1,               // always 1 (single load per boot)
  "error": "cuda_oom",        // cuda_oom | timeout | start_failed | exception | null
  "detail": "RuntimeError: ...CUDACachingAllocator...",
  "needs_reboot": true,       // set when the load failed (box is rebooting)
  "last_error": { "service": "vlm", "error": "cuda_oom",
                  "detail": "...", "needs_reboot": true, "when": 1782... }
}
```
`phase == "up"` means the requested service is loaded and serving on `:8000`.
`phase == "rebooting"` means the load failed and the Jetson is restarting; the
connection will drop and come back with the last-requested service loading.
(`failed` only appears if the switch hit an internal supervisor exception rather
than a model-load failure.)

## Recommended flow

### A. Poll (works even if you disconnect between steps)
```python
sup.switch_to("vlm")
# (optional: quit here to free memory, reconnect later, then:)
status = sup.wait_until_settled(on_update=print)   # polls get_status()
if status["phase"] == "up":
    ...  # talk to the service on :8000
elif status["phase"] == "rebooting":
    # Load failed; the Jetson is rebooting on its own. Wait for it to come
    # back, reconnect, and poll again — boot-restore reloads the last-requested
    # service. You do NOT need to call reboot() yourself.
    ...
```
`wait_until_settled()` keeps the client running; on the memory-constrained Jetson
prefer firing `switch_to()`, quitting, then a single `get_status()` after.

### B. Push callback (remote host has memory to spare)
```python
from jetson_supervisor_client import CallbackServer

cb = CallbackServer(bind_host="0.0.0.0", port=8003, handler=print)
cb.start()
sup.switch_to("vlm", callback_url=cb.url_for("192.168.55.100"))  # this host's IP
# events arrive at your handler:
#   {'event':'attempt','service':'vlm','attempt':1}
#   {'event':'up','service':'vlm','attempt':1}
#     ...or, on a failed load (the box then reboots)...
#   {'event':'rebooting','service':'vlm','error':'cuda_oom','detail':'...'}
cb.stop()
```
The supervisor pushes to `callback_url`'s `switch_progress(event)` XML-RPC method.
`CallbackServer` implements that for you. Callbacks are best-effort — always also
trust `get_status()`.

## After a successful switch, talk to the service on :8000

Once `phase == "up"` (or the `up` event arrives), the target answers on
`http://192.168.55.1:8000/`. Use the existing `nano_owl_client.py` /
`nano_vlm_client.py` to drive it.

## Quick manual test
```bash
python3 jetson_supervisor_client.py -i           # fire-and-return; 'status' to poll
python3 jetson_supervisor_client.py -i --watch    # block and print progress until settled
# commands: owl, vlm, none, current, status, watch, ping, reboot, quit
```
