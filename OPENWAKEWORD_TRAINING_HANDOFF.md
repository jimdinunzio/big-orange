# openWakeWord Training — Handoff

**Purpose:** Continue the wake-word work on a separate **3090 Windows box** (via
VS Code Remote-WSL). This is a self-contained brief — you do **not** need the
robot codebase on the 3090 to train. It carries every decision from the prior
session plus the exact "drop-in" contract so the trained models work on the
robot unchanged.

**To resume:** put this file in your WSL workspace (e.g. `~/oww/`), open that
folder in VS Code Remote-WSL, start Claude Code there, and say:
*"Read OPENWAKEWORD_TRAINING_HANDOFF.md and continue setting up the trainer."*

---

## 1. Goal

Train **two custom openWakeWord models** for the robot "Big Orange," using the
user's **own voice recordings** for accuracy:

| Spoken phrase | `model_name` | Output file | Role |
|---|---|---|---|
| "hey orange" | `hey_orange` | `hey_orange.onnx` | LISTEN — wake, then record/transcribe command |
| "stop now" | `stop_now` | `stop_now.onnx` | IMMEDIATE — abort current operation |

## 2. Integration contract (get this right or detection silently fails)

The robot side is **already coded and committed** on the other machine
(LattePanda, Windows, conda env `big-orange-py39`, Python 3.9). Training only has
to produce two files that satisfy:

- **Exact filenames:** `hey_orange.onnx` and `stop_now.onnx`.
- **Location on robot:** `python/models/`.
- **Key = filename stem.** openWakeWord reports scores under the model's filename
  stem, and the robot reads them by stem. So set the notebook/trainer
  `model_name` to exactly `hey_orange` / `stop_now`, and **do not rename the file
  afterward** — rename via `model_name` instead.
- **ONNX only.** The robot runtime is Windows → `onnxruntime` only (no tflite).
  `inference_framework="onnx"`. Ignore/skip any `.tflite` output.
- **Thresholds:** robot uses `0.5` for each (tunable in `main.py` near the
  `OpenWakeWordListener.Config`).
- **Fatal if missing:** `main.py` asserts both files exist at startup — a missing
  model is a hard crash (intended; no fallback).

Robot-side facts (for reference; nothing to change there):
- Vendored `speech_recognition` has an `OpenWakeWordListener` class +
  `oww_config` plumbing; Porcupine code was purged; the old Vosk text-gate was
  deleted.
- `openwakeword 0.6.0` + `onnxruntime 1.19.2` already installed on the robot;
  `numpy 2.0.2` (fine for inference).
- Test harness: `python/tests/test_openwakeword_wake_word.py` (accepts model
  names/paths; resolves bundled + `python/models/` + file paths).

## 3. Key decisions (and why)

- **Train locally, not openwakeword.com.** Hosted is ~$16 / 5000 credits
  (~5 generations) and can't easily use your real recordings (only via its own
  synthesizer + voice upload). Local is free, repeatable, private, higher quality
  ceiling, and lets you inject your own clips directly.
- **Use the 3090, NOT the 5090.** A wake-word model is tiny (Colab trains it on a
  weak T4 in minutes). The **3090 is Ampere (sm_86)** — rock-solid in stable
  PyTorch. The **5090 is Blackwell (sm_120)** — needs bleeding-edge CUDA/torch
  that fights the trainer's older pinned deps, for zero speed benefit here. Save
  the 5090 for genuinely heavy work (LLM fine-tuning, etc.).
- **WSL2 + conda Python 3.10.** Training is Linux-only (Piper TTS). The WSL base
  distro version is cosmetic (24.04 fresh is fine, existing 20.04 is fine) — what
  matters is **conda Python 3.10**. Do **not** use the system Python 3.12; that is
  exactly what breaks the pipeline.
- **The official notebook/instructions are bitrotted** (dscripka/openWakeWord
  issue #296). On modern Python:
  - `piper-phonemize`, `tensorflow-cpu==2.8.1`, `tensorflow-addons` → no Py3.12
    wheels → "no matching distribution."
  - `torch_audiomentations` calls `torchaudio.set_audio_backend`, **removed in
    torchaudio 2.x** → fatal import error during training.
  - Fix path: **conda Py3.10** (restores piper-phonemize/TF wheels) + **newer
    `torch_audiomentations`** + **skip the TF→tflite conversion** (we only need
    ONNX, and that path is where the TF/onnx-tf/addons errors live).
- **Prefer a maintained trainer fork** over the official notebook:
  - `lgpearson1771/openwakeword-trainer` — patches for torchaudio 2.10+, Piper,
    speechbrain; Linux/WSL2 + CUDA; small ONNX output.
  - `CoreWorxLab/openwakeword-training` — alternative.
  - Else clone `dscripka/openWakeWord` and apply the fixes above manually.

## 4. Training-data assets (from prior Mycroft Precise work)

Ranked by value for openWakeWord:

1. **~400 custom "Hey Orange" recordings → highest value. Use as real positives.**
   openWakeWord positives are normally all-synthetic; mixing in real recordings
   of the actual voice/mic/room is the biggest robustness win for a single-user
   robot.
2. **Custom not-wake-word recordings → use the useful subset as hard negatives:**
   ones recorded in the robot's environment, or phrases that sound *similar* to
   "hey orange" (adversarial negatives cut false activations). Generic chatter =
   low value.
3. **100K downloaded generic not-wake-word → skip.** Redundant with
   openWakeWord's built-in large negative/feature corpora; not worth integrating.

**Format:** Precise used 16 kHz / 16-bit / mono WAV = openWakeWord's required
input — no resampling. Required step: run your WAVs through openWakeWord's
**feature extraction** (melspectrogram → embedding) and add to the
positive/negative feature sets. Note Precise clips often put the wake word at the
*end* / are tightly trimmed; ensure each clip cleanly contains the phrase.

> TODO on the 3090: confirm where the WAV folders are and their exact layout, then
> wire the feature-extraction + injection step into the chosen trainer's config.

## 5. Setup checklist (execute on the 3090, in WSL)

1. **WSL2 + GPU passthrough**
   - `wsl -l -v` → ensure Version **2**.
   - Install the **Windows** NVIDIA driver (includes WSL CUDA). **Do not** install
     a GPU driver *inside* WSL.
   - Verify inside WSL: `nvidia-smi` shows the **3090**.
2. **Python env**
   - Install miniconda in WSL → `conda create -n oww python=3.10 && conda activate oww`.
3. **PyTorch (Ampere)**
   - Install CUDA-enabled stable PyTorch (cu121/cu124). Verify
     `python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"`
     → `True 3090`.
4. **Trainer**
   - Clone a maintained fork (above) or `dscripka/openWakeWord` + patches; install
     training deps; upgrade `torch_audiomentations`; **skip tflite**.
5. **Datasets**
   - Download augmentation/negative data (RIRs, background noise, negative
     features) — **tens of GB**. Keep on **ext4** (e.g. `~/oww-data`), **not**
     `/mnt/c` (slow + bloats C:).
6. **Configs**
   - `hey_orange.yaml`: `target_phrase: "hey orange"`, `model_name: "hey_orange"`.
   - `stop_now.yaml`: `target_phrase: "stop now"`, `model_name: "stop_now"`.
   - Start with defaults for `n_samples`/`steps`; scale up if detection is weak.
7. **Inject real data**
   - Feature-extract the 400 positives + the useful custom negatives; add to the
     training sets.
8. **Train → ONNX**
   - Run generate → augment → train for each model; export **ONNX**.
9. **Deliver**
   - Copy `hey_orange.onnx` + `stop_now.onnx` to the robot's `python/models/`.

## 6. Validation (back on the robot / Windows)

1. `python tests/test_openwakeword_wake_word.py hey_orange stop_now` — watch live
   scores while saying each phrase; tune the `0.5` thresholds if needed.
2. Run `python python/main.py`; say **"hey orange"** → wake ring + record command
   → executes. During an operation, say **"stop now"** → aborts (red ring).
3. Sanity: removing either model file should make `main.py` abort at startup with
   the "Required wake-word model missing" assertion.

## 7. Open items

- Confirm the chosen fork's exact real-clip injection mechanism.
- Verify/record the final working versions (torch, torch_audiomentations,
  onnxruntime, openwakeword) and pin them once training succeeds.
- Decide final `n_samples`/`steps` after a first pass + threshold tuning.
