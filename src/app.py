# touch_ui.py
import os
import sys
import subprocess
import threading
import queue
import time
from pathlib import Path

import streamlit as st

# ========== CONFIG ==========
DEFAULT_SCRIPT_PATH = Path(__file__).with_name("manikin_controller.py")
PYTHON_BIN = sys.executable
# ============================

st.set_page_config(page_title="Manikin Head Controller", layout="centered")

# --- Duke Blue theme + buttons ---
st.markdown(
    """
    <style>
    :root {
        --duke-blue: #001A57;
        --duke-blue-light: #003366;
        --duke-red: #990000;
        --inactive-gray: #e0e0e0;
    }

    .stButton > button {
        height: 88px;
        font-size: 1.15rem;
        font-weight: 700;
        line-height: 1.2;
        padding: 0.6rem 0.8rem;
        border-radius: 14px;
        white-space: pre-line;
        text-align: center;
        transition: background-color 0.15s ease, color 0.15s ease, border 0.15s ease;
    }

    /* Primary (active / highlighted) */
    div[data-testid="stButton"] > button[kind="primary"] {
        background-color: var(--duke-blue) !important;
        color: white !important;
        border: 2px solid var(--duke-blue) !important;
    }
    div[data-testid="stButton"] > button[kind="primary"]:hover {
        background-color: var(--duke-blue-light) !important;
    }

    /* Secondary (inactive) */
    div[data-testid="stButton"] > button[kind="secondary"] {
        background-color: var(--inactive-gray) !important;
        color: black !important;
        border: 2px solid var(--duke-blue) !important;
    }
    div[data-testid="stButton"] > button[kind="secondary"]:hover {
        background-color: #f0f0f0 !important;
    }

    .bar {
        display:flex; gap: 12px; align-items:center; justify-content:center; margin: 6px 0 12px 0;
    }
    .status-ok { color: var(--duke-blue); font-weight: 600; }
    .status-stop { color: var(--duke-red); font-weight: 600; }
    </style>
    """,
    unsafe_allow_html=True,
)

# --- Session state ---
if "proc" not in st.session_state:
    st.session_state.proc = None
if "reader_thread" not in st.session_state:
    st.session_state.reader_thread = None
if "out_queue" not in st.session_state:
    st.session_state.out_queue = queue.Queue()
if "j_toggle" not in st.session_state:
    st.session_state.j_toggle = False
if "script_path" not in st.session_state:
    st.session_state.script_path = str(DEFAULT_SCRIPT_PATH)
if "active_key" not in st.session_state:
    st.session_state.active_key = None  # '1'..'9','0'
if "pending_rerun" not in st.session_state:
    st.session_state.pending_rerun = False  # used to force immediate visual update

DESC = {
    "1": "Normal",
    "2": "Horizontal nystagmus",
    "3": "Vertical nystagmus",
    "4": "Left stroke",
    "5": "Left eyelid tremor",
    "6": "Left dysconjugate gaze",
    "7": "Right stroke",
    "8": "Right eyelid tremor",
    "9": "Right dysconjugate gaze",
    "0": "Joystick (direct, both sticks)",
}

# ---------- Subprocess helpers ----------
def _proc_running() -> bool:
    return st.session_state.proc is not None and st.session_state.proc.poll() is None

def start_process():
    script_path = Path(st.session_state.script_path)
    if not script_path.exists():
        st.error(f"Script not found: {script_path}")
        return
    if _proc_running():
        st.info("Controller already running.")
        return

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"

    st.session_state.proc = subprocess.Popen(
        [PYTHON_BIN, "-u", str(script_path)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,   # captured but not shown
        stderr=subprocess.STDOUT,
        bufsize=1,
        universal_newlines=True,
        env=env,
    )

    def _read_stdout(proc, out_q):
        # Drain to avoid pipe blockage; we don't display logs.
        try:
            for line in proc.stdout:
                out_q.put(line.rstrip("\n"))
        except Exception:
            pass

    st.session_state.out_queue = queue.Queue()
    st.session_state.reader_thread = threading.Thread(
        target=_read_stdout, args=(st.session_state.proc, st.session_state.out_queue), daemon=True
    )
    st.session_state.reader_thread.start()

def stop_process():
    if st.session_state.proc is None:
        return
    try:
        if _proc_running() and st.session_state.proc.stdin:
            try:
                st.session_state.proc.stdin.write("q\n")
                st.session_state.proc.stdin.flush()
                time.sleep(0.3)
            except Exception:
                pass
        if _proc_running():
            st.session_state.proc.terminate()
            try:
                st.session_state.proc.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                st.session_state.proc.kill()
    finally:
        st.session_state.proc = None
        st.session_state.reader_thread = None
        st.session_state.out_queue = queue.Queue()
        st.session_state.active_key = None

def _send_to_controller(base_key: str):
    """Send base_key with/without 'j' depending on checkbox; update highlight."""
    if not _proc_running() or not st.session_state.proc.stdin:
        st.error("Controller is not running.")
        return False
    try:
        send_key = base_key
        if st.session_state.j_toggle and base_key != "0":
            send_key = f"{base_key}j"
        st.session_state.proc.stdin.write(send_key + "\n")
        st.session_state.proc.stdin.flush()
        st.session_state.active_key = base_key
        return True
    except Exception as e:
        st.error(f"Failed to send '{base_key}': {e}")
        return False

# Button callback that both sends and forces immediate UI update
def on_mode_click(base_key: str):
    if _send_to_controller(base_key):
        # ensure immediate visual highlight on first click
        st.session_state.pending_rerun = True

# Checkbox callback: immediately switch between scripted <-> j realtime
def on_j_toggle_change():
    # If there's an active mode, resend it with the new j state immediately
    if st.session_state.active_key and _proc_running():
        _send_to_controller(st.session_state.active_key)
        st.session_state.pending_rerun = True

# ---------- UI ----------
st.title("Manikin Head — Touchscreen Controller")

colB, colC = st.columns([1, 1], gap="large")
with colB:
    st.button("Start", type="primary", use_container_width=True, key="btn_start", on_click=start_process)
with colC:
    st.button("Stop", use_container_width=True, key="btn_stop", on_click=stop_process)

if _proc_running():
    st.markdown("<div class='bar'><span class='status-ok'>Status: running</span></div>", unsafe_allow_html=True)
else:
    st.markdown("<div class='bar'><span class='status-stop'>Status: stopped</span></div>", unsafe_allow_html=True)

st.write("")

def option_button(col, key_char):
    label = f"{key_char} - {DESC[key_char]}"
    btype = "primary" if st.session_state.active_key == key_char else "secondary"
    # Use on_click so state updates happen during this run
    clicked = col.button(
        label,
        use_container_width=True,
        key=f"btn_{key_char}",
        help=DESC[key_char],
        type=btype,
        on_click=on_mode_click,
        args=(key_char,),
    )
    return clicked

# 3×3 grid 1..9
row1 = st.columns(3)
option_button(row1[0], "1")
option_button(row1[1], "2")
option_button(row1[2], "3")

row2 = st.columns(3)
option_button(row2[0], "4")
option_button(row2[1], "5")
option_button(row2[2], "6")

row3 = st.columns(3)
option_button(row3[0], "7")
option_button(row3[1], "8")
option_button(row3[2], "9")

# bottom row: [space] 0 [space]  (0 sits below 8)
row4 = st.columns(3)
with row4[0]:
    st.write(" ")
option_button(row4[1], "0")
with row4[2]:
    st.write(" ")

st.write("")

# Checkbox BELOW grid — immediate effect via on_change
st.checkbox(
    "Add Joystick(s) for realtime control)",
    value=st.session_state.j_toggle,
    key="j_toggle",
    help="Applies to 1–9 (sends e.g. 2j). Ignored for 0.",
    on_change=on_j_toggle_change,
)

# If we changed state that alters styling/labels this run, rerun once to show it immediately
if st.session_state.pending_rerun:
    st.session_state.pending_rerun = False
    st.rerun()
