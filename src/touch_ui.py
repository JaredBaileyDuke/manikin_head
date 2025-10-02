#!/usr/bin/env python3
# Pi A — Streamlit UI that sends modes + joystick frames to Pi B over TCP

import json, os, socket, threading, time, sys
from pathlib import Path
import streamlit as st

# -----------------------------------------------------------------------------
# Import shim so "streamlit run src/touch_ui.py" works from repo root
# -----------------------------------------------------------------------------
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

# Joystick helper (must live in the same folder)
from joystick_serial import frames as joy_frames


# ====== NETWORK CONFIG (EDIT HOST AS NEEDED) ======
PIB_HOST = "172.28.214.226"          # set to your Pi B IP/hostname
PIB_PORT = int(os.getenv("PIB_PORT", "5055"))
# ==================================================

st.set_page_config(page_title="Manikin Head Controller", layout="centered")

# ---- Styles (Duke Blue) ----
st.markdown("""
<style>
:root { --duke-blue:#001A57; --duke-blue-light:#003366; --duke-red:#990000; --inactive-gray:#e0e0e0; }
.stButton > button { height:88px; font-size:1.15rem; font-weight:700; line-height:1.2; padding:.6rem .8rem; border-radius:14px; white-space:pre-line; }
div[data-testid="stButton"] > button[kind="primary"] { background-color:var(--duke-blue)!important; color:white!important; border:2px solid var(--duke-blue)!important; }
div[data-testid="stButton"] > button[kind="primary"]:hover { background-color:var(--duke-blue-light)!important; }
div[data-testid="stButton"] > button[kind="secondary"] { background-color:var(--inactive-gray)!important; color:black!important; border:2px solid var(--duke-blue)!important; }
div[data-testid="stButton"] > button[kind="secondary"]:hover { background-color:#f0f0f0!important; }
.bar{display:flex;gap:12px;align-items:center;justify-content:center;margin:6px 0 12px 0;}
.status-ok{color:var(--duke-blue);font-weight:600;}
.status-stop{color:var(--duke-red);font-weight:600;}
</style>
""", unsafe_allow_html=True)

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

# ---- State ----
if "sock" not in st.session_state: st.session_state.sock = None
if "active_key" not in st.session_state: st.session_state.active_key = None
if "j_toggle" not in st.session_state: st.session_state.j_toggle = False
if "joy_thread" not in st.session_state: st.session_state.joy_thread = None
if "joy_stop" not in st.session_state: st.session_state.joy_stop = threading.Event()
# remembers the exact last key we sent to Pi B, e.g. "7j" vs "7"
if "last_sent_key" not in st.session_state: st.session_state.last_sent_key = None

# ---- Networking helpers ----
def _connected() -> bool:
    return st.session_state.sock is not None

def connect():
    if _connected():
        return
    try:
        addr = (PIB_HOST, PIB_PORT)
        s = socket.create_connection(addr, timeout=3.0)
        st.session_state.sock = s
    except OSError as e:
        st.error(f"Connect failed to {PIB_HOST}:{PIB_PORT} → {e}")
        st.session_state.sock = None

def disconnect():
    try:
        if st.session_state.sock:
            st.session_state.sock.close()
    except Exception:
        pass
    st.session_state.sock = None
    st.session_state.active_key = None
    st.session_state.last_sent_key = None
    _stop_joy_thread()

def _send(obj: dict) -> bool:
    if not _connected():
        st.error("Not connected to Pi B.")
        return False
    try:
        line = json.dumps(obj, separators=(",",":")) + "\n"
        st.session_state.sock.sendall(line.encode("utf-8"))
        return True
    except OSError as e:
        st.error(f"Send failed: {e}")
        disconnect()
        return False

def _send_mode(base_key: str) -> bool:
    send_key = base_key
    # “j” applies to 1–9, ignored for 0
    if st.session_state.j_toggle and base_key != "0":
        send_key = f"{base_key}j"
    ok = _send({"type":"mode", "key":send_key})
    if ok:
        st.session_state.active_key = base_key
        st.session_state.last_sent_key = send_key  # remember exact key we told Pi B
    return ok

# ---- Joystick streaming helpers ----
def _need_joystick_stream() -> bool:
    """
    Stream when:
      - active mode is "0" (both sticks), OR
      - the last key we sent ends with 'j' (Pi B is in a j-mode and expects left stick), OR
      - the checkbox is ON while on 1..9 (so next sends will be j)
    """
    if st.session_state.active_key == "0":
        return True
    if (st.session_state.last_sent_key or "").endswith("j"):
        return True
    if st.session_state.j_toggle and st.session_state.active_key in set("123456789"):
        return True
    return False

def _joy_loop(stop_evt):
    for frame in joy_frames():
        if stop_evt.is_set(): break
        if not _need_joystick_stream():
            time.sleep(0.02); continue
        frame["type"] = "joy"
        _send(frame)

def _ensure_joy_thread():
    if st.session_state.joy_thread and st.session_state.joy_thread.is_alive():
        return
    st.session_state.joy_stop = threading.Event()
    th = threading.Thread(target=_joy_loop, args=(st.session_state.joy_stop,), daemon=True)
    st.session_state.joy_thread = th
    th.start()

def _stop_joy_thread():
    if st.session_state.joy_thread and st.session_state.joy_thread.is_alive():
        st.session_state.joy_stop.set()
        st.session_state.joy_thread.join(timeout=1.0)
    st.session_state.joy_thread = None

# ---- UI callbacks ----
def on_mode_click(base_key: str):
    if not _connected():
        st.error("Not connected to Pi B.")
        return
    if _send_mode(base_key):
        if _need_joystick_stream():
            _ensure_joy_thread()
        else:
            _stop_joy_thread()

def on_j_toggle_change():
    # resend current base mode with/without 'j'
    if st.session_state.active_key and _connected():
        _send_mode(st.session_state.active_key)
    if _need_joystick_stream():
        _ensure_joy_thread()
    else:
        _stop_joy_thread()

# ---------- UI ----------
st.title("Manikin Head — Touchscreen Controller (Network)")
st.caption(f"Target Pi B: {PIB_HOST}:{PIB_PORT}")

colB, colC = st.columns([1,1], gap="large")
with colB:
    st.button("Connect", type="primary", use_container_width=True, on_click=connect)
with colC:
    st.button("Disconnect", use_container_width=True, on_click=disconnect)

if _connected():
    st.markdown("<div class='bar'><span class='status-ok'>Status: connected</span></div>", unsafe_allow_html=True)
else:
    st.markdown("<div class='bar'><span class='status-stop'>Status: disconnected</span></div>", unsafe_allow_html=True)

# small hint about joystick streaming state
st.caption(
    "🎮 Joy stream: " +
    ("ON" if (_connected() and _need_joystick_stream()) else "off") +
    (f" · last={st.session_state.last_sent_key or '-'}")
)

st.write("")

def option_button(col, key_char):
    label = f"{key_char} - {DESC[key_char]}"
    btype = "primary" if st.session_state.active_key == key_char else "secondary"
    col.button(label, use_container_width=True, type=btype, key=f"btn_{key_char}",
               on_click=on_mode_click, args=(key_char,))

# 3×3 grid 1..9
row1 = st.columns(3); option_button(row1[0], "1"); option_button(row1[1], "2"); option_button(row1[2], "3")
row2 = st.columns(3); option_button(row2[0], "4"); option_button(row2[1], "5"); option_button(row2[2], "6")
row3 = st.columns(3); option_button(row3[0], "7"); option_button(row3[1], "8"); option_button(row3[2], "9")
row4 = st.columns(3); row4[0].write(" "); option_button(row4[1], "0"); row4[2].write(" ")

st.checkbox(
    "Add Joystick(s) for realtime control",
    value=st.session_state.j_toggle,
    key="j_toggle",
    help="Applies to 1–9 (sends e.g. 2j). Ignored for 0.",
    on_change=on_j_toggle_change,
)

# Keep joystick thread aligned with state (e.g., reconnects/reruns)
if _connected() and _need_joystick_stream():
    _ensure_joy_thread()
else:
    _stop_joy_thread()
