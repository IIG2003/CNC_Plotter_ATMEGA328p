"""
plotter_studio.py

A small desktop app: load an image -> convert it to G-code sized for
this plotter (see gcode_converter.py / xy_gcode_plotter_no_servo.ino)
-> preview the resulting pen path -> stream it to the board over TTL
serial, all in one window.

Run:
    python plotter_studio.py

Requires: opencv-python-headless, numpy, pillow, pyserial (all pip
installable). Tkinter ships with standard Python on Windows.
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
import serial.tools.list_ports
from PIL import Image, ImageTk

from gcode_converter import ConversionParams, convert_image, BED_MAX_X_MM, BED_MAX_Y_MM

CANVAS_W = 620
CANVAS_H = int(CANVAS_W * BED_MAX_Y_MM / BED_MAX_X_MM)
POLL_INTERVAL = 1.0
HEARTBEAT_EVERY = 3.0


# ------------------------------------------------------------------
# Serial streaming (same handshake as send_gcode.py), but reporting
# progress through callbacks instead of print(), and abortable.
# ------------------------------------------------------------------
def wait_for_ok(ser, line_desc, max_wait, log, abort_event):
    waited = 0.0
    last_heartbeat = 0.0
    while waited < max_wait:
        if abort_event.is_set():
            return False
        response = ser.readline().decode(errors="replace").strip()
        if response:
            if "ok" in response.lower():
                return True
            log(f"<< {response}")
            waited = 0.0
            last_heartbeat = 0.0
            continue
        waited += POLL_INTERVAL
        last_heartbeat += POLL_INTERVAL
        if last_heartbeat >= HEARTBEAT_EVERY:
            log(f"   ... still waiting for '{line_desc}' ({waited:.0f}s/{max_wait:.0f}s)")
            last_heartbeat = 0.0
    return False


def stream_gcode(port, baud, lines, max_wait, log, progress, abort_event, done):
    try:
        ser = serial.Serial(port, baud, timeout=POLL_INTERVAL)
    except serial.SerialException as e:
        log(f"!! Could not open {port}: {e}")
        done(False)
        return

    time.sleep(2)
    time.sleep(2)
    while ser.in_waiting:
        log(f"<< {ser.readline().decode(errors='replace').strip()}")

    total = len(lines)
    ok_all = True
    for i, line in enumerate(lines, 1):
        if abort_event.is_set():
            log("!! Aborted by user")
            ok_all = False
            break
        ser.write((line + "\n").encode())
        log(f">> [{i}/{total}] {line}")
        if not wait_for_ok(ser, line, max_wait, log, abort_event):
            if not abort_event.is_set():
                log(f"!! No response for line {i} ('{line}') after {max_wait}s -- check connection")
            ok_all = False
            break
        progress(i, total)
    else:
        log("Done streaming.")

    ser.close()
    done(ok_all)


class PlotterStudio:
    def __init__(self, root):
        self.root = root
        root.title("Plotter Studio")

        self.image_path = None
        self.gcode_lines = None
        self.contours_mm = None
        self.abort_event = threading.Event()
        self.log_q = queue.Queue()
        self.busy = False

        self._build_ui()
        self._refresh_ports()
        self.root.after(100, self._poll_queue)

    # ---------------- UI ----------------
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Button(top, text="Load Image...", command=self.load_image).pack(side=tk.LEFT)
        ttk.Button(top, text="Load .gcode file...", command=self.load_gcode_file).pack(side=tk.LEFT, padx=4)
        self.file_label = ttk.Label(top, text="No image loaded")
        self.file_label.pack(side=tk.LEFT, padx=10)

        body = ttk.Frame(self.root, padding=6)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- left: bed/path preview canvas ---
        left = ttk.Frame(body)
        left.pack(side=tk.LEFT, padx=(0, 8))
        self.canvas = tk.Canvas(left, width=CANVAS_W, height=CANVAS_H, bg="white",
                                 highlightthickness=1, highlightbackground="#888")
        self.canvas.pack()
        self._draw_bed_outline()
        self.stats_label = ttk.Label(left, text="")
        self.stats_label.pack(anchor="w", pady=(4, 0))

        # --- right: parameters ---
        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.Y)

        bed = ttk.LabelFrame(right, text=f"Bed: {BED_MAX_X_MM:.0f} x {BED_MAX_Y_MM:.0f} mm", padding=6)
        bed.pack(fill=tk.X, pady=4)
        self.margin_var = tk.DoubleVar(value=10.0)
        self._labeled_spin(bed, "Margin (mm)", self.margin_var, 0, 100, 1)

        proc = ttk.LabelFrame(right, text="Image processing", padding=6)
        proc.pack(fill=tk.X, pady=4)
        self.mode_var = tk.StringVar(value="edges")
        mode_row = ttk.Frame(proc); mode_row.pack(fill=tk.X)
        ttk.Radiobutton(mode_row, text="Edges (photos)", variable=self.mode_var, value="edges").pack(side=tk.LEFT)
        ttk.Radiobutton(mode_row, text="Threshold (line art)", variable=self.mode_var, value="threshold").pack(side=tk.LEFT)

        self.canny_low_var = tk.IntVar(value=60)
        self.canny_high_var = tk.IntVar(value=150)
        self.threshold_var = tk.IntVar(value=128)
        self.blur_var = tk.IntVar(value=3)
        self.epsilon_var = tk.DoubleVar(value=1.5)
        self.min_len_var = tk.DoubleVar(value=6.0)
        self.invert_var = tk.BooleanVar(value=False)
        self._labeled_spin(proc, "Canny low", self.canny_low_var, 0, 255, 1)
        self._labeled_spin(proc, "Canny high", self.canny_high_var, 0, 255, 1)
        self._labeled_spin(proc, "Threshold value", self.threshold_var, 0, 255, 1)
        self._labeled_spin(proc, "Blur kernel (odd)", self.blur_var, 0, 15, 2)
        self._labeled_spin(proc, "Simplify epsilon (px)", self.epsilon_var, 0.1, 10.0, 0.1)
        self._labeled_spin(proc, "Min stroke length (px)", self.min_len_var, 0, 100, 1)
        ttk.Checkbutton(proc, text="Invert (dark background)", variable=self.invert_var).pack(anchor="w")

        orient = ttk.LabelFrame(right, text="Orientation", padding=6)
        orient.pack(fill=tk.X, pady=4)
        self.flip_x_var = tk.BooleanVar(value=False)
        self.flip_y_var = tk.BooleanVar(value=True)
        self.rotate_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(orient, text="Flip X", variable=self.flip_x_var).pack(anchor="w")
        ttk.Checkbutton(orient, text="Flip Y (on by default: image top != plotter Y=0)",
                         variable=self.flip_y_var).pack(anchor="w")
        ttk.Checkbutton(orient, text="Rotate 90 deg", variable=self.rotate_var).pack(anchor="w")

        speed = ttk.LabelFrame(right, text="Speed", padding=6)
        speed.pack(fill=tk.X, pady=4)
        self.feed_var = tk.DoubleVar(value=900.0)
        self._labeled_spin(speed, "Draw feed (mm/min)", self.feed_var, 60, 3000, 10)

        gen_row = ttk.Frame(right)
        gen_row.pack(fill=tk.X, pady=6)
        self.generate_btn = ttk.Button(gen_row, text="Generate Preview", command=self.generate_preview)
        self.generate_btn.pack(side=tk.LEFT)
        self.save_btn = ttk.Button(gen_row, text="Save G-code As...", command=self.save_gcode, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT, padx=4)

        # --- serial section ---
        ser_frame = ttk.LabelFrame(self.root, text="Send via TTL Serial", padding=6)
        ser_frame.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(0, 6))

        row1 = ttk.Frame(ser_frame); row1.pack(fill=tk.X)
        ttk.Label(row1, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(row1, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)
        ttk.Button(row1, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)

        ttk.Label(row1, text="Baud:").pack(side=tk.LEFT, padx=(10, 0))
        self.baud_var = tk.StringVar(value="9600")
        ttk.Combobox(row1, textvariable=self.baud_var, width=8, state="readonly",
                     values=["9600", "115200"]).pack(side=tk.LEFT, padx=4)

        ttk.Label(row1, text="Max wait/line (s):").pack(side=tk.LEFT, padx=(10, 0))
        self.max_wait_var = tk.IntVar(value=60)
        ttk.Spinbox(row1, from_=5, to=300, textvariable=self.max_wait_var, width=5).pack(side=tk.LEFT, padx=4)

        self.send_btn = ttk.Button(row1, text="Send to Plotter", command=self.send_to_plotter)
        self.send_btn.pack(side=tk.LEFT, padx=(10, 4))
        self.stop_btn = ttk.Button(row1, text="Stop", command=self.stop_streaming, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT)

        self.progress = ttk.Progressbar(ser_frame, mode="determinate")
        self.progress.pack(fill=tk.X, pady=(6, 0))

        log_frame = ttk.Frame(ser_frame)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        self.log_text = tk.Text(log_frame, height=8, font=("Consolas", 9), state=tk.DISABLED)
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.LEFT, fill=tk.Y)

    def _labeled_spin(self, parent, label, var, lo, hi, step):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=1)
        ttk.Label(row, text=label, width=22).pack(side=tk.LEFT)
        ttk.Spinbox(row, from_=lo, to=hi, increment=step, textvariable=var, width=8).pack(side=tk.LEFT)

    # ---------------- bed / preview drawing ----------------
    def _draw_bed_outline(self):
        self.canvas.delete("bed")
        self.canvas.create_rectangle(1, 1, CANVAS_W - 1, CANVAS_H - 1, outline="#888", tags="bed")

    def _mm_to_canvas(self, x_mm, y_mm):
        px = x_mm / BED_MAX_X_MM * CANVAS_W
        py = CANVAS_H - (y_mm / BED_MAX_Y_MM * CANVAS_H)
        return px, py

    def _draw_paths(self, contours_mm):
        self.canvas.delete("path")
        for c in contours_mm:
            if len(c) < 2:
                continue
            pts = []
            for (x, y) in c:
                pts.extend(self._mm_to_canvas(x, y))
            self.canvas.create_line(*pts, fill="#1a1a1a", width=1, tags="path")

    # ---------------- image / gcode loading ----------------
    def load_image(self):
        path = filedialog.askopenfilename(
            filetypes=[("Images", "*.png *.jpg *.jpeg *.bmp *.gif *.tif *.tiff")])
        if not path:
            return
        self.image_path = path
        self.gcode_lines = None
        self.contours_mm = None
        self.save_btn.configure(state=tk.DISABLED)
        self.file_label.configure(text=path)
        self._draw_bed_outline()
        self.canvas.delete("path")
        self.stats_label.configure(text="")

    def load_gcode_file(self):
        path = filedialog.askopenfilename(filetypes=[("G-code", "*.gcode *.nc *.txt"), ("All files", "*.*")])
        if not path:
            return
        with open(path) as f:
            self.gcode_lines = [ln.strip() for ln in f if ln.strip()]
        self.contours_mm = None
        self.image_path = None
        self.file_label.configure(text=f"{path}  ({len(self.gcode_lines)} lines, ready to send)")
        self._draw_bed_outline()
        self.canvas.delete("path")
        self.save_btn.configure(state=tk.NORMAL)
        self.stats_label.configure(text=f"{len(self.gcode_lines)} G-code lines loaded from file")

    def _gather_params(self):
        p = ConversionParams()
        p.margin_mm = self.margin_var.get()
        p.mode = self.mode_var.get()
        p.canny_low = self.canny_low_var.get()
        p.canny_high = self.canny_high_var.get()
        p.threshold_value = self.threshold_var.get()
        p.blur_ksize = self.blur_var.get()
        p.simplify_epsilon_px = self.epsilon_var.get()
        p.min_contour_len_px = self.min_len_var.get()
        p.invert = self.invert_var.get()
        p.flip_x = self.flip_x_var.get()
        p.flip_y = self.flip_y_var.get()
        p.rotate_90 = self.rotate_var.get()
        p.draw_feed_mm_min = self.feed_var.get()
        return p

    def generate_preview(self):
        if not self.image_path:
            messagebox.showwarning("No image", "Load an image first.")
            return
        if self.busy:
            return
        params = self._gather_params()
        self._set_busy(True, "Generating...")
        threading.Thread(target=self._generate_worker, args=(self.image_path, params), daemon=True).start()

    def _generate_worker(self, path, params):
        try:
            result = convert_image(path, params)
            self.log_q.put(("generated", result))
        except Exception as e:
            self.log_q.put(("error", str(e)))

    def save_gcode(self):
        if not self.gcode_lines:
            return
        path = filedialog.asksaveasfilename(defaultextension=".gcode",
                                             filetypes=[("G-code", "*.gcode")])
        if not path:
            return
        with open(path, "w") as f:
            f.write("\n".join(self.gcode_lines) + "\n")
        self._log(f"Saved {path}")

    # ---------------- serial streaming ----------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def send_to_plotter(self):
        if not self.gcode_lines:
            messagebox.showwarning("Nothing to send", "Generate or load G-code first.")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a serial port.")
            return
        if self.busy:
            return

        confirm = messagebox.askyesno(
            "Confirm send",
            f"Send {len(self.gcode_lines)} G-code lines to the plotter on {port}?\n"
            "Make sure the bed is clear and the machine is ready.")
        if not confirm:
            return

        self.abort_event.clear()
        self._set_busy(True, "Streaming...")
        self.stop_btn.configure(state=tk.NORMAL)
        self.progress.configure(maximum=len(self.gcode_lines), value=0)

        baud = int(self.baud_var.get())
        max_wait = self.max_wait_var.get()
        threading.Thread(
            target=stream_gcode,
            args=(port, baud, list(self.gcode_lines), max_wait,
                  lambda msg: self.log_q.put(("log", msg)),
                  lambda i, total: self.log_q.put(("progress", i, total)),
                  self.abort_event,
                  lambda ok: self.log_q.put(("stream_done", ok))),
            daemon=True,
        ).start()

    def stop_streaming(self):
        self.abort_event.set()

    # ---------------- queue / busy state plumbing ----------------
    def _set_busy(self, busy, status=""):
        self.busy = busy
        state = tk.DISABLED if busy else tk.NORMAL
        self.generate_btn.configure(state=state)
        self.send_btn.configure(state=state)
        if not busy:
            self.stop_btn.configure(state=tk.DISABLED)
        if status:
            self._log(status)

    def _log(self, msg):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _poll_queue(self):
        try:
            while True:
                item = self.log_q.get_nowait()
                kind = item[0]
                if kind == "log":
                    self._log(item[1])
                elif kind == "progress":
                    self.progress.configure(value=item[1])
                elif kind == "generated":
                    result = item[1]
                    self.gcode_lines = result.gcode_lines
                    self.contours_mm = result.contours_mm
                    self._draw_bed_outline()
                    self._draw_paths(result.contours_mm)
                    s = result.stats
                    self.stats_label.configure(
                        text=f"{s['contours']} strokes, {s['points']} points, "
                             f"{s['gcode_lines']} G-code lines, "
                             f"~{s['est_seconds']/60:.1f} min estimated")
                    self.save_btn.configure(state=tk.NORMAL)
                    self._set_busy(False, "Preview generated.")
                elif kind == "error":
                    messagebox.showerror("Conversion failed", item[1])
                    self._set_busy(False, f"Error: {item[1]}")
                elif kind == "stream_done":
                    ok = item[1]
                    self._set_busy(False, "Streaming finished." if ok else "Streaming stopped/failed.")
        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)


def main():
    root = tk.Tk()
    PlotterStudio(root)
    root.mainloop()


if __name__ == "__main__":
    main()
