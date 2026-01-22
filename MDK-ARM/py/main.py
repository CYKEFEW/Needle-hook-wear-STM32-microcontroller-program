"""
realtime_serial_plot.py (Enhanced)

新增功能：
✓ 字体整体增大（Tkinter + Matplotlib）
✓ 添加“保存 CSV”按钮
"""

import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
from collections import deque
from datetime import datetime
import csv

try:
    import serial
    import serial.tools.list_ports
except Exception as e:
    print("请先安装 pyserial：pip install pyserial")
    raise

import matplotlib
matplotlib.use("TkAgg")

# ---- 全局字体放大 ----
matplotlib.rcParams.update({
    "font.size": 12,
    "axes.titlesize": 14,
    "axes.labelsize": 12,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
})

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ---------- 配置 ----------
MAX_POINTS = 2000
PLOT_INTERVAL_MS = 30
AXIS_WINDOW_SEC = 10
SERIAL_TIMEOUT = 0.05
# -------------------------

class SerialPlotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("实时串口电压监测")
        self.root.geometry("1100x650")

        # ---- Tkinter 大字体 ----
        default_font = ("Microsoft YaHei", 12)
        self.root.option_add("*Font", default_font)

        # 串口控制变量
        self.ser = None
        self.serial_thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        # 数据存储
        self.start_time = None
        self.time_data = deque(maxlen=MAX_POINTS)
        self.voltage_data = deque(maxlen=MAX_POINTS)

        self.frame_count = 0

        self._build_ui()
        self._init_plot()

        self.updating = False

    # ---------------------------------------------------------
    # UI 构建
    # ---------------------------------------------------------
    def _build_ui(self):
        top_frame = ttk.Frame(self.root)
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=8)

        # 串口选择
        ttk.Label(top_frame, text="串口:").grid(row=0, column=0)
        self.port_cb = ttk.Combobox(top_frame, width=15, state="readonly")
        self.port_cb.grid(row=0, column=1, padx=4)

        ttk.Button(top_frame, text="刷新端口", command=self.refresh_ports).grid(row=0, column=2, padx=4)

        # 波特率
        ttk.Label(top_frame, text="波特率:").grid(row=0, column=3)
        self.baud_entry = ttk.Entry(top_frame, width=12)
        self.baud_entry.insert(0, "115200")
        self.baud_entry.grid(row=0, column=4, padx=4)

        # 连接 / 断开
        self.connect_btn = ttk.Button(top_frame, text="连接", width=12, command=self.connect_port)
        self.connect_btn.grid(row=0, column=5, padx=6)

        self.disconnect_btn = ttk.Button(top_frame, text="断开", width=12, command=self.disconnect_port, state=tk.DISABLED)
        self.disconnect_btn.grid(row=0, column=6, padx=4)

        # 开始 / 停止采集
        self.start_btn = ttk.Button(top_frame, text="开始采集", width=12, command=self.start_acquire, state=tk.DISABLED)
        self.start_btn.grid(row=1, column=0, pady=10)

        self.stop_btn = ttk.Button(top_frame, text="停止采集", width=12, command=self.stop_acquire, state=tk.DISABLED)
        self.stop_btn.grid(row=1, column=1, pady=10)

        # 保存按钮（新增）
        self.save_btn = ttk.Button(top_frame, text="保存 CSV", width=12, command=self.save_csv, state=tk.NORMAL)
        self.save_btn.grid(row=1, column=2, padx=4)

        # 发送命令
        send_frame = ttk.Frame(top_frame)
        send_frame.grid(row=1, column=3, columnspan=5, sticky=tk.W, padx=6)

        ttk.Label(send_frame, text="发送:").pack(side=tk.LEFT)
        self.send_entry = ttk.Entry(send_frame, width=40)
        self.send_entry.pack(side=tk.LEFT, padx=6)

        self.send_btn = ttk.Button(send_frame, text="发送并接收", width=15, command=self.send_command, state=tk.DISABLED)
        self.send_btn.pack(side=tk.LEFT, padx=4)

        # 绘图区域
        plot_frame = ttk.Frame(self.root)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig, self.ax = plt.subplots(figsize=(8, 5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.line, = self.ax.plot([], [], '-', linewidth=1.5)

        # 日志区域
        right_frame = ttk.Frame(self.root)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10)

        ttk.Label(right_frame, text="串口日志 / 回复:").pack(anchor=tk.W)
        self.log_text = scrolledtext.ScrolledText(right_frame, width=40, height=25, state=tk.DISABLED)
        self.log_text.pack(fill=tk.Y, expand=False)

        # 状态栏
        self.status_var = tk.StringVar(value="未连接")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, anchor=tk.W, relief=tk.SUNKEN)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        # 初次刷新串口
        self.refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    # ---------------------------------------------------------
    # 刷新端口
    # ---------------------------------------------------------
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        lst = [p.device for p in ports]
        self.port_cb["values"] = lst
        if lst:
            self.port_cb.current(0)

    # ---------------------------------------------------------
    # 绘图初始化
    # ---------------------------------------------------------
    def _init_plot(self):
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Voltage (V)")
        self.ax.set_title("Real-Time Voltage Monitor")
        self.ax.grid(True)
        self.canvas.draw()

    # ---------------------------------------------------------
    # 串口连接
    # ---------------------------------------------------------
    def connect_port(self):
        port = self.port_cb.get()
        if not port:
            messagebox.showwarning("警告", "请选择端口")
            return

        try:
            baud = int(self.baud_entry.get())
        except:
            messagebox.showwarning("警告", "波特率无效")
            return

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=SERIAL_TIMEOUT)
            self.status_var.set(f"已连接：{port} @ {baud}")
            self.log(f"连接成功：{port} @ {baud}")

            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.start_btn.config(state=tk.NORMAL)
            self.send_btn.config(state=tk.NORMAL)

            self.port_cb.config(state="disabled")
            self.baud_entry.config(state="disabled")

        except Exception as e:
            messagebox.showerror("错误", f"连接失败：{e}")

    def disconnect_port(self):
        self.stop_acquire()

        if self.ser:
            try:
                self.ser.close()
            except:
                pass

        self.ser = None
        self.status_var.set("已断开")

        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.start_btn.config(state=tk.DISABLED)
        self.send_btn.config(state=tk.DISABLED)

        self.port_cb.config(state="readonly")
        self.baud_entry.config(state="normal")

        self.log("已断开串口")

    # ---------------------------------------------------------
    # 开始 / 停止采集
    # ---------------------------------------------------------
    def start_acquire(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("警告", "请先连接串口")
            return

        with self.lock:
            self.time_data.clear()
            self.voltage_data.clear()
            self.start_time = time.time()

        self.stop_event.clear()
        self.serial_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.serial_thread.start()

        self.updating = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)

        self.root.after(PLOT_INTERVAL_MS, self._update_plot)

    def stop_acquire(self):
        self.stop_event.set()
        self.updating = False

        if self.serial_thread:
            self.serial_thread.join(timeout=0.5)

        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    # ---------------------------------------------------------
    # 串口读线程
    # ---------------------------------------------------------
    def _serial_read_loop(self):
        while not self.stop_event.is_set() and self.ser and self.ser.is_open:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()

                # 解析形如 {voltage:3.45}
                if line.startswith("{voltage:") and line.endswith("}"):
                    try:
                        num = line[9:-1]   # 去掉 {voltage: 和 }
                        voltage = float(num)
                    except:
                        continue

                    t = time.time() - self.start_time

                    with self.lock:
                        self.time_data.append(t)
                        self.voltage_data.append(voltage)

                else:
                    self._log_from_thread(line)

            except:
                continue

    # ---------------------------------------------------------
    # 发送命令
    # ---------------------------------------------------------
    def send_command(self):
        if not (self.ser and self.ser.is_open):
            return

        text = self.send_entry.get()
        if not text:
            return

        if not text.endswith("\n"):
            text += "\n"

        try:
            self.ser.write(text.encode())
            self.log(f"[TX] {text.strip()}")

            time.sleep(0.02)
            t0 = time.time()
            while time.time() - t0 < 0.2:
                if self.ser.in_waiting:
                    r = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if r:
                        self.log(f"[RX] {r}")
                else:
                    time.sleep(0.01)
        except Exception as e:
            self.log(f"发送失败: {e}")

    # ---------------------------------------------------------
    # 保存 CSV （新增）
    # ---------------------------------------------------------
    def save_csv(self):
        if len(self.time_data) == 0:
            messagebox.showwarning("提示", "没有数据可保存！")
            return

        filename = filedialog.asksaveasfilename(
            title="保存 CSV",
            defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")]
        )

        if not filename:
            return

        with self.lock:
            times = list(self.time_data)
            volts = list(self.voltage_data)

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "time(s)", "voltage(V)"])

            for t, v in zip(times, volts):
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                writer.writerow([ts, t, v])

        self.log(f"数据已保存到：{filename}")
        messagebox.showinfo("成功", "保存完成！")

    # ---------------------------------------------------------
    # 日志
    # ---------------------------------------------------------
    def log(self, text):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _log_from_thread(self, text):
        self.root.after(0, lambda: self.log(text))

    # ---------------------------------------------------------
    # 绘图刷新（主线程）
    # ---------------------------------------------------------
    def _update_plot(self):
        if not self.updating:
            return

        with self.lock:
            xs = list(self.time_data)
            ys = list(self.voltage_data)

        if xs:
            self.line.set_data(xs, ys)

            # 每几帧更新轴
            if self.frame_count % 5 == 0:
                t_end = xs[-1]
                self.ax.set_xlim(max(0, t_end - AXIS_WINDOW_SEC), t_end + 0.1)

                ymin = min(ys)
                ymax = max(ys)
                yr = ymax - ymin
                if yr < 0.01:
                    yr = 0.1
                self.ax.set_ylim(ymin - 0.1 * yr, ymax + 0.1 * yr)

            self.canvas.draw_idle()
            self.frame_count += 1

        self.root.after(PLOT_INTERVAL_MS, self._update_plot)

    # ---------------------------------------------------------
    # 关闭时清理
    # ---------------------------------------------------------
    def on_closing(self):
        self.stop_event.set()
        self.updating = False

        if self.serial_thread:
            self.serial_thread.join(timeout=0.5)

        if self.ser:
            try:
                self.ser.close()
            except:
                pass

        self.root.destroy()


def main():
    root = tk.Tk()
    app = SerialPlotApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
