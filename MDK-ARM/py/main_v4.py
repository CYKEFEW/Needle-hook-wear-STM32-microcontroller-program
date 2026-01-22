import threading
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, filedialog
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import csv
from datetime import datetime
import sys  # 用于退出程序


class SerialPlotApp:
    def __init__(self, root):
        self.root = root
        root.title("Serial Real-Time Plotter")

        # ==================== 窗口关闭事件 ====================
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ==================== 字体 ====================
        self.font_big = ("Microsoft YaHei", 20)
        self.font_reply = ("Microsoft YaHei", 20)

        # ==================== 按钮和Combobox样式 ====================
        style = ttk.Style()
        style.configure("Big.TButton", font=self.font_big, padding=(6, 6))
        style.configure("Big.TCombobox", font=self.font_big)

        # ==================== 数据相关 ====================
        self.data = []  # 存储所有数据，格式为 [[voltage1], [voltage2], ...]
        self.timestamps = []  # 存储时间戳
        self.window_size = 300
        self.min_window = 50
        self.max_window = 6000
        self.max_reply_lines = 200

        self.running = False
        self.ser = None

        # 存储每条曲线的数据
        self.lines = []

        # ==================== UI ====================
        self.build_ui()

        # ==================== Matplotlib 图 ====================
        self.fig, self.ax = plt.subplots(figsize=(9, 5), dpi=100)
        self.ax.set_title("Voltage", fontsize=28)
        self.ax.set_xlabel("Samples", fontsize=24)
        self.ax.set_ylabel("Voltage (V)", fontsize=24)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.mpl_connect("scroll_event", self.on_scroll)

        # 高频刷新
        self.update_plot()
        self.update_status_label()

    # ============================================================
    #                       UI 构建
    # ============================================================
    def build_ui(self):
        # 顶部操作栏
        top = ttk.Frame(self.root)
        top.pack(side="top", fill="x", pady=10)

        ttk.Label(top, text="端口: ", font=self.font_big).pack(side="left")

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            top, textvariable=self.port_var, width=10, style="Big.TCombobox"
        )
        self.port_combo.pack(side="left", padx=10)
        self.refresh_ports()

        # 按钮
        ttk.Button(top, text="刷新端口", command=self.refresh_ports, width=20, style="Big.TButton").pack(side="left", padx=10)
        ttk.Button(top, text="连接", command=self.connect_port, width=20, style="Big.TButton").pack(side="left", padx=10)
        ttk.Button(top, text="断开", command=self.disconnect_port, width=20, style="Big.TButton").pack(side="left", padx=10)
        ttk.Button(top, text="开始采集", command=self.start, width=20, style="Big.TButton").pack(side="left", padx=10)
        ttk.Button(top, text="停止采集", command=self.stop, width=20, style="Big.TButton").pack(side="left", padx=10)
        ttk.Button(top, text="保存CSV", command=self.save_csv, width=20, style="Big.TButton").pack(side="left", padx=10)

        # 发送框
        send_frame = ttk.Frame(self.root)
        send_frame.pack(side="top", fill="x", pady=10)

        ttk.Label(send_frame, text="发送：", font=self.font_big).pack(side="left")
        self.send_entry = tk.Entry(send_frame, font=self.font_big, width=15)
        self.send_entry.pack(side="left", padx=10)
        ttk.Button(send_frame, text="发送", command=self.send_data, width=6, style="Big.TButton").pack(side="left")

        # 主区域布局：左图右信息
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True)

        # 左侧图形
        self.plot_frame = ttk.Frame(main_frame)
        self.plot_frame.pack(side="left", fill="both", expand=True)

        # 右侧回复区
        reply_frame = ttk.Frame(main_frame)
        reply_frame.pack(side="right", fill="y", padx=10, pady=10)

        ttk.Label(reply_frame, text="回复信息：", font=self.font_big).pack(anchor="w")

        self.reply_text = tk.Text(
            reply_frame, width=20, height=18,
            font=self.font_reply, wrap="word"
        )
        self.reply_text.pack(fill="y")

        # 右下角连接状态
        self.status_label = ttk.Label(
            reply_frame, text="未连接",
            font=self.font_big,
            foreground="red"
        )
        self.status_label.pack(anchor="s", pady=20)

    # ============================================================
    #                       串口相关
    # ============================================================
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def connect_port(self):
        try:
            self.ser = serial.Serial(self.port_var.get(), 115200, timeout=0.02)
        except Exception:
            pass  # 不弹窗

    def disconnect_port(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ============================================================
    #                       发送 + 回复
    # ============================================================
    def send_data(self):
        if not self.ser or not self.ser.is_open:
            return

        msg = self.send_entry.get().encode()
        msg += b"\r\n"
        print(f"发送: {msg}")
        self.ser.write(msg)

        time.sleep(0.05)
        recv = self.ser.read_all().decode(errors="ignore")

        if recv.strip():
            self.append_reply(recv)

    def append_reply(self, text):
        self.reply_text.insert("end", text + "\n")
        self.reply_text.see("end")

        lines = int(self.reply_text.index('end-1c').split('.')[0])
        if lines > self.max_reply_lines:
            self.reply_text.delete("1.0", f"{lines - self.max_reply_lines}.0")

    # ============================================================
    #                       串口采集
    # ============================================================
    def start(self):
        if not self.ser or not self.ser.is_open:
            return
        if self.running:
            return
        self.running = True
        threading.Thread(target=self.read_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def read_loop(self):
        buffer = ""
        while self.running:
            try:
                raw = self.ser.read(self.ser.in_waiting or 1).decode(errors="ignore")
                if raw:
                    buffer += raw
                    if "}" in buffer:
                        while "{" in buffer and "}" in buffer:
                            s = buffer.find("{")
                            e = buffer.find("}")
                            packet = buffer[s+1:e]
                            buffer = buffer[e+1:]

                            if "F/g:" in packet:
                                try:
                                    # 提取电压数据
                                    packet = packet.split(":")[1]  # 提取出 voltage 后面的数据
                                    voltage_data = packet.strip().split(",")  # 按逗号分隔
                                    voltage_values = [float(v.strip()) for v in voltage_data]  # 转换为浮动数值

                                    # 对每个电压值创建一条独立的曲线
                                    if len(self.data) == 0:
                                        # 初始化每个电压值的数据列表
                                        self.data = [[] for _ in range(len(voltage_values))]

                                    # 将电压数据分别加入对应的曲线
                                    for i, voltage in enumerate(voltage_values):
                                        self.data[i].append(voltage)
                                        self.timestamps.append(datetime.now())

                                    # 保证每条数据的列表不会超过最大窗口
                                    for i in range(len(self.data)):
                                        if len(self.data[i]) > self.max_window:
                                            self.data[i] = self.data[i][-self.max_window:]

                                except Exception as e:
                                    print(f"解析错误: {e}")
            except Exception as e:
                pass
            time.sleep(0.001)

    # ============================================================
    #                       滚轮缩放
    # ============================================================
    def on_scroll(self, event):
        if event.button == "up":
            self.window_size = max(self.min_window, int(self.window_size * 0.8))
        elif event.button == "down":
            self.window_size = min(self.max_window, int(self.window_size * 1.25))

    # ============================================================
    #                       图更新
    # ============================================================
    def update_plot(self):
        if len(self.data) > 0:
            show_y = [d[-self.window_size:] for d in self.data]  # 获取每条曲线的数据

            # 初始化每条线条
            if len(self.lines) == 0:
                for _ in range(len(self.data)):
                    line, = self.ax.plot([], [], lw=3)
                    self.lines.append(line)

            # 更新每一条曲线的数据
            for i, line in enumerate(self.lines):
                if i < len(show_y):
                    line.set_data(range(len(show_y[i])), show_y[i])

            self.ax.set_xlim(0, len(show_y[0]))  # 设置x轴的显示范围
            self.ax.set_ylim(min(min(show_y)) - 0.1, max(max(show_y)) + 0.1)  # 设置y轴的显示范围

        self.canvas.draw_idle()  # 更新画布
        self.root.after(10, self.update_plot)  # 定期更新图形

    # ============================================================
    #                       状态栏更新
    # ============================================================
    def update_status_label(self):
        if self.ser and self.ser.is_open:
            self.status_label.config(
                text=f"已连接: {self.ser.port}",
                foreground="green"
            )
        else:
            self.status_label.config(
                text="未连接",
                foreground="red"
            )
        self.root.after(200, self.update_status_label)

    # ============================================================
    #                       保存 CSV
    # ============================================================
    def save_csv(self):
        if not self.data:
            return

        file = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")]
        )
        if not file:
            return

        with open(file, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "voltage1", "voltage2"])  # 根据电压数量扩展列
            for t, *voltages in zip(self.timestamps, *self.data):
                writer.writerow([t.strftime("%Y-%m-%d %H:%M:%S.%f"), *voltages])

    # ============================================================
    #                   窗口关闭逻辑
    # ============================================================
    def on_close(self):
        # 停止后台线程
        self.running = False

        # 关闭串口
        if self.ser and self.ser.is_open:
            self.ser.close()

        # 销毁窗口并退出程序
        self.root.destroy()
        sys.exit(0)


# ============================================================
#                          主程序
# ============================================================
root = tk.Tk()
app = SerialPlotApp(root)
root.mainloop()
