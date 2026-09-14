"""ROV 数据集抽帧桌面工具：拖入视频，导出全部或均匀选取的帧。"""

from __future__ import annotations

import argparse
import base64
import os
import queue
import subprocess
import sys
import threading
from pathlib import Path
from urllib.parse import unquote, urlparse

import cv2
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .frame_extractor import (
    ExtractionOptions,
    ExtractionProgress,
    ExtractionResult,
    FrameExtractionError,
    VideoInfo,
    available_disk_bytes,
    default_output_directory,
    estimate_output_bytes,
    extract_all_frames,
    probe_video,
    uniform_frame_indices,
)

try:
    from tkinterdnd2 import DND_FILES, TkinterDnD
except ImportError:  # 点击选择仍可用，安装脚本会正常安装拖放依赖。
    DND_FILES = None
    TkinterDnD = None


SUPPORTED_VIDEO_SUFFIXES = {
    ".mp4",
    ".mkv",
    ".avi",
    ".mov",
    ".m4v",
    ".webm",
    ".mts",
    ".m2ts",
    ".ts",
}


def _format_bytes(value: int | None) -> str:
    """把字节数格式化成适合界面阅读的文字。"""

    if value is None:
        return "未知"
    number = float(value)
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if number < 1024.0 or unit == "TiB":
            return f"{number:.1f} {unit}"
        number /= 1024.0
    return f"{number:.1f} TiB"


def _format_duration(seconds: float | None) -> str:
    """把秒数格式化为小时、分钟和秒。"""

    if seconds is None or seconds < 0:
        return "未知"
    total = int(round(seconds))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    if hours:
        return f"{hours:d}:{minutes:02d}:{secs:02d}"
    return f"{minutes:02d}:{secs:02d}"


def _normalise_dropped_path(raw_path: str) -> Path:
    """兼容桌面文件管理器给出的普通路径和 file:// URI。"""

    value = raw_path.strip()
    if value.startswith("file://"):
        parsed = urlparse(value)
        value = unquote(parsed.path)
    return Path(value).expanduser()


def _open_directory(path: Path) -> None:
    """使用当前操作系统的文件管理器打开输出目录。"""

    if sys.platform == "darwin":
        subprocess.Popen(["open", str(path)])
    elif os.name == "nt":
        os.startfile(str(path))  # type: ignore[attr-defined]
    else:
        subprocess.Popen(["xdg-open", str(path)])


class FrameExtractorApp:
    """只负责界面；耗时解码和写盘全部放在后台线程。"""

    def __init__(self, root: tk.Tk, *, project_directory: Path) -> None:
        self.root = root
        self.project_directory = project_directory.expanduser().resolve()
        self.default_output_root = (
            self.project_directory / "output" / "extracted_frames"
        )
        self.video_info: VideoInfo | None = None
        self.estimated_bytes: int | None = None
        self.worker: threading.Thread | None = None
        self.cancel_event = threading.Event()
        self.events: queue.Queue[
            tuple[str, ExtractionProgress | ExtractionResult | BaseException]
        ] = queue.Queue()
        self.close_when_done = False
        self.preview_image: tk.PhotoImage | None = None

        self.video_var = tk.StringVar()
        self.output_var = tk.StringVar()
        self.frame_count_var = tk.StringVar(value="0")
        self.selection_var = tk.StringVar(value="0 = 导出全部帧")
        self.format_var = tk.StringVar(value="jpg")
        self.quality_var = tk.IntVar(value=95)
        self.info_var = tk.StringVar(value="尚未选择视频")
        self.estimate_var = tk.StringVar(value="拖入视频后显示预计输出大小")
        self.status_var = tk.StringVar(value="等待视频")
        self.progress_var = tk.DoubleVar(value=0.0)

        self._build_window()
        self._build_layout()
        self.frame_count_var.trace_add("write", self._on_frame_count_changed)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._poll_worker_events)

    def _build_window(self) -> None:
        """设置适合 1080p 屏幕的窗口和基础风格。"""

        self.root.title("ROV 数据集抽帧工具")
        self.root.geometry("880x760")
        self.root.minsize(760, 680)
        self.root.configure(background="#eef3f7")
        # Tcl 字体字符串中的族名含空格时必须用花括号包住，否则会把
        # ``Sans`` 误解析成字号。在没有该字体的系统上 Tk 会自动回退。
        self.root.option_add("*Font", "{Noto Sans CJK SC} 11")

        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("TFrame", background="#eef3f7")
        style.configure("Card.TFrame", background="#ffffff")
        style.configure(
            "Title.TLabel",
            background="#eef3f7",
            foreground="#15324b",
            font=("Noto Sans CJK SC", 22, "bold"),
        )
        style.configure(
            "Subtitle.TLabel",
            background="#eef3f7",
            foreground="#52697c",
        )
        style.configure(
            "CardTitle.TLabel",
            background="#ffffff",
            foreground="#19384f",
            font=("Noto Sans CJK SC", 12, "bold"),
        )
        style.configure("Card.TLabel", background="#ffffff", foreground="#334e62")
        style.configure("Accent.TButton", font=("Noto Sans CJK SC", 11, "bold"))

    def _build_layout(self) -> None:
        """创建拖放区、参数区、进度区和操作按钮。"""

        outer = ttk.Frame(self.root, padding=(24, 20, 24, 20))
        outer.pack(fill=tk.BOTH, expand=True)

        ttk.Label(outer, text="视频抽帧", style="Title.TLabel").pack(anchor=tk.W)
        ttk.Label(
            outer,
            text="拖入一个视频，导出全部帧或在整段视频中均匀取样。",
            style="Subtitle.TLabel",
        ).pack(anchor=tk.W, pady=(2, 16))

        card = ttk.Frame(outer, style="Card.TFrame", padding=18)
        card.pack(fill=tk.BOTH, expand=True)
        card.columnconfigure(0, weight=3)
        card.columnconfigure(1, weight=2)

        self.drop_zone = tk.Label(
            card,
            text="把视频拖到这里\n\n或点击选择视频",
            bg="#e7f4fb",
            fg="#17678c",
            activebackground="#d5edf9",
            activeforeground="#0f5878",
            relief=tk.GROOVE,
            borderwidth=2,
            cursor="hand2",
            font=("Noto Sans CJK SC", 15, "bold"),
            padx=20,
            pady=34,
        )
        self.drop_zone.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        self.drop_zone.bind("<Button-1>", lambda _event: self._choose_video())

        if DND_FILES is not None and hasattr(self.drop_zone, "drop_target_register"):
            self.drop_zone.drop_target_register(DND_FILES)
            self.drop_zone.dnd_bind("<<Drop>>", self._on_drop)
        else:
            self.drop_zone.configure(
                text="点击选择视频\n\n当前环境未启用拖放，选择功能仍可使用"
            )

        preview_frame = ttk.Frame(card, style="Card.TFrame")
        preview_frame.grid(row=0, column=1, sticky="nsew")
        self.preview_label = tk.Label(
            preview_frame,
            text="视频预览",
            bg="#1d2c38",
            fg="#b9c8d3",
            width=34,
            height=12,
        )
        self.preview_label.pack(fill=tk.BOTH, expand=True)

        ttk.Label(card, text="视频", style="CardTitle.TLabel").grid(
            row=1, column=0, columnspan=2, sticky=tk.W, pady=(16, 4)
        )
        video_entry = ttk.Entry(card, textvariable=self.video_var, state="readonly")
        video_entry.grid(row=2, column=0, columnspan=2, sticky="ew")
        ttk.Label(card, textvariable=self.info_var, style="Card.TLabel").grid(
            row=3, column=0, columnspan=2, sticky=tk.W, pady=(5, 10)
        )

        ttk.Label(card, text="输出目录", style="CardTitle.TLabel").grid(
            row=4, column=0, columnspan=2, sticky=tk.W, pady=(2, 4)
        )
        output_row = ttk.Frame(card, style="Card.TFrame")
        output_row.grid(row=5, column=0, columnspan=2, sticky="ew")
        output_row.columnconfigure(0, weight=1)
        self.output_entry = ttk.Entry(output_row, textvariable=self.output_var)
        self.output_entry.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        self.output_button = ttk.Button(
            output_row, text="选择位置", command=self._choose_output_root
        )
        self.output_button.grid(row=0, column=1)

        count_row = ttk.Frame(card, style="Card.TFrame")
        count_row.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(14, 2))
        ttk.Label(count_row, text="输出帧数：", style="Card.TLabel").pack(
            side=tk.LEFT
        )
        self.frame_count_spinbox = ttk.Spinbox(
            count_row,
            from_=0,
            to=0,
            increment=1,
            textvariable=self.frame_count_var,
            width=12,
            state=tk.DISABLED,
        )
        self.frame_count_spinbox.pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(
            count_row,
            text="0 = 全部；正数 = 在整段视频中均匀选取",
            style="Card.TLabel",
        ).pack(side=tk.LEFT)

        ttk.Label(card, textvariable=self.selection_var, style="Card.TLabel").grid(
            row=7, column=0, columnspan=2, sticky=tk.W, pady=(3, 2)
        )

        settings = ttk.Frame(card, style="Card.TFrame")
        settings.grid(row=8, column=0, columnspan=2, sticky="ew", pady=(8, 2))
        ttk.Label(settings, text="图片格式：", style="Card.TLabel").pack(
            side=tk.LEFT
        )
        ttk.Radiobutton(
            settings,
            text="JPG（推荐，体积较小）",
            value="jpg",
            variable=self.format_var,
            command=self._on_format_changed,
        ).pack(side=tk.LEFT, padx=(0, 14))
        ttk.Radiobutton(
            settings,
            text="PNG（无损，体积很大）",
            value="png",
            variable=self.format_var,
            command=self._on_format_changed,
        ).pack(side=tk.LEFT)

        quality_row = ttk.Frame(card, style="Card.TFrame")
        quality_row.grid(row=9, column=0, columnspan=2, sticky="ew", pady=(8, 2))
        ttk.Label(quality_row, text="JPG 质量：", style="Card.TLabel").pack(
            side=tk.LEFT
        )
        self.quality_scale = ttk.Scale(
            quality_row,
            from_=70,
            to=100,
            variable=self.quality_var,
            orient=tk.HORIZONTAL,
            command=self._on_quality_changed,
            length=250,
        )
        self.quality_scale.pack(side=tk.LEFT, padx=(4, 8))
        self.quality_label = ttk.Label(
            quality_row, text="95", style="Card.TLabel", width=4
        )
        self.quality_label.pack(side=tk.LEFT)

        ttk.Label(card, textvariable=self.estimate_var, style="Card.TLabel").grid(
            row=10, column=0, columnspan=2, sticky=tk.W, pady=(8, 8)
        )

        self.progress = ttk.Progressbar(
            card,
            variable=self.progress_var,
            maximum=100.0,
            mode="determinate",
        )
        self.progress.grid(row=11, column=0, columnspan=2, sticky="ew", pady=(4, 5))
        ttk.Label(card, textvariable=self.status_var, style="Card.TLabel").grid(
            row=12, column=0, columnspan=2, sticky=tk.W
        )

        buttons = ttk.Frame(card, style="Card.TFrame")
        buttons.grid(row=13, column=0, columnspan=2, sticky="ew", pady=(16, 0))
        self.start_button = ttk.Button(
            buttons,
            text="开始导出全部帧",
            style="Accent.TButton",
            command=self._start_extraction,
            state=tk.DISABLED,
        )
        self.start_button.pack(side=tk.LEFT)
        self.cancel_button = ttk.Button(
            buttons,
            text="取消",
            command=self._cancel_extraction,
            state=tk.DISABLED,
        )
        self.cancel_button.pack(side=tk.LEFT, padx=8)
        self.open_button = ttk.Button(
            buttons,
            text="打开输出目录",
            command=self._open_output,
            state=tk.DISABLED,
        )
        self.open_button.pack(side=tk.RIGHT)

    def _choose_video(self) -> None:
        """打开文件选择器并加载一个视频。"""

        path = filedialog.askopenfilename(
            title="选择要抽帧的视频",
            filetypes=(
                ("常见视频", "*.mp4 *.mkv *.avi *.mov *.m4v *.webm *.mts *.m2ts *.ts"),
                ("所有文件", "*.*"),
            ),
        )
        if path:
            self._load_video(Path(path))

    def _on_drop(self, event) -> None:
        """解析原生拖放事件，正确处理包含空格的路径。"""

        paths = self.root.tk.splitlist(event.data)
        if not paths:
            return
        if len(paths) > 1:
            messagebox.showinfo("一次一个视频", "请一次只拖入一个视频文件。")
            return
        self._load_video(_normalise_dropped_path(paths[0]))

    def _load_video(self, path: Path) -> None:
        """验证视频、生成预览和默认输出目录。"""

        if self.worker is not None and self.worker.is_alive():
            messagebox.showwarning("正在导出", "请先完成或取消当前任务。")
            return
        resolved = path.expanduser().resolve()
        if resolved.suffix.lower() not in SUPPORTED_VIDEO_SUFFIXES:
            if not messagebox.askyesno(
                "扩展名不常见",
                "这个文件的扩展名不常见。仍然尝试使用 OpenCV 打开吗？",
            ):
                return
        try:
            info = probe_video(resolved)
        except FrameExtractionError as exc:
            messagebox.showerror("无法读取视频", str(exc))
            return

        self.video_info = info
        self.video_var.set(str(info.path))
        self.output_var.set(
            str(default_output_directory(info.path, self.default_output_root))
        )
        # 每次换视频都回到最安全、最容易理解的默认值。
        self.frame_count_var.set("0")
        if info.total_frames > 0:
            self.frame_count_spinbox.configure(to=info.total_frames)
            self.frame_count_spinbox.state(["!disabled"])
        else:
            self.frame_count_spinbox.configure(to=0)
            self.frame_count_spinbox.state(["disabled"])
        fps_text = f"{info.fps:.3f}" if info.fps > 0 else "未知"
        frame_text = f"{info.total_frames:,}" if info.total_frames > 0 else "未知"
        self.info_var.set(
            f"{info.width}×{info.height}  |  {fps_text} FPS  |  "
            f"{frame_text} 帧  |  {_format_duration(info.duration_s)}  |  "
            f"源文件 {_format_bytes(info.file_size_bytes)}"
        )
        self._update_preview(info.path)
        self._update_selection_text()
        self._refresh_estimate()
        self.status_var.set("视频已就绪；可导出全部帧，也可输入数量均匀取样。")
        self.start_button.configure(state=tk.NORMAL)
        self.open_button.configure(state=tk.DISABLED)

    def _update_preview(self, path: Path) -> None:
        """读取一张缩略图；失败不影响正式抽帧。"""

        capture = cv2.VideoCapture(str(path))
        try:
            ok, frame = capture.read()
        finally:
            capture.release()
        if not ok or frame is None:
            self.preview_label.configure(image="", text="无法生成预览")
            return
        height, width = frame.shape[:2]
        scale = min(360 / width, 220 / height, 1.0)
        preview = cv2.resize(
            frame,
            (max(1, int(width * scale)), max(1, int(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
        ok, encoded = cv2.imencode(".png", preview)
        if not ok:
            return
        data = base64.b64encode(encoded.tobytes()).decode("ascii")
        self.preview_image = tk.PhotoImage(data=data)
        self.preview_label.configure(image=self.preview_image, text="")

    def _choose_output_root(self) -> None:
        """选择父目录，再自动创建本次独立子目录。"""

        if self.video_info is None:
            messagebox.showinfo("先选视频", "请先拖入或选择一个视频。")
            return
        root = filedialog.askdirectory(
            title="选择输出位置",
            initialdir=str(self.default_output_root.parent),
        )
        if root:
            self.output_var.set(
                str(default_output_directory(self.video_info.path, Path(root)))
            )
            self._refresh_estimate()

    def _on_format_changed(self) -> None:
        """切换格式时更新质量控件和容量估算。"""

        if self.format_var.get() == "jpg":
            self.quality_scale.state(["!disabled"])
        else:
            self.quality_scale.state(["disabled"])
        self._refresh_estimate()

    def _requested_frame_count(self, *, show_error: bool = False) -> int | None:
        """读取数量输入；编辑中的空文本返回 ``None``。"""

        text = self.frame_count_var.get().strip()
        error = ""
        if not text:
            error = "请输入 0 或一个正整数"
        else:
            try:
                requested = int(text)
            except ValueError:
                error = "输出帧数必须是整数；0 表示全部帧"
            else:
                if requested < 0:
                    error = "输出帧数不能小于 0"
                elif self.video_info is not None:
                    maximum = self.video_info.total_frames
                    if maximum <= 0 and requested != 0:
                        error = "视频没有可靠的总帧数，只能输入 0 导出全部"
                    elif maximum > 0 and requested > maximum:
                        error = f"最多只能输出 {maximum:,} 帧"
                    else:
                        return requested
                else:
                    return requested

        if show_error and error:
            messagebox.showerror("输出帧数无效", error)
        return None

    def _on_frame_count_changed(self, *_args: object) -> None:
        """数量改变时立即更新分配解释、按钮和容量估算。"""

        if not hasattr(self, "start_button"):
            return
        self._update_selection_text()
        self._refresh_estimate()

    def _update_selection_text(self) -> None:
        """用小白能直接核对的文字说明抽样分布。"""

        if self.video_info is None:
            self.selection_var.set("0 = 导出全部帧")
            self.start_button.configure(state=tk.DISABLED)
            return
        requested = self._requested_frame_count()
        if requested is None:
            self.selection_var.set("输入无效：请填 0 到视频最大帧数")
            self.start_button.configure(state=tk.DISABLED)
            return

        total = self.video_info.total_frames
        if requested == 0:
            if total > 0:
                self.selection_var.set(f"将导出全部 {total:,} 帧")
            else:
                self.selection_var.set("将从开头连续解码，直到视频结束")
            button_text = "开始导出全部帧"
        elif requested == total:
            self.selection_var.set(f"将导出全部 {total:,} 帧")
            button_text = f"开始导出 {requested:,} 帧"
        elif requested == 1:
            source_index = tuple(uniform_frame_indices(total, requested))[0]
            self.selection_var.set(
                f"取视频中间的 1 帧（源帧序号 {source_index}）"
            )
            button_text = "开始导出 1 帧"
        else:
            indices = tuple(uniform_frame_indices(total, requested))
            if requested <= 8:
                index_text = "、".join(str(index) for index in indices)
                self.selection_var.set(
                    f"均匀取 {requested:,} 帧；源帧序号：{index_text}"
                )
            else:
                self.selection_var.set(
                    f"在源帧 0 到 {total - 1:,} 之间均匀取 "
                    f"{requested:,} 帧（包含首尾）"
                )
            button_text = f"开始导出 {requested:,} 帧"

        self.start_button.configure(text=button_text)
        running = self.worker is not None and self.worker.is_alive()
        if not running:
            self.start_button.configure(state=tk.NORMAL)

    def _on_quality_changed(self, value: str) -> None:
        """显示整数质量；拖动结束前只做轻量防抖。"""

        quality = int(round(float(value)))
        self.quality_var.set(quality)
        self.quality_label.configure(text=str(quality))
        if hasattr(self, "_estimate_after_id"):
            self.root.after_cancel(self._estimate_after_id)
        self._estimate_after_id = self.root.after(250, self._refresh_estimate)

    def _refresh_estimate(self) -> None:
        """估算输出占用并显示目标磁盘剩余空间。"""

        if self.video_info is None:
            return
        requested = self._requested_frame_count()
        if requested is None:
            self.estimated_bytes = None
            self.estimate_var.set("请先填写有效的输出帧数。")
            return
        try:
            self.estimated_bytes = estimate_output_bytes(
                self.video_info,
                image_format=self.format_var.get(),
                jpeg_quality=self.quality_var.get(),
                frame_count=requested,
            )
            destination = Path(self.output_var.get()).expanduser()
            free = available_disk_bytes(destination)
            self.estimate_var.set(
                f"预计输出约 {_format_bytes(self.estimated_bytes)}；"
                f"目标磁盘当前可用 {_format_bytes(free)}。"
            )
        except (FrameExtractionError, OSError) as exc:
            self.estimated_bytes = None
            self.estimate_var.set(f"暂时无法估算输出大小：{exc}")

    def _start_extraction(self) -> None:
        """确认参数后创建后台工作线程。"""

        if self.video_info is None:
            return
        requested = self._requested_frame_count(show_error=True)
        if requested is None:
            return
        output_text = self.output_var.get().strip()
        if not output_text:
            messagebox.showerror("缺少输出目录", "请选择输出目录。")
            return
        output = Path(output_text).expanduser().resolve()
        if output.exists() and not output.is_dir():
            messagebox.showerror("输出路径错误", "输出位置必须是文件夹。")
            return
        if output.exists() and any(output.iterdir()):
            messagebox.showerror(
                "输出目录已有文件",
                "为避免覆盖旧数据，请选择一个空目录或新的输出位置。",
            )
            return
        try:
            free = available_disk_bytes(output)
        except OSError as exc:
            messagebox.showerror("无法检查磁盘", str(exc))
            return
        if self.estimated_bytes is not None and self.estimated_bytes > free:
            messagebox.showerror("空间不足", "预计输出大于目标磁盘可用空间。")
            return
        if self.estimated_bytes is not None and self.estimated_bytes >= 5 * 1024**3:
            if not messagebox.askyesno(
                "输出可能很大",
                f"预计会生成约 {_format_bytes(self.estimated_bytes)} 的图片。"
                "\n确认继续吗？",
            ):
                return

        options = ExtractionOptions(
            video_path=self.video_info.path,
            output_directory=output,
            image_format=self.format_var.get(),
            jpeg_quality=self.quality_var.get(),
            frame_count=requested,
        )
        self.cancel_event.clear()
        self.close_when_done = False
        self.progress_var.set(0.0)
        if self.video_info.total_frames <= 0:
            self.progress.configure(mode="indeterminate")
            self.progress.start(10)
        else:
            self.progress.stop()
            self.progress.configure(mode="determinate", maximum=100.0)
        self._set_running(True)
        self.status_var.set("正在准备解码……")
        self.worker = threading.Thread(
            target=self._worker_main,
            args=(options,),
            name="frame-extractor",
            daemon=True,
        )
        self.worker.start()

    def _worker_main(self, options: ExtractionOptions) -> None:
        """后台线程入口，只通过队列向主线程传递消息。"""

        try:
            result = extract_all_frames(
                options,
                cancel_event=self.cancel_event,
                progress_callback=lambda progress: self.events.put(
                    ("progress", progress)
                ),
            )
            self.events.put(("result", result))
        except BaseException as exc:  # 线程异常必须显示给操作员。
            self.events.put(("error", exc))

    def _poll_worker_events(self) -> None:
        """在 Tk 主线程中消费进度和完成事件。"""

        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "progress":
                    self._show_progress(payload)  # type: ignore[arg-type]
                elif kind == "result":
                    self._finish_result(payload)  # type: ignore[arg-type]
                else:
                    self._finish_error(payload)  # type: ignore[arg-type]
        except queue.Empty:
            pass
        try:
            root_exists = bool(self.root.winfo_exists())
        except tk.TclError:
            root_exists = False
        if root_exists:
            self.root.after(100, self._poll_worker_events)

    def _show_progress(self, progress: ExtractionProgress) -> None:
        """更新百分比、已完成帧数和预计剩余时间。"""

        if progress.total_frames > 0:
            percent = min(
                100.0,
                progress.frames_scanned * 100.0 / progress.total_frames,
            )
            self.progress_var.set(percent)
            percent_text = f"{percent:.1f}%"
            scan_text = (
                f"已扫描 {progress.frames_scanned:,} / "
                f"{progress.total_frames:,} 帧"
            )
        else:
            percent_text = ""
            scan_text = f"已扫描 {progress.frames_scanned:,} 帧"
        eta_text = (
            f"，预计剩余 {_format_duration(progress.eta_s)}"
            if progress.eta_s is not None
            else ""
        )
        saved_text = f"已保存 {progress.frames_written:,}"
        if progress.target_frames > 0:
            saved_text += f" / {progress.target_frames:,} 张"
        else:
            saved_text += " 张"
        self.status_var.set(
            f"正在处理：{scan_text}，{saved_text} "
            f"{percent_text}{eta_text}"
        )

    def _finish_result(self, result: ExtractionResult) -> None:
        """显示成功或取消结果，并允许打开输出目录。"""

        self.progress.stop()
        self.progress.configure(mode="determinate")
        self._set_running(False)
        self.output_var.set(str(result.output_directory))
        self.open_button.configure(state=tk.NORMAL)
        if result.outcome == "completed":
            self.progress_var.set(100.0)
            self.status_var.set(
                f"完成：共导出 {result.frames_written:,} 帧，耗时 "
                f"{_format_duration(result.elapsed_s)}。"
            )
            if not self.close_when_done:
                messagebox.showinfo(
                    "导出完成",
                    f"共导出 {result.frames_written:,} 张图片。\n\n"
                    f"位置：{result.image_directory}",
                )
        elif result.outcome == "cancelled":
            self.status_var.set(
                f"已取消；保留 {result.frames_written:,} 张已完成图片。"
            )
            if not self.close_when_done:
                messagebox.showinfo("已取消", self.status_var.get())
        else:
            self.status_var.set(
                f"导出未完整完成：保留 {result.frames_written:,} 张图片。"
            )
            if not self.close_when_done:
                messagebox.showerror("导出不完整", result.detail)
        if self.close_when_done:
            self.root.destroy()

    def _finish_error(self, error: BaseException) -> None:
        """显示失败原因；已写入的图片和状态文件保持可检查。"""

        self.progress.stop()
        self.progress.configure(mode="determinate")
        self._set_running(False)
        self.open_button.configure(
            state=(tk.NORMAL if Path(self.output_var.get()).exists() else tk.DISABLED)
        )
        self.status_var.set(f"导出失败：{error}")
        if self.close_when_done:
            self.root.destroy()
        else:
            messagebox.showerror("导出失败", str(error))

    def _set_running(self, running: bool) -> None:
        """导出期间锁住会改变路径和格式的控件。"""

        state = tk.DISABLED if running else tk.NORMAL
        self.start_button.configure(state=state)
        self.output_button.configure(state=state)
        self.output_entry.configure(state=state)
        self.cancel_button.configure(state=(tk.NORMAL if running else tk.DISABLED))
        self.drop_zone.configure(state=state)
        if running or self.video_info is None or self.video_info.total_frames <= 0:
            self.frame_count_spinbox.state(["disabled"])
        else:
            self.frame_count_spinbox.state(["!disabled"])
        if running:
            self.quality_scale.state(["disabled"])
        elif self.format_var.get() == "jpg":
            self.quality_scale.state(["!disabled"])

    def _cancel_extraction(self) -> None:
        """请求后台线程在当前图片写完后安全停止。"""

        if self.worker is not None and self.worker.is_alive():
            self.cancel_event.set()
            self.cancel_button.configure(state=tk.DISABLED)
            self.status_var.set("正在取消；等待当前图片写完……")

    def _open_output(self) -> None:
        """打开当前输出目录。"""

        path = Path(self.output_var.get()).expanduser()
        if path.is_dir():
            try:
                _open_directory(path)
            except OSError as exc:
                messagebox.showerror("无法打开目录", str(exc))

    def _on_close(self) -> None:
        """关闭窗口时先取消工作线程，避免正在写入的图片被截断。"""

        if self.worker is not None and self.worker.is_alive():
            if not messagebox.askyesno(
                "仍在导出",
                "是否取消导出并关闭？已经完成的图片会保留。",
            ):
                return
            self.close_when_done = True
            self._cancel_extraction()
            return
        self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    """创建图形工具的少量启动参数。"""

    parser = argparse.ArgumentParser(
        description="拖入视频并导出全部或均匀选取的帧图片。"
    )
    parser.add_argument("video", nargs="?", help="可选：启动时直接加载的视频")
    parser.add_argument(
        "--project-dir",
        default=os.environ.get("ROV_PROJECT_DIR", str(Path.cwd())),
        help="工程根目录，用于确定默认 output 位置",
    )
    return parser


def _create_root() -> tk.Tk:
    """优先创建支持系统拖放的窗口，缺依赖时回退到点击选择。"""

    if TkinterDnD is not None:
        return TkinterDnD.Tk()
    return tk.Tk()


def main(argv: list[str] | None = None) -> int:
    """启动桌面抽帧软件。"""

    args = build_parser().parse_args(argv)
    try:
        root = _create_root()
    except tk.TclError as exc:
        print(f"无法创建图形窗口：{exc}", file=sys.stderr)
        return 2
    app = FrameExtractorApp(root, project_directory=Path(args.project_dir))
    if args.video:
        app._load_video(Path(args.video))
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
