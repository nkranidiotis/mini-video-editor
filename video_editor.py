#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mini Video Editor v2 (PyQt6) — Windows 10
- Drag & drop / Add Files
- Exact trim inputs (HH:MM:SS.mmm)
- Crop overlay with aspect presets
- Rotate, Flip, Snapshot PNG
- Extras: speed, scale, brightness/contrast/saturation, watermark text, fade in/out
- Export via FFmpeg (H.264/H.265, CRF), mute/copy audio, batch
- Dark theme with high-contrast white button text

pip install PyQt6 opencv-python numpy
"""
from __future__ import annotations

import json
import os
import re
import shlex
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, List

import cv2
import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets

APP_NAME = "Mini Video Editor"
ORG_NAME = "RIA Labs"
SETTINGS_FILE = str(Path.home() / ".mini_video_editor_settings.json")
VIDEO_EXTS = {".mp4", ".mov", ".mkv", ".avi", ".m4v", ".webm"}

# ------------------------------ Utility ------------------------------------

def load_settings():
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return {}
    return {}

def save_settings(data: dict):
    try:
        with open(SETTINGS_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
    except Exception:
        pass

def find_ffmpeg(settings: dict) -> Optional[str]:
    ff = settings.get("ffmpeg_path")
    if ff and Path(ff).exists():
        return ff
    try:
        from shutil import which
        if which("ffmpeg"):
            return "ffmpeg"
    except Exception:
        pass
    return None

# Time helpers

def seconds_to_hhmmss(sec: float) -> str:
    sec = max(0.0, float(sec))
    h = int(sec // 3600)
    m = int((sec % 3600) // 60)
    s = sec - h * 3600 - m * 60
    return f"{h:02d}:{m:02d}:{s:06.3f}"

def hhmmss_to_seconds(hhmmss: str) -> float:
    try:
        h, m, s = hhmmss.split(":")
        return int(h) * 3600 + int(m) * 60 + float(s)
    except Exception:
        return 0.0

# ------------------------------ Crop Overlay -------------------------------
class CropOverlay(QtWidgets.QWidget):
    cropChanged = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self._pixmap: Optional[QtGui.QPixmap] = None
        self._scaled_pixmap: Optional[QtGui.QPixmap] = None
        self._pixmap_rect = QtCore.QRect()
        self._crop_norm = QtCore.QRectF(0.1, 0.1, 0.8, 0.8)
        self._dragging = False
        self._resizing = False
        self._drag_start = QtCore.QPointF()
        self._resize_handle = None  # 'tl','tr','bl','br','l','r','t','b'
        self._aspect_lock: Optional[float] = None
        self._show_grid = True

    def setPixmap(self, pm: Optional[QtGui.QPixmap]):
        self._pixmap = pm
        self._scaled_pixmap = None
        self.update()

    def setAspectLock(self, ratio: Optional[float]):
        self._aspect_lock = ratio
        self.update()

    def paintEvent(self, e: QtGui.QPaintEvent):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor(18, 18, 20))
        if self._pixmap:
            if not self._scaled_pixmap or self._scaled_pixmap.size() != self.size():
                self._scaled_pixmap = self._pixmap.scaled(self.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio, QtCore.Qt.TransformationMode.SmoothTransformation)
                x = (self.width() - self._scaled_pixmap.width()) // 2
                y = (self.height() - self._scaled_pixmap.height()) // 2
                self._pixmap_rect = QtCore.QRect(x, y, self._scaled_pixmap.width(), self._scaled_pixmap.height())
            p.drawPixmap(self._pixmap_rect, self._scaled_pixmap)
            crop_px = self._norm_to_pixels(self._crop_norm)
            shade = QtGui.QColor(0, 0, 0, 140)
            for r in self._outside_rects(crop_px):
                p.fillRect(r, shade)
            if self._show_grid:
                pen_grid = QtGui.QPen(QtGui.QColor(255, 255, 255, 70), 1)
                p.setPen(pen_grid)
                x1 = crop_px.left() + crop_px.width() / 3
                x2 = crop_px.left() + 2 * crop_px.width() / 3
                y1 = crop_px.top() + crop_px.height() / 3
                y2 = crop_px.top() + 2 * crop_px.height() / 3
                p.drawLine(int(x1), crop_px.top(), int(x1), crop_px.bottom())
                p.drawLine(int(x2), crop_px.top(), int(x2), crop_px.bottom())
                p.drawLine(crop_px.left(), int(y1), crop_px.right(), int(y1))
                p.drawLine(crop_px.left(), int(y2), crop_px.right(), int(y2))
            pen = QtGui.QPen(QtGui.QColor(0, 200, 255), 2)
            p.setPen(pen)
            p.drawRect(crop_px)
            p.setBrush(QtGui.QColor(0, 200, 255))
            for _name, rect in self._handle_rects(crop_px):
                p.drawRect(rect)
        else:
            p.setPen(QtGui.QPen(QtGui.QColor(80, 80, 80)))
            p.drawText(self.rect(), QtCore.Qt.AlignmentFlag.AlignCenter, "Drop a video or click 'Add Files' ☝️")

    def _norm_to_pixels(self, r: QtCore.QRectF) -> QtCore.QRect:
        if self._pixmap_rect.isNull():
            return QtCore.QRect()
        x = self._pixmap_rect.left() + int(r.x() * self._pixmap_rect.width())
        y = self._pixmap_rect.top() + int(r.y() * self._pixmap_rect.height())
        w = int(r.width() * self._pixmap_rect.width())
        h = int(r.height() * self._pixmap_rect.height())
        return QtCore.QRect(x, y, w, h)

    def _pixels_to_norm(self, r: QtCore.QRect) -> QtCore.QRectF:
        if self._pixmap_rect.isNull() or self._pixmap_rect.width() == 0 or self._pixmap_rect.height() == 0:
            return QtCore.QRectF(0.1, 0.1, 0.8, 0.8)
        x = (r.x() - self._pixmap_rect.left()) / self._pixmap_rect.width()
        y = (r.y() - self._pixmap_rect.top()) / self._pixmap_rect.height()
        w = r.width() / self._pixmap_rect.width()
        h = r.height() / self._pixmap_rect.height()
        return QtCore.QRectF(x, y, w, h)

    def _outside_rects(self, crop_px: QtCore.QRect) -> List[QtCore.QRect]:
        pr = self._pixmap_rect
        return [
            QtCore.QRect(pr.left(), pr.top(), pr.width(), crop_px.top() - pr.top()),
            QtCore.QRect(pr.left(), crop_px.bottom(), pr.width(), pr.bottom() - crop_px.bottom()),
            QtCore.QRect(pr.left(), crop_px.top(), crop_px.left() - pr.left(), crop_px.height()),
            QtCore.QRect(crop_px.right(), crop_px.top(), pr.right() - crop_px.right(), crop_px.height()),
        ]

    def _handle_rects(self, crop_px: QtCore.QRect) -> List[Tuple[str, QtCore.QRect]]:
        s = 10
        l, t, r, b = crop_px.left(), crop_px.top(), crop_px.right(), crop_px.bottom()
        return [
            ("tl", QtCore.QRect(l - s, t - s, 2 * s, 2 * s)),
            ("tr", QtCore.QRect(r - s, t - s, 2 * s, 2 * s)),
            ("bl", QtCore.QRect(l - s, b - s, 2 * s, 2 * s)),
            ("br", QtCore.QRect(r - s, b - s, 2 * s, 2 * s)),
            ("l", QtCore.QRect(l - s, t + (crop_px.height() // 2) - s, 2 * s, 2 * s)),
            ("r", QtCore.QRect(r - s, t + (crop_px.height() // 2) - s, 2 * s, 2 * s)),
            ("t", QtCore.QRect(l + (crop_px.width() // 2) - s, t - s, 2 * s, 2 * s)),
            ("b", QtCore.QRect(l + (crop_px.width() // 2) - s, b - s, 2 * s, 2 * s)),
        ]

    def mousePressEvent(self, e: QtGui.QMouseEvent):
        if not self._pixmap:
            return
        crop_px = self._norm_to_pixels(self._crop_norm)
        for name, r in self._handle_rects(crop_px):
            if r.contains(e.pos()):
                self._resizing = True
                self._resize_handle = name
                self._drag_start = e.position()
                self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.SizeAllCursor))
                return
        if crop_px.contains(e.pos()):
            self._dragging = True
            self._drag_start = e.position()
            self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.ClosedHandCursor))

    def mouseMoveEvent(self, e: QtGui.QMouseEvent):
        if not self._pixmap:
            return
        crop_px = self._norm_to_pixels(self._crop_norm)
        pos = e.position()
        if self._dragging:
            delta = pos - self._drag_start
            self._drag_start = pos
            new_rect = crop_px.translated(int(delta.x()), int(delta.y()))
            new_rect = new_rect.intersected(self._pixmap_rect)
            if not self._pixmap_rect.contains(new_rect):
                new_rect.moveLeft(max(self._pixmap_rect.left(), min(new_rect.left(), self._pixmap_rect.right() - new_rect.width())))
                new_rect.moveTop(max(self._pixmap_rect.top(), min(new_rect.top(), self._pixmap_rect.bottom() - new_rect.height())))
            self._crop_norm = self._pixels_to_norm(new_rect)
            self.cropChanged.emit()
            self.update()
            return
        if self._resizing and self._resize_handle:
            rx = pos.x() - self._drag_start.x()
            ry = pos.y() - self._drag_start.y()
            self._drag_start = pos
            r = QtCore.QRect(crop_px)
            if self._resize_handle == 'tl':
                r.setLeft(r.left() + int(rx)); r.setTop(r.top() + int(ry))
            elif self._resize_handle == 'tr':
                r.setRight(r.right() + int(rx)); r.setTop(r.top() + int(ry))
            elif self._resize_handle == 'bl':
                r.setLeft(r.left() + int(rx)); r.setBottom(r.bottom() + int(ry))
            elif self._resize_handle == 'br':
                r.setRight(r.right() + int(rx)); r.setBottom(r.bottom() + int(ry))
            elif self._resize_handle == 'l':
                r.setLeft(r.left() + int(rx))
            elif self._resize_handle == 'r':
                r.setRight(r.right() + int(rx))
            elif self._resize_handle == 't':
                r.setTop(r.top() + int(ry))
            elif self._resize_handle == 'b':
                r.setBottom(r.bottom() + int(ry))
            r = r.intersected(self._pixmap_rect)
            if self._aspect_lock and r.width() > 10 and r.height() > 10:
                desired_h = int(r.width() / self._aspect_lock)
                cy = r.center().y()
                r.setHeight(max(10, desired_h))
                r.moveCenter(QtCore.QPoint(r.center().x(), cy))
                r = r.intersected(self._pixmap_rect)
            if r.width() < 20 or r.height() < 20:
                return
            self._crop_norm = self._pixels_to_norm(r)
            self.cropChanged.emit()
            self.update()
            return
        for name, hr in self._handle_rects(crop_px):
            if hr.contains(e.pos()):
                self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.SizeAllCursor)); return
        if crop_px.contains(e.pos()):
            self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.OpenHandCursor))
        else:
            self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.ArrowCursor))

    def mouseReleaseEvent(self, e: QtGui.QMouseEvent):
        self._dragging = False
        self._resizing = False
        self._resize_handle = None
        self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.ArrowCursor))

    def setCropNorm(self, rect: QtCore.QRectF):
        self._crop_norm = QtCore.QRectF(rect)
        self.cropChanged.emit()
        self.update()

    def cropNorm(self) -> QtCore.QRectF:
        return QtCore.QRectF(self._crop_norm)

# ------------------------------ Video Loader --------------------------------
@dataclass
class VideoInfo:
    path: str
    width: int
    height: int
    fps: float
    frame_count: int
    duration: float

class VideoReader(QtCore.QObject):
    frameReady = QtCore.pyqtSignal(QtGui.QPixmap, int, float)
    infoReady = QtCore.pyqtSignal(VideoInfo)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.cap: Optional[cv2.VideoCapture] = None
        self.info: Optional[VideoInfo] = None
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._read_next)
        self.playing = False
        self._frame_idx = 0

    def open(self, path: str):
        if self.cap:
            self.cap.release(); self.cap = None
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            QtWidgets.QMessageBox.critical(None, APP_NAME, f"Failed to open video:\n{path}"); return
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(self.cap.get(cv2.CAP_PROP_FPS) or 25.0)
        count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        duration = float(count / fps) if fps > 0 else 0
        self.info = VideoInfo(path, w, h, fps, count, duration)
        self._frame_idx = 0
        self.infoReady.emit(self.info)
        self.seek_frame(0)

    def _cvimg_to_qpix(self, img: np.ndarray) -> QtGui.QPixmap:
        if img is None:
            return QtGui.QPixmap()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = img.shape
        qimg = QtGui.QImage(img.data, w, h, ch * w, QtGui.QImage.Format.Format_RGB888)
        return QtGui.QPixmap.fromImage(qimg)

    def _emit_frame(self, idx: int):
        if not self.cap:
            return
        ok, frame = self.cap.read()
        if not ok:
            return
        self._frame_idx = idx
        pm = self._cvimg_to_qpix(frame)
        sec = idx / max(self.info.fps, 1.0) if self.info else 0.0
        self.frameReady.emit(pm, idx, sec)

    def _read_next(self):
        if not self.cap or not self.info:
            return
        next_idx = self._frame_idx + 1
        if next_idx >= self.info.frame_count:
            self.pause(); return
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, next_idx)
        self._emit_frame(next_idx)

    def play(self):
        if not self.cap or not self.info:
            return
        self.playing = True
        interval = int(1000.0 / max(self.info.fps, 1.0))
        self.timer.start(max(15, interval))

    def pause(self):
        self.playing = False
        self.timer.stop()

    def seek_frame(self, idx: int):
        if not self.cap or not self.info:
            return
        idx = max(0, min(int(idx), self.info.frame_count - 1))
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        self._emit_frame(idx)

# ------------------------------ FFmpeg Runner -------------------------------
class FFmpegRunner(QtCore.QObject):
    progress = QtCore.pyqtSignal(float, str)
    finished = QtCore.pyqtSignal(int)

    def __init__(self, ffmpeg_cmd: List[str], duration: float, workdir: Optional[str] = None):
        super().__init__()
        self.cmd = ffmpeg_cmd
        self.duration = max(0.001, duration)
        self.workdir = workdir
        self.proc: Optional[QtCore.QProcess] = None

    def start(self):
        self.proc = QtCore.QProcess()
        if self.workdir:
            self.proc.setWorkingDirectory(self.workdir)
        self.proc.setProcessChannelMode(QtCore.QProcess.ProcessChannelMode.MergedChannels)
        self.proc.readyReadStandardError.connect(self._read)
        self.proc.readyReadStandardOutput.connect(self._read)
        self.proc.finished.connect(self._finished)
        program = self.cmd[0]; args = self.cmd[1:]
        self.proc.start(program, args)

    def _read(self):
        if not self.proc:
            return
        data = self.proc.readAllStandardError().data().decode('utf-8', errors='ignore')
        if not data:
            data = self.proc.readAllStandardOutput().data().decode('utf-8', errors='ignore')
        if not data:
            return
        for line in data.splitlines():
            m = re.search(r"time=(\d+):(\d+):(\d+\.?\d*)", line)
            if m:
                h, m_, s = int(m.group(1)), int(m.group(2)), float(m.group(3))
                t = h * 3600 + m_ * 60 + s
                pct = min(100.0, max(0.0, (t / self.duration) * 100.0))
                self.progress.emit(pct, line)
            else:
                self.progress.emit(-1.0, line)

    def _finished(self, code: int, _status: QtCore.QProcess.ExitStatus):
        self.finished.emit(code)

    def kill(self):
        if self.proc and self.proc.state() != QtCore.QProcess.ProcessState.NotRunning:
            self.proc.kill()

# ------------------------------ UI -----------------------------------------
class DropList(QtWidgets.QListWidget):
    filesDropped = QtCore.pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(True)
        self.setDragDropMode(QtWidgets.QAbstractItemView.DragDropMode.DropOnly)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        self.setAlternatingRowColors(True)

    def dragEnterEvent(self, e: QtGui.QDragEnterEvent):
        if e.mimeData().hasUrls(): e.acceptProposedAction()
        else: super().dragEnterEvent(e)
    def dragMoveEvent(self, e: QtGui.QDragMoveEvent):
        if e.mimeData().hasUrls(): e.acceptProposedAction()
        else: super().dragMoveEvent(e)
    def dropEvent(self, e: QtGui.QDropEvent):
        paths = []
        for url in e.mimeData().urls():
            p = url.toLocalFile()
            if os.path.isdir(p):
                for root, _dirs, files in os.walk(p):
                    for f in files:
                        if Path(f).suffix.lower() in VIDEO_EXTS:
                            paths.append(str(Path(root) / f))
            else:
                if Path(p).suffix.lower() in VIDEO_EXTS:
                    paths.append(p)
        if paths: self.filesDropped.emit(paths)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} — {ORG_NAME}")
        self.resize(1280, 780)
        self.settings = load_settings()
        self.ffmpeg_path = find_ffmpeg(self.settings)

        self.reader = VideoReader()
        self.reader.frameReady.connect(self.on_frame)
        self.reader.infoReady.connect(self.on_info)

        self._setup_ui()
        self._apply_dark_theme()
        if not self.ffmpeg_path:
            self.ask_ffmpeg_path()

    def _setup_ui(self):
        splitter = QtWidgets.QSplitter()
        splitter.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.setCentralWidget(splitter)

        left = QtWidgets.QWidget(); left_layout = QtWidgets.QVBoxLayout(left)
        self.list = DropList(); self.list.filesDropped.connect(self.add_files); self.list.itemSelectionChanged.connect(self.on_select)
        left_layout.addWidget(self.list)
        btns = QtWidgets.QHBoxLayout()
        self.btn_add = QtWidgets.QPushButton("Add Files…"); self.btn_add.clicked.connect(self.on_add)
        self.btn_remove = QtWidgets.QPushButton("Remove"); self.btn_remove.clicked.connect(self.on_remove)
        self.btn_clear = QtWidgets.QPushButton("Clear"); self.btn_clear.clicked.connect(self.on_clear)
        btns.addWidget(self.btn_add); btns.addWidget(self.btn_remove); btns.addWidget(self.btn_clear)
        left_layout.addLayout(btns)
        splitter.addWidget(left); splitter.setStretchFactor(0, 0)

        right = QtWidgets.QWidget(); right_layout = QtWidgets.QVBoxLayout(right)
        self.overlay = CropOverlay();
        preview_container = QtWidgets.QWidget(); vbl = QtWidgets.QVBoxLayout(preview_container); vbl.setContentsMargins(0,0,0,0); vbl.addWidget(self.overlay)
        right_layout.addWidget(preview_container, 5)

        info_bar = QtWidgets.QHBoxLayout()
        self.lbl_res = QtWidgets.QLabel("— x — @ — fps")
        self.lbl_time = QtWidgets.QLabel("00:00:00.000 / 00:00:00.000")
        info_bar.addWidget(self.lbl_res); info_bar.addStretch(1); info_bar.addWidget(self.lbl_time)
        right_layout.addLayout(info_bar)

        # Timeline + custom time fields
        tline = QtWidgets.QHBoxLayout()
        self.btn_play = QtWidgets.QPushButton("▶ Play"); self.btn_play.clicked.connect(self.on_play_pause)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal); self.slider.setRange(0,0); self.slider.valueChanged.connect(self.on_slider)
        self.btn_in = QtWidgets.QPushButton("[ Set In"); self.btn_in.clicked.connect(self.on_set_in)
        self.btn_out = QtWidgets.QPushButton("] Set Out"); self.btn_out.clicked.connect(self.on_set_out)
        self.lbl_inout = QtWidgets.QLabel("In: 00:00:00.000  |  Out: End")
        self.edit_in = QtWidgets.QLineEdit("00:00:00.000"); self.edit_in.setFixedWidth(120); self.edit_in.setPlaceholderText("HH:MM:SS.mmm"); self.edit_in.editingFinished.connect(self.apply_in_from_edit)
        self.edit_out = QtWidgets.QLineEdit(""); self.edit_out.setFixedWidth(120); self.edit_out.setPlaceholderText("HH:MM:SS.mmm or blank=End"); self.edit_out.editingFinished.connect(self.apply_out_from_edit)
        tline.addWidget(self.btn_play); tline.addWidget(self.slider, 1); tline.addWidget(self.btn_in); tline.addWidget(self.btn_out)
        tline.addWidget(QtWidgets.QLabel("In")); tline.addWidget(self.edit_in); tline.addWidget(QtWidgets.QLabel("Out")); tline.addWidget(self.edit_out); tline.addWidget(self.lbl_inout)
        right_layout.addLayout(tline)

        # Tools row
        tools = QtWidgets.QHBoxLayout()
        self.cmb_aspect = QtWidgets.QComboBox(); self.cmb_aspect.addItems(["Free","16:9","9:16","1:1","4:3"]); self.cmb_aspect.currentIndexChanged.connect(self.on_aspect)
        self.btn_reset_crop = QtWidgets.QPushButton("Reset Crop"); self.btn_reset_crop.clicked.connect(self.on_reset_crop)
        self.cmb_rotate = QtWidgets.QComboBox(); self.cmb_rotate.addItems(["Rotate: 0°","90°","180°","270°"]) 
        self.chk_flip_h = QtWidgets.QCheckBox("Flip H"); self.chk_flip_v = QtWidgets.QCheckBox("Flip V")
        self.btn_snapshot = QtWidgets.QPushButton("Snapshot PNG"); self.btn_snapshot.clicked.connect(self.on_snapshot)
        tools.addWidget(QtWidgets.QLabel("Aspect")); tools.addWidget(self.cmb_aspect); tools.addWidget(self.btn_reset_crop); tools.addSpacing(20)
        tools.addWidget(self.cmb_rotate); tools.addWidget(self.chk_flip_h); tools.addWidget(self.chk_flip_v); tools.addSpacing(20); tools.addWidget(self.btn_snapshot); tools.addStretch(1)
        right_layout.addLayout(tools)

        # Extras panel
        extras = QtWidgets.QGroupBox("Extras"); gridx = QtWidgets.QGridLayout(extras)
        self.cmb_speed = QtWidgets.QComboBox(); self.cmb_speed.addItems(["0.25x","0.5x","0.75x","1.0x","1.25x","1.5x","2.0x"]); self.cmb_speed.setCurrentText("1.0x")
        self.cmb_scale = QtWidgets.QComboBox(); self.cmb_scale.addItems(["Original","4K (3840x2160)","1440p (2560x1440)","1080p (1920x1080)","720p (1280x720)","200%","75%","50%"]) ; self.cmb_scale.setCurrentText("Original")
        self.sld_brightness = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal); self.sld_brightness.setRange(-100,100)
        self.sld_contrast = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal); self.sld_contrast.setRange(50,200); self.sld_contrast.setValue(100)
        self.sld_saturation = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal); self.sld_saturation.setRange(50,200); self.sld_saturation.setValue(100)
        self.edit_watermark = QtWidgets.QLineEdit(); self.edit_watermark.setPlaceholderText("Watermark text (optional)")
        self.cmb_wm_pos = QtWidgets.QComboBox(); self.cmb_wm_pos.addItems(["Top-Left","Top-Right","Bottom-Left","Bottom-Right"]) 
        self.spin_fade_in = QtWidgets.QDoubleSpinBox(); self.spin_fade_in.setRange(0.0, 30.0); self.spin_fade_in.setDecimals(1); self.spin_fade_in.setSuffix(" s")
        self.spin_fade_out = QtWidgets.QDoubleSpinBox(); self.spin_fade_out.setRange(0.0, 30.0); self.spin_fade_out.setDecimals(1); self.spin_fade_out.setSuffix(" s")
        r=0
        gridx.addWidget(QtWidgets.QLabel("Playback speed"), r,0); gridx.addWidget(self.cmb_speed, r,1); r+=1
        gridx.addWidget(QtWidgets.QLabel("Scale"), r,0); gridx.addWidget(self.cmb_scale, r,1); r+=1
        gridx.addWidget(QtWidgets.QLabel("Brightness"), r,0); gridx.addWidget(self.sld_brightness, r,1); r+=1
        gridx.addWidget(QtWidgets.QLabel("Contrast"), r,0); gridx.addWidget(self.sld_contrast, r,1); r+=1
        gridx.addWidget(QtWidgets.QLabel("Saturation"), r,0); gridx.addWidget(self.sld_saturation, r,1); r+=1
        gridx.addWidget(QtWidgets.QLabel("Watermark"), r,0); gridx.addWidget(self.edit_watermark, r,1); r+=1
        gridx.addWidget(QtWidgets.QLabel("WM Position"), r,0); gridx.addWidget(self.cmb_wm_pos, r,1); r+=1
        gridx.addWidget(QtWidgets.QLabel("Fade In"), r,0); gridx.addWidget(self.spin_fade_in, r,1); r+=1
        gridx.addWidget(QtWidgets.QLabel("Fade Out"), r,0); gridx.addWidget(self.spin_fade_out, r,1)
        right_layout.addWidget(extras)

        # Export panel
        export = QtWidgets.QGroupBox("Export"); form = QtWidgets.QGridLayout(export)
        self.cmb_codec = QtWidgets.QComboBox(); self.cmb_codec.addItems(["H.264 (libx264)", "H.265 (libx265)"])
        self.sld_crf = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal); self.sld_crf.setRange(10,35); self.sld_crf.setValue(20)
        self.lbl_crf = QtWidgets.QLabel("CRF: 20 (lower = better)"); self.sld_crf.valueChanged.connect(lambda v: self.lbl_crf.setText(f"CRF: {v} (lower = better)"))
        self.chk_mute = QtWidgets.QCheckBox("Mute audio")
        self.txt_outdir = QtWidgets.QLineEdit(self.settings.get("output_dir", str(Path.home() / "Videos")))
        self.btn_outdir = QtWidgets.QPushButton("Browse…"); self.btn_outdir.clicked.connect(self.on_pick_outdir)
        self.chk_batch = QtWidgets.QCheckBox("Apply to ALL files in the list")
        self.btn_export = QtWidgets.QPushButton("Export"); self.btn_export.clicked.connect(self.on_export)
        self.btn_cancel = QtWidgets.QPushButton("Cancel Job"); self.btn_cancel.clicked.connect(self.on_cancel); self.btn_cancel.setEnabled(False)
        self.prog = QtWidgets.QProgressBar(); self.prog.setRange(0,100); self.prog.setValue(0)
        self.txt_log = QtWidgets.QPlainTextEdit(); self.txt_log.setReadOnly(True); self.txt_log.setMaximumHeight(140)
        form.addWidget(QtWidgets.QLabel("Codec"), 0,0); form.addWidget(self.cmb_codec, 0,1)
        form.addWidget(self.lbl_crf, 1,0); form.addWidget(self.sld_crf, 1,1)
        form.addWidget(self.chk_mute, 2,1)
        form.addWidget(QtWidgets.QLabel("Output Folder"), 3,0); form.addWidget(self.txt_outdir, 3,1); form.addWidget(self.btn_outdir, 3,2)
        form.addWidget(self.chk_batch, 4,1)
        form.addWidget(self.btn_export, 5,1); form.addWidget(self.btn_cancel, 5,2)
        form.addWidget(self.prog, 6,0,1,3); form.addWidget(self.txt_log, 7,0,1,3)
        right_layout.addWidget(export)

        splitter.addWidget(right); splitter.setStretchFactor(1, 1)

    def _apply_dark_theme(self):
        pal = QtGui.QPalette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor(18,18,20))
        pal.setColor(QtGui.QPalette.ColorRole.WindowText, QtCore.Qt.GlobalColor.white)
        pal.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor(25,25,28))
        pal.setColor(QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor(30,30,34))
        pal.setColor(QtGui.QPalette.ColorRole.Text, QtCore.Qt.GlobalColor.white)
        pal.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor(30,30,34))
        pal.setColor(QtGui.QPalette.ColorRole.ButtonText, QtCore.Qt.GlobalColor.white)
        pal.setColor(QtGui.QPalette.ColorRole.Highlight, QtGui.QColor(0,170,255))
        pal.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtCore.Qt.GlobalColor.black)
        self.setPalette(pal)
        self.setStyleSheet(
            """
            QWidget { font-size: 12px; }
            QGroupBox { border: 1px solid #3a3a3a; border-radius: 6px; margin-top: 12px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: #9ad; }
            QPushButton { background: #2d2f33; border: 1px solid #4a4a4a; border-radius: 6px; padding: 6px 10px; color: #ffffff; }
            QPushButton:hover { background: #3a3d42; }
            QToolButton { color: #ffffff; }
            QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox { background: #1f1f22; border: 1px solid #3a3a3a; border-radius: 4px; padding: 4px; color: #ffffff; }
            QSlider::groove:horizontal { height: 6px; background: #444; border-radius: 3px; }
            QSlider::handle:horizontal { width: 12px; background: #9ad; border: 1px solid #6aa; margin: -6px 0; border-radius: 6px; }
            QListWidget { background: #1f1f22; border: 1px solid #3a3a3a; }
            QPlainTextEdit { background: #1a1a1d; border: 1px solid #333; color: #ffffff; }
            QLabel { color: #ddd; }
            QCheckBox { color: #ffffff; }
            """
        )

    # --------------------------- Actions ------------------------------------
    def append_log(self, text: str):
        self.txt_log.appendPlainText(text.rstrip())
        self.txt_log.verticalScrollBar().setValue(self.txt_log.verticalScrollBar().maximum())

    def ask_ffmpeg_path(self):
        m = QtWidgets.QMessageBox(self)
        m.setWindowTitle(APP_NAME)
        m.setText("FFmpeg not found. Please locate ffmpeg.exe (usually in ffmpeg\\bin).")
        m.setIcon(QtWidgets.QMessageBox.Icon.Warning)
        m.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Ok | QtWidgets.QMessageBox.StandardButton.Cancel)
        if m.exec() == QtWidgets.QMessageBox.StandardButton.Ok:
            fn, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select ffmpeg.exe", str(Path.home()), "Executable (*.exe)")
            if fn:
                self.ffmpeg_path = fn
                self.settings["ffmpeg_path"] = fn
                save_settings(self.settings)
        else:
            self.append_log("FFmpeg not configured. Preview works; export disabled until set.")

    def add_files(self, paths: List[str]):
        added = 0
        for p in paths:
            ext = Path(p).suffix.lower()
            if ext in VIDEO_EXTS:
                if not any(self.list.item(i).data(QtCore.Qt.ItemDataRole.UserRole) == p for i in range(self.list.count())):
                    it = QtWidgets.QListWidgetItem(Path(p).name)
                    it.setToolTip(p)
                    it.setData(QtCore.Qt.ItemDataRole.UserRole, p)
                    self.list.addItem(it)
                    added += 1
        if added:
            self.append_log(f"Added {added} file(s)")
            self.list.setCurrentRow(self.list.count() - 1)

    def on_add(self):
        files, _ = QtWidgets.QFileDialog.getOpenFileNames(self, "Select videos", str(Path.home()), "Video Files (*.mp4 *.mov *.mkv *.avi *.m4v *.webm);;All Files (*.*)")
        if files: self.add_files(files)

    def on_remove(self):
        for it in self.list.selectedItems():
            row = self.list.row(it); self.list.takeItem(row)
        if self.list.count() == 0:
            self.overlay.setPixmap(None); self.lbl_res.setText("— x — @ — fps"); self.lbl_time.setText("00:00:00.000 / 00:00:00.000")

    def on_clear(self):
        self.list.clear(); self.overlay.setPixmap(None); self.lbl_res.setText("— x — @ — fps"); self.lbl_time.setText("00:00:00.000 / 00:00:00.000")

    def on_select(self):
        it = self.list.currentItem()
        if not it: return
        path = it.data(QtCore.Qt.ItemDataRole.UserRole)
        self.reader.open(path)

    # Video callbacks
    def on_info(self, info: VideoInfo):
        self.slider.setRange(0, max(0, info.frame_count - 1))
        self.lbl_res.setText(f"{info.width} x {info.height} @ {info.fps:.3f} fps")
        self._in_sec = 0.0; self._out_sec = info.duration
        self.edit_in.setText(seconds_to_hhmmss(self._in_sec)); self.edit_out.setText("")
        self.lbl_inout.setText(f"In: {seconds_to_hhmmss(self._in_sec)}  |  Out: End")
        self.lbl_time.setText(f"00:00:00.000 / {seconds_to_hhmmss(info.duration)}")
        self.overlay.setCropNorm(QtCore.QRectF(0.1, 0.1, 0.8, 0.8))

    def on_frame(self, pm: QtGui.QPixmap, idx: int, sec: float):
        self.overlay.setPixmap(pm)
        self.slider.blockSignals(True); self.slider.setValue(idx); self.slider.blockSignals(False)
        dur = self.reader.info.duration if self.reader.info else 0
        self.lbl_time.setText(f"{seconds_to_hhmmss(sec)} / {seconds_to_hhmmss(dur)}")

    def on_play_pause(self):
        if self.reader.playing:
            self.reader.pause(); self.btn_play.setText("▶ Play")
        else:
            self.reader.play(); self.btn_play.setText("❚❚ Pause")

    def on_slider(self, value: int):
        self.reader.seek_frame(int(value))

    def on_set_in(self):
        if not self.reader.info: return
        sec = (self.slider.value() / max(self.reader.info.fps, 1.0))
        self._in_sec = max(0.0, min(sec, self.reader.info.duration))
        self.edit_in.setText(seconds_to_hhmmss(self._in_sec))
        out_text = "End" if self._out_sec >= self.reader.info.duration else seconds_to_hhmmss(self._out_sec)
        self.lbl_inout.setText(f"In: {seconds_to_hhmmss(self._in_sec)}  |  Out: {out_text}")

    def on_set_out(self):
        if not self.reader.info: return
        sec = (self.slider.value() / max(self.reader.info.fps, 1.0))
        self._out_sec = max(0.0, min(sec, self.reader.info.duration))
        self.edit_out.setText(seconds_to_hhmmss(self._out_sec))
        out_text = seconds_to_hhmmss(self._out_sec)
        self.lbl_inout.setText(f"In: {seconds_to_hhmmss(self._in_sec)}  |  Out: {out_text}")

    def on_aspect(self, idx: int):
        mapping = {0: None, 1: 16/9, 2: 9/16, 3: 1.0, 4: 4/3}
        self.overlay.setAspectLock(mapping.get(idx))

    def on_reset_crop(self):
        self.overlay.setCropNorm(QtCore.QRectF(0.1, 0.1, 0.8, 0.8))

    def on_pick_outdir(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Select output folder", self.txt_outdir.text())
        if d:
            self.txt_outdir.setText(d); self.settings["output_dir"] = d; save_settings(self.settings)

    def _ensure_outdir(self) -> Optional[str]:
        d = self.txt_outdir.text().strip()
        if not d:
            QtWidgets.QMessageBox.warning(self, APP_NAME, "Please select an output folder."); return None
        Path(d).mkdir(parents=True, exist_ok=True)
        return d

    def current_crop_pixels(self) -> Tuple[int, int, int, int]:
        if not self.reader.info: return (0,0,0,0)
        W, H = self.reader.info.width, self.reader.info.height
        r = self.overlay.cropNorm()
        x = int(round(r.x() * W)); y = int(round(r.y() * H))
        w = int(round(r.width() * W)); h = int(round(r.height() * H))
        if w % 2: w -= 1
        if h % 2: h -= 1
        if x % 2: x -= 1
        if y % 2: y -= 1
        w = max(2, min(W - x, w)); h = max(2, min(H - y, h))
        return (x, y, w, h)

    def build_ffmpeg_command(self, in_path: str, out_path: str, crop_px: Tuple[int,int,int,int], in_sec: float, out_sec: float) -> List[str]:
        if not self.reader.info: raise RuntimeError("No video loaded")
        w, h = self.reader.info.width, self.reader.info.height
        x_px = max(0, min(w - 2, crop_px[0])); y_px = max(0, min(h - 2, crop_px[1]))
        cw = max(2, min(w - x_px, crop_px[2])); ch = max(2, min(h - y_px, crop_px[3]))
        vf_parts = [f"crop={cw}:{ch}:{x_px}:{y_px}"]
        # Rotation/flip
        rot_idx = self.cmb_rotate.currentIndex()
        if rot_idx == 1: vf_parts.append("transpose=1")
        elif rot_idx == 2: vf_parts.append("transpose=2,transpose=2")
        elif rot_idx == 3: vf_parts.append("transpose=2")
        if self.chk_flip_h.isChecked(): vf_parts.append("hflip")
        if self.chk_flip_v.isChecked(): vf_parts.append("vflip")
        # Scale
        scale_text = self.cmb_scale.currentText()
        if scale_text != "Original":
            if "x" in scale_text:
                m = re.search(r"(\d+)x(\d+)", scale_text)
                if m:
                    sw, sh = m.group(1), m.group(2)
                    vf_parts.append(f"scale={sw}:{sh}")
            elif "%" in scale_text:
                pct = int(scale_text.replace("%",""))
                vf_parts.append(f"scale=trunc(iw*{pct}/100/2)*2:trunc(ih*{pct}/100/2)*2")
            elif scale_text.endswith("p"):
                m = re.search(r"(\d+)p", scale_text)
                if m:
                    target_h = int(m.group(1))
                    vf_parts.append(f"scale=-2:{target_h}")
        # Color EQ
        b = self.sld_brightness.value() / 100.0
        c = self.sld_contrast.value() / 100.0
        s = self.sld_saturation.value() / 100.0
        if abs(b) > 0.001 or abs(c-1.0) > 0.001 or abs(s-1.0) > 0.001:
            vf_parts.append(f"eq=brightness={b}:contrast={c}:saturation={s}")
        # Watermark
        wm = self.edit_watermark.text().strip()
        if wm:
            pos = self.cmb_wm_pos.currentText()
            pos_map = {"Top-Left": ("10","10"), "Top-Right": ("w-tw-10","10"), "Bottom-Left": ("10","h-th-10"), "Bottom-Right": ("w-tw-10","h-th-10")}
            xexpr, yexpr = pos_map.get(pos, ("10","10"))
            wm_escaped = wm.replace("'","\\'")
            vf_parts.append(f"drawtext=font='Arial':text='{wm_escaped}':x={xexpr}:y={yexpr}:fontsize=24:fontcolor=white@0.85:box=1:boxcolor=black@0.45:boxborderw=6")
        # Fades (post-trim)
        dur = (out_sec - in_sec) if (out_sec > in_sec > 0) else (self.reader.info.duration - in_sec)
        fade_in = float(self.spin_fade_in.value()); fade_out = float(self.spin_fade_out.value())
        if fade_in > 0.0: vf_parts.append(f"fade=t=in:st=0:d={fade_in}")
        if fade_out > 0.0 and dur > fade_out:
            st = max(0.0, dur - fade_out); vf_parts.append(f"fade=t=out:st={st}:d={fade_out}")
        # Speed
        spd = float(self.cmb_speed.currentText().replace('x',''))
        if abs(spd - 1.0) > 0.001: vf_parts.append(f"setpts={(1.0/spd):.6f}*PTS")
        vf = ",".join(vf_parts)

        codec = self.cmb_codec.currentIndex(); vcodec = "libx264" if codec == 0 else "libx265"; crf = self.sld_crf.value()
        args = [self.ffmpeg_path or "ffmpeg", "-y"]
        if in_sec > 0: args += ["-ss", seconds_to_hhmmss(in_sec)]
        args += ["-i", in_path]
        if out_sec > in_sec > 0: args += ["-to", seconds_to_hhmmss(out_sec - in_sec)]
        args += ["-vf", vf, "-c:v", vcodec, "-crf", str(crf), "-preset", "medium"]
        if self.chk_mute.isChecked():
            args += ["-an"]
        else:
            spd = float(self.cmb_speed.currentText().replace('x',''))
            if abs(spd - 1.0) > 0.001:
                def atempo_chain(x):
                    chain = []
                    while x < 0.5:
                        chain.append("atempo=0.5"); x /= 0.5
                    while x > 2.0:
                        chain.append("atempo=2.0"); x /= 2.0
                    chain.append(f"atempo={x:.3f}")
                    return ",".join(chain)
                args += ["-filter:a", atempo_chain(spd)]
            else:
                args += ["-c:a", "aac", "-b:a", "192k"]
        args += [out_path]
        return args

    def on_export(self):
        if not self.ffmpeg_path:
            self.ask_ffmpeg_path();
            if not self.ffmpeg_path: return
        outdir = self._ensure_outdir();
        if not outdir: return
        items = self.list.selectedItems() if not self.chk_batch.isChecked() else [self.list.item(i) for i in range(self.list.count())]
        if not items:
            QtWidgets.QMessageBox.information(self, APP_NAME, "No files selected."); return
        crop = self.current_crop_pixels()
        in_sec = getattr(self, "_in_sec", 0.0); out_sec = getattr(self, "_out_sec", 0.0)
        # Allow manual overrides
        in_txt = self.edit_in.text().strip(); out_txt = self.edit_out.text().strip()
        if in_txt: in_sec = hhmmss_to_seconds(in_txt)
        if out_txt: out_sec = hhmmss_to_seconds(out_txt)
        self.export_queue: List[Tuple[List[str], float, str]] = []
        ts = time.strftime("%Y%m%d-%H%M%S")
        for it in items:
            in_path = it.data(QtCore.Qt.ItemDataRole.UserRole)
            name = Path(in_path).stem
            out_path = str(Path(outdir) / f"{name}_EDIT_{ts}.mp4")
            duration = max(0.1, (out_sec - in_sec) if out_sec > in_sec else (self.reader.info.duration - in_sec if self.reader.info else 1.0))
            cmd = self.build_ffmpeg_command(in_path, out_path, crop, in_sec, out_sec if out_sec > 0 else 0.0)
            self.export_queue.append((cmd, duration, out_path))
        self.btn_export.setEnabled(False); self.btn_cancel.setEnabled(True); self.prog.setValue(0); self.txt_log.clear(); self._run_next_job()

    def _run_next_job(self):
        if not self.export_queue:
            self.append_log("All jobs finished."); self.btn_export.setEnabled(True); self.btn_cancel.setEnabled(False); return
        cmd, duration, out_path = self.export_queue.pop(0)
        self.append_log("Running: " + " ".join(shlex.quote(c) for c in cmd))
        self.current_runner = FFmpegRunner(cmd, duration)
        self.current_runner.progress.connect(self.on_progress)
        self.current_runner.finished.connect(self.on_finished_job)
        self.current_runner.start(); self.current_outpath = out_path

    def on_progress(self, pct: float, line: str):
        if pct >= 0: self.prog.setValue(int(pct))
        if line and ("frame=" in line or "speed=" in line or "time=" in line): self.append_log(line)

    def on_finished_job(self, code: int):
        if code == 0: self.append_log(f"✅ Done: {self.current_outpath}")
        else: self.append_log(f"❌ FFmpeg exited with code {code}")
        self._run_next_job()

    def on_cancel(self):
        try: self.current_runner.kill()
        except Exception: pass
        self.export_queue.clear(); self.btn_export.setEnabled(True); self.btn_cancel.setEnabled(False); self.append_log("Cancelled.")

    def apply_in_from_edit(self):
        if not self.reader.info: return
        t = self.edit_in.text().strip(); self._in_sec = 0.0 if not t else max(0.0, min(hhmmss_to_seconds(t), self.reader.info.duration))
        out_text = "End" if getattr(self, "_out_sec", self.reader.info.duration) >= self.reader.info.duration else seconds_to_hhmmss(self._out_sec)
        self.lbl_inout.setText(f"In: {seconds_to_hhmmss(self._in_sec)}  |  Out: {out_text}")

    def apply_out_from_edit(self):
        if not self.reader.info: return
        t = self.edit_out.text().strip(); self._out_sec = self.reader.info.duration if not t else max(0.0, min(hhmmss_to_seconds(t), self.reader.info.duration))
        out_text = "End" if self._out_sec >= self.reader.info.duration else seconds_to_hhmmss(self._out_sec)
        self.lbl_inout.setText(f"In: {seconds_to_hhmmss(self._in_sec)}  |  Out: {out_text}")

    def on_snapshot(self):
        if not getattr(self.overlay, '_pixmap', None): return
        outdir = self._ensure_outdir();
        if not outdir: return
        ts = time.strftime("%Y%m%d-%H%M%S")
        base = Path(self.list.currentItem().data(QtCore.Qt.ItemDataRole.UserRole)).stem if self.list.currentItem() else "snapshot"
        path = Path(outdir) / f"{base}_frame_{ts}.png"
        self.overlay._pixmap.save(str(path), "PNG")
        self.append_log(f"Saved snapshot: {path}")

# ------------------------------ Main ----------------------------------------
def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setOrganizationName(ORG_NAME); app.setApplicationName(APP_NAME)
    w = MainWindow(); w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
