# Mini Video Editor – PyQt6

🎬 **Mini Video Editor** is a lightweight, cross-platform prototype built in **Python (PyQt6)**.  
It provides a dark-themed GUI with basic yet powerful features for trimming, cropping, and editing videos, inspired by professional tools like DaVinci Resolve, but simplified and open for extension.

---

## ✨ Features

- 🖤 **Dark Theme** with high contrast for readability
- 🎚 **Precise trim fields** (`HH:MM:SS.mmm` format) for In/Out points
- ⏩ **Playback speed control** (0.5×, 1.0×, 1.5×, 2.0×)
- ⚫ **Grayscale filter** toggle
- 📝 **Text overlay** field
- 📦 Ready to integrate with **FFmpeg** for exporting
- 🚀 Built on **Python + PyQt6** — lightweight and hackable

---

## 📸 Screenshot

![Mini Video Editor UI](screenshot-mini-video-editor.png)


---

## 🔧 Installation

1. Clone the repository:

```bash
git clone https://github.com/nkranidiotis/mini-video-editor.git
cd mini-video-editor
pip install PyQt6 opencv-python numpy
python video_editor.py
```

⚠️ Make sure you have FFmpeg installed and available in your system PATH.
Download FFmpeg: https://ffmpeg.org/download.html

## 🚀 Usage
Open the app and drag-and-drop a video or select a file.
Set In and Out times manually or with the trim fields.
Adjust extras (speed, grayscale, text overlay).
Click Export (FFmpeg integration required).
