# Edge-AI Mobile Robot with Real-Time Object Tracking

## Project Objective

The goal of this project is to design and implement a **4-wheel autonomous mobile robot**, capable of **real-time object tracking**, using a **distributed control architecture** based on:

- A **Raspberry Pi 3B+** (master node) for visual processing and AI inference
- An **Arduino UNO** (slave node) for low-level motor and sensor control

Two visual tracking modes are implemented:
1. **Classical color-based tracking (HSV)**
2. **Neural-based object detection using EfficientDet-Lite0 (TensorFlow Lite)**

---

## System Architecture

- **Raspberry Pi 3B+** → Cognitive node (image acquisition, AI inference)
- **Arduino UNO R3** → Reactive node (motor control, safety management)
- **Serial Communication** → USB with custom asynchronous protocol
- **Motion** → Differential drive (4 DC motors + L298N driver)
- **Safety Sensors** → HC-SR04 ultrasonic module

---

## Main Components
| Component       | Purpose                                 |
|------------------|------------------------------------------|
| Raspberry Pi     | AI execution + object detection         |
| Arduino UNO      | Motor + sensor management               |
| DC Motors        | Mobile platform motion                  |
| L298N Driver      | Motor control interface                 |
| HC-SR04          | Obstacle detection                      |
| USB Webcam       | Image acquisition (640x480 @ 30FPS)     |

---

## Vision Modes

### 1. HSV Tracking
- Lightweight, fast, and simple
- Poor robustness to lighting and background noise
- Implemented with OpenCV

### 2. AI-based Detection (EfficientDet-Lite0)
- Semantic, more accurate
- Slower inference on Raspberry Pi
- Implemented with TensorFlow Lite

---

## Experimental Results

| Metric             | HSV Mode          | AI Mode (TFLite)       |
|--------------------|-------------------|------------------------|
| FPS (average)      | ~10 FPS           | ~2–3 FPS               |
| Latency            | ~100 ms           | ~400 ms                |
| Tracking Robustness| Low               | High                   |

> Trade-off: speed vs. intelligence.

---

## Running the System (on Raspberry Pi)

### 1. Install Python dependencies

```bash
pip3 install -r requirements.txt
python3 src/vision_ai/main.py
Project Structer: 
smart-mobile-base-ai/
├── src/
│   ├── vision_ai/         # Python code for AI vision
│   └── motor_control/     # Arduino motor control code
├── docs/
│   ├── thesis_summary.pdf # Full thesis in PDF format
│   └── images/            # Diagrams and robot pictures
├── assets/                # Video demonstrations
├── scripts/               # Bash scripts for setup
├── requirements.txt       # Python library dependencies
├── LICENSE                # Project license (MIT)
└── README.md              # This file


