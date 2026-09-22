A comprehensive system for detecting, tracking and gimbal centering which aims to guide an anti-drone drone to its target by using image processing methods.

# Anti-Drone Computer Vision System (C-UAS Perception Stack)

A simulation-based computer vision pipeline for **detecting, tracking, and
visually locking onto** an airborne drone target from another aerial
platform, built on **PX4 + ROS 2 + Gazebo**.

> **Scope:** detection, tracking, and visual servoing (keeping the target
> centered in frame / pointing a sensor at it). Weapon guidance and
> engagement are explicitly **out of scope** and are not addressed anywhere
> in this repository.

---

## 📽️ Demo

[![Watch the demo](https://drive.google.com/thumbnail?id=11CeErPuAVbYg5EnJ62YkM-md4crZ_mjL&sz=w1000)](https://drive.google.com/file/d/11CeErPuAVbYg5EnJ62YkM-md4crZ_mjL/view?usp=drive_link)

---

## 📖 Overview

This project simulates an anti-drone (C-UAS) perception system end-to-end:
an observer drone flies in a Gazebo simulation, its onboard camera feed is
streamed through ROS 2, and a set of interchangeable detection and tracking
modules locate and follow a second, target drone in the scene.

The system was built in stages, each validated independently before being
combined:

1. **Simulation environment** — a working PX4 SITL + Gazebo + ROS 2 stack
2. **Detection** — a YOLO-based ROS 2 node that finds the target drone in
   the camera feed
3. **Classical motion detection** — a from-scratch C++ implementation of a
   Gaussian background-subtraction detector, for small/low-contrast targets
   where deep learning struggles
4. **Tracking / lock** — a from-scratch correlation tracker (MOSSE) plus a
   wrapped, production-grade tracker (CSRT), both behind a shared interface
   so they can be swapped or compared

---

## 🧱 Architecture

```
                     ┌────────────────────┐
                     │   Gazebo Harmonic   │  simulated world, two drones
                     └─────────┬──────────┘
                               │ camera topic (ros_gz bridge)
                               ▼
                     ┌────────────────────┐
                     │   ROS 2 (Humble)    │  image + flight-data transport
                     └─────────┬──────────┘
              ┌────────────────┼────────────────┐
              ▼                                  ▼
   ┌─────────────────────┐          ┌───────────────────────┐
   │   Detection layer     │          │  PX4 flight data      │
   │  - YOLO (learned)     │          │  (via Micro XRCE-DDS  │
   │  - GMD (classical,    │          │   Agent bridge)       │
   │    motion-based)      │          └───────────────────────┘
   └──────────┬───────────┘
              │ bounding box (once, or on re-acquisition)
              ▼
   ┌─────────────────────┐
   │   Tracking / lock    │   VTracker interface
   │  - MOSSE (from scratch, own FFT)
   │  - CSRT (OpenCV, wraps the same CSR-DCF
   │          algorithm commercial libraries use)
   └──────────┬───────────┘
              ▼
        locked target
     (x, y, width, height,
      confidence per frame)
```

**Why two detectors?** Deep-learning detectors (YOLO) struggle on very
small, low-contrast, distant targets — at 15–40 px there is little texture
for a CNN to latch onto. Classical motion detection (background modeling +
thresholding) exploits *temporal* change instead of *appearance* and
performs better at range, at the cost of requiring a mostly-static camera.
The project treats both as interchangeable **detector implementations**
behind the same interface, in the spirit of how commercial C-UAS vision
libraries are structured.

**Why two trackers?** MOSSE was implemented from scratch (including its own
FFT) to understand *why* correlation tracking works, not just to call a
library. CSRT (OpenCV) is the production-grade descendant of the same
algorithm family (MOSSE → KCF → CSR-DCF) and is used as a benchmark against
the hand-written version. Both sit behind one `VTracker` interface so the
backend is swappable with a single argument.

---

## 🛠️ Tech Stack

| Layer | Technology |
|---|---|
| OS | Ubuntu 22.04 LTS (dual-boot) |
| Flight stack | PX4-Autopilot v1.17.0 (SITL) |
| Simulator | Gazebo Harmonic |
| Middleware | ROS 2 Humble |
| PX4 ↔ ROS 2 bridge | Micro XRCE-DDS Agent |
| Gazebo ↔ ROS 2 bridge | `ros_gz_image` |
| Detection (learned) | Ultralytics YOLO (v8 / v11 / v26), PyTorch, CUDA |
| Detection (classical) | Custom C++17 Gaussian motion detector (zero third-party dependencies) |
| Tracking | Custom C++17 MOSSE (own FFT) + OpenCV CSRT, behind a shared `VTracker` interface |
| Dataset / training | Custom-trained + public UAV detection datasets |

---

## 📁 Repository Structure

```
anti-drone-cv/
├── docs/                         reports, write-ups, images
├── sim/                          simulation launch scripts
├── perception/
│   ├── detection/                YOLO ROS 2 detection node
│   ├── motion_detection/         from-scratch Gaussian motion detector (C++)
│   ├── tracking/                 VTracker interface, MOSSE, CSRT, demos
│   └── control/                  pursuit / visual-servoing controller
├── models/                       trained model weights + links to external ones
└── data/                         → see Google Drive link below
```

---

## 📂 Dataset & Large Files

Training data, video clips, and full-size model weights are kept outside
this repository (GitHub is not meant for large binary storage) and are
available here:

👉 **[Google Drive folder](https://drive.google.com/drive/folders/1Q9OAEnbDwisLjJL6cXj7viFEG7oHPNO4?usp=sharing)**

This includes the labeled dataset, training notebooks/scripts, and the
trained detection model weights.

---

## 🚀 Getting Started

```bash
# 1. Launch the simulation (PX4 SITL + Gazebo + camera bridge)
./sim/start_drone_sim.sh

# 2. Run the YOLO detection node
python3 perception/detection/yolo_detect_node.py

# 3. Run a tracking demo (click on the target while the video plays)
g++ -std=c++17 -O2 perception/tracking/tracker_click_demo.cpp \
    -o tracker_click_demo `pkg-config --cflags --libs opencv4`
./tracker_click_demo <video.mp4> csrt   # or: mosse
```

Full setup instructions (Ubuntu install, PX4/ROS 2/Gazebo version pinning,
bridge configuration) are in [`docs/`](docs/).

---

**Limits:** ~10 MB for older accounts, up to 100 MB on repos with GitHub's
newer video support enabled. If your clip is larger, either trim it, or
host it on YouTube and embed a clickable thumbnail instead:
```markdown
[![Watch the demo](docs/images/demo-thumbnail.png)](https://youtu.be/VIDEO_ID)
```
(This is what the placeholder at the top of this README uses — replace
`VIDEO_ID` and the thumbnail image once you have one.)

---

## 🧭 Scope & Limitations

- Detection, tracking, and visual servoing are implemented and tested in
  simulation. **Weapon guidance / engagement is out of scope** and is not
  part of this project.
- Off-the-shelf detection models show a sim-to-real gap against Gazebo's
  rendering; fine-tuning on simulation-captured frames or a purpose-built
  dataset closes most of this gap.
- The Gaussian motion detector assumes a largely static camera; a moving
  observer platform requires ego-motion compensation (not yet implemented)
  or a switch to the correlation/learned detector.
