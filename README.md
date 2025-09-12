<p align="center">
    <img src="docs/assets/gravlib-banner.png">
</p>

<p align="center">
    <img src="https://img.shields.io/badge/Version-Beta%20V0.1-red?style=for-the-badge&color=blue">
    <img src="https://img.shields.io/github/stars/GravityLib/GravLib?style=for-the-badge&color=yellow">
    <img src="https://img.shields.io/github/contributors/GravityLib/GravLib?style=for-the-badge&color=pink">
    <img src="https://img.shields.io/badge/Flavour-Coke-red?style=for-the-badge&color=red">
</p>
<hr>

> GravLib is currently in **Beta (V0.1)**. Expect frequent updates and potential breaking changes.  
> You can always find the latest stable release [here](https://github.com/GravityLib/GravLib/releases).

<!--TODO:

make sure you create releases with the link

-->

![👷‍♂️ Build](https://img.shields.io/github/actions/workflow/status/GravityLib/GravLib/build.yml?style=for-the-badge)

## 🤗 Welcome!

👋 Welcome Adventurer! This repository is a merged effort between programmers of active V5RC teams from The King's School (1831) in the "Push Back" 2025-26 season. 

🫢 With this project, we are aiming to extend upon the existing LemLib project by [SizzinSeal](https://github.com/SizzinSeal) and its contributors, optimising it for the "Push Back" season. 

🌟 If you decide to use our code, please consider **starring our project** (and lemlib) to support us!

😊 This project was heavily inspired by Lemlib and 2654E's [Echo Rust](https://github.com/alexDickhans/echo-rs) codebase!!

---

## 📋 Table of Contents

- [🤗 Welcome!](#-welcome)
- [ℹ️ About Us](#ℹ️-about-us)
- [🪪 License](#-license)
- [🧳 V0.1 Contents](#-v01-contents)
- [👨‍🏫 Tutorials & Documentation](#-tutorials--documentation)
- [🏁 Getting Started](#-getting-started)
- [✨ How to Contribute](#-how-to-contribute)
- [❓ FAQ](#-faq)

---

## ℹ️ About Us

**Team 1831E** is a currently active V5RC **HS** team based in **The King's School, Paramatta, Sydney Australia** competing in the VEX Robotics Competition 2025-2026 "Push Back". 

**Team 1831A** is a currently active V5RC **HS** team based in the same school.

**The King’s School, Parramatta**, is one of Australia’s oldest independent schools, with a strong focus on academic excellence and innovation.

Our robotics program competes in the **VEX V5 Robotics Competition**, regularly qualifying for Nationals and Worlds. Through robotics, we focus on engineering, coding, teamwork, and creative problem-solving, essential skills that inspired the development of this library.

---

## 🪪 License  

This project is licensed under the MIT License. See the [LICENSE](https://github.com/GravityLib/GravLib/blob/main/LICENSE) file for details.  

---  

## 🧳 V0.1 Contents

### 📲 Features
- **Driver Control**
  - Tank, Split Arcade, Single Arcade
- **Odometry (Planned)**
  - Supports IMU
  - Works without tracking wheels
  - Supports multiple tracking wheel configurations
- **Motion**
  - Move to Pose
  - Turn to Point/Heading
  - Swing to Point/Heading
  - Pure Pursuit Path Following
  - Motion Chaining
- **PID Control**
  - Generic PID class for modular tuning
- **Subsystem Modularity**
  - Simple MotorGroup API
 
<!--
- **Driver Quality of Life**
  - Expo drive curves
  - Smooth deadzone compensation
  - Minimum output control
-->
 
---

## 👨‍🏫 Tutorials & Documentation
Our documentation site is currently under construction.  
Future tutorials will cover:
- Setting up GravLib in PROS
- Configuring Odometry & IMU
- Using Motion Commands (MoveToPose, Pure Pursuit, etc.)
- Creating Autonomous Routines

---

## 🏁 Getting Started  

### 📂 Prerequisites

Before running/building the project, ensure you have the following installed and set up:

- **Rust toolchain**  
  Install Rust using `rustup`. You will need `rustc`, `cargo`, and related tools.  

- **VEXIDE**

- **Hardware: VEX V5 Brain & Controller**  
  A physical V5 Brain and Controller are needed to deploy/run the code.  

- **Robot configured with required motors/sensors**  
  To use the features of GravLib (like motor control, tracking, odometry etc.), your robot must have the appropriate hardware components installed (motors, sensors, etc.).

### ⬇️ Installation

1. Clone the repository to your local machine
```bash
git clone https://github.com/GravityLib/GravLib.git
```

2. Open the project in Visual Studio Code

3. Build and deploy the code to the robot
```bash
cargo v5 build
cargo v5 upload
```

---

## ✨ How to Contribute  

We welcome contributions! But before you do so, please read the below: 

We heavily recommend using the **GitHub Desktop** Application if you are unfamiliar with Github.

To contribute:
1. Fork this repository.  
2. Create a feature branch:  
   ```bash  
   git checkout -b feature-name  
   ```  
3. Commit your changes:  
   ```bash  
   git commit -m "Add feature-name"  
   ```  
4. Push your changes and create a pull request! (Thanks for contributing!)

---

## ❓ FAQ
_**1. Is GravLib legal for VEX Robotics Competition (V5RC)?**_  
Yes. GravLib is legal for the VEX Robotics Competition as long as students can explain how it works, demonstrate understanding of the code they use, and ensure their robot still reflects their own work and skill level.

_**2. Do I need tracking wheels or an inertial sensor?**_  
No, but both are highly recommended for best performance once odometry and motion systems are fully implemented.

_**3. Do I need an SD card?**_  
No, GravLib does not require an SD card.

_**4. What units does GravLib use?**_  
Currently inches and degrees. Unit-agnostic support may be considered in the future.

---

🙏 Thanks for reading my clumsy writing, happy coding! 🙏

Cheers! 🍻

Alex Cai (LycoKodo) - Programmer of 1831E

Haoran Fang (venus-beetroot) - Programmer of 1831A

Carlos Zhang (blatantac) - Programmer of 1831N

<!--
Not contributing atm :(

Sky Fan (Skiiboi) - Programmer/Project Manager of 1831D
-->
