## Welcome To 1831E's Cafe! - This time serving: Coke 🔥🚀
---
Welcome Adventurer! This is the **worlds comp code repository** for the active V5RC team **1831E** in the "high stakes" season. 

With this project, we are aiming to extend upon the existing LemLib project (Now V4 or stable branch) by @SizzinSeal and its contributors, optimising it for the "high stakes" season. 

If you decide to use our code, please consider **starring our project** (and lemlib) to support us! (Extra credits to 1831A MS)

**Version Key Features**
 - 18 slot auton selector
 - developers page for subsystem monitoring
 - smartMotor class for modular subsystem PID (Fixed integral buildup issue)
 - A secret easteregg!

---

## License
This project is licensed under the MIT license. Check [LICENSE](https://github.com/1831-Code-Community/1831-Common-Codebase/blob/main/LICENSE) for more details.

## Table of Contents  

- [About Us](#about-us)  
- [Project Structure](#project-structure)  
- [Getting Started](#getting-started)  
- [Features](#features)  
- [How to Contribute](#how-to-contribute)  
- [License](#license)  

---

## About Us  

**Team 1831E** is a currently active V5RC **HS** team based in **The King's School, Paramatta, Sydney Australia** competing in the VEX Robotics Competition 2024-2025 "High Stakes". 

**Team 1831A** is a currently active V5RC **MS** team based in the same school.

**Team 1831D** is a currently active V5RC **HS** team based in **The King's School, Paramatta, Sydney Australia** competing in the VEX Robotics Competition 2024-2025 "High Stakes". 


<!--
NOTE - Commented out
---
## Project Structure  

Here's an overview of the repository:  

```
1831E-Robotics/
├── include/            # Header files for modular design  
│  
├── src/                # Source code for the robot  
│   ├── autons/         # Stores Auton routines for auton selector
│   └── lemlib/         # Lemlib library files
│       └── selector.cpp    # [Added feat.] Custom auton selector, contains subsystem monitoring code.
│       └── smartMotor.cpp  # [Added feat.] Subsystem PID class
│   ├── main.cpp           
│   ├── controls.cpp        # Code for subsystem controls 
│   └── robot-config.cpp    # Mostly constructurs for initiating robot devices & sensors
│
└── README.md           # Repository overview  
```  
-->
---

## Getting Started  

### Prerequisites  

Before running the code, ensure you have the following installed:  
- PROS API (https://pros.cs.purdue.edu/) *Recommended through VSCode Extension 
- VEX V5 Brain and Controller  
- Robot configured with necessary hardware components.  

### Installation  

1. Clone the repository to your local machine:  
   ```bash  
   git clone https://github.com/LycoKodo/1831E-Robotics.git  
   ```  
2. Open the project in VScode

3. Build and deploy the code to the robot. with "pros mu"

---

## Features  

- **Autonomous Modes**: Optimized routines for various competition scenarios.  
- **Driver Control**: Streamlined controls for intuitive operation.  
- **Subsystem Modularity**: Easy-to-modify subsystems for efficient development.  
- **Error Handling**: Robust mechanisms to detect and handle runtime issues.  

---

## How to Contribute  

We welcome contributions! But before you do so, please read the below: 

We heavily recommend using the **GitHub Desktop** Application if you are **unfamiliar with github**, but otherwise, you can also use the CLI if you feel comfortable enough.

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

## License  

This project is licensed under the MIT License. See the `LICENSE` file for details.  

---  

Thanks for reading my clumsy writing, happy coding!

Cheers! 🍻

Alex Cai (LycoKodo) - Programmer of 1831E

Haoran Fang (venus-beetroot) - Programmer of 1831A
