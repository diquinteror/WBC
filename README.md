# 🤖 Whole Body Control (WBC) - ECN 2024–2025

### Projet Robotique

**Authors**  
- QUINTERO-ROIS  
- ZHONG  

This repository contains the implementation and results of a Whole Body Control (WBC) strategy applied to a 13 Degrees of Freedom (DoF) robot.

---

## 📌 Overview

This project is part of the robotics curriculum at **École Centrale de Nantes (ECN)** for the academic year **2024–2025**. It focuses on the development and validation of a **QP-based Whole Body Controller** for a complex robot.

---

## 📂 Structure

- `src/` – Implementation files
- `scenes/` – Robot model and enviroment
- `api/` - CopeliaSim api
- `README.md` – Project description

---
## 🚀 How to Launch

1. **Open the Simulation Scene**  
   Launch the file `scenes/13ddl_scene.ttt` using [CoppeliaSim](https://www.coppeliarobotics.com/).

2. **Run the Controller**  
   Execute the main script from the root directory:
   ```bash
   python src/main.py
## 📈 Features

- QP-based inverse dynamics control
- 13 DoF robot simulation
- Obstacle avoidance
- Constraint handling (speed limits, joint limits, etc.)
- End-effector trajectory tracking
- Performance evaluation (computation time, cost function, tracking error)

---

## 📧 Contact

For any questions or feedback, feel free to contact us via GitHub.