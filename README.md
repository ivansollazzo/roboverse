# What is Roboverse

**Roboverse** is a distributed robotic system designed to manage a swarm of autonomous robots inside a simulated environment.  
Each robot explores an assigned area, collects data, and shares it with other robots in the swarm.  
The ultimate goal is to build a **shared knowledge base** without a central coordinator.

---

## 📌 Project Overview

Roboverse is based on **ROS 2** and **Unity** integration, with robots modeled as unicycle vehicles.  
Key features include:

- **Point-to-point motion control**: robots autonomously reach target positions in the simulated 3D world using a stability-guaranteed controller and a finite state machine (FSM).
- **Environmental data collection**: each robot gathers sensor data (temperature, humidity, air quality) during exploration.
- **Decentralized rendezvous system**: robots calculate meeting points without a leader, ensuring distributed coordination.
- **Knowledge merging**: at rendezvous, robots exchange and merge collected data consistently.
- **Dashboard visualization**: a web interface allows operators to view knowledge and datasets in real time.

---

## 🛠️ Technologies & Tools

- **ROS 2 Jazzy**: open source operating system for modular robotic nodes and inter-node communication.  
- **Unity 6.0**: 3D simulator for environment and robot kinematics.  
- **Unity–ROS Bridge**:  
  - *ROS-TCP-Endpoint* (ROS side)  
  - *ROS-TCP-Connector* (Unity side)  
- **Ubuntu 24.04.3 LTS**: free and open source GNU/Linux operating system running ROS and networking with Unity.  
- **C++**: main programming language for performance and modular design.  
- **CMake + colcon**: build system for ROS workspace.  
- **Docker**: containerized deployment of the web dashboard (Nginx).  
- **Shapr3D**: 3D modeling of unicycle robots.

---

## 📂 Workspace Structure

```
dashboard/            # Web dashboard (HTML, resources, Docker configs). Not mandatory for system.
src/                  # ROS 2 source code (unicycle package, nodes)
srv/                  # Custom ROS services (UpdateKnowledge.srv)
roboverse_nodes.launch.xml  # ROS launch configuration
start.sh              # System startup/shutdown script
```

---

## ⚙️ System Architecture

Roboverse integrates several modules per robot:

- **Task Manager**: Handles robot tasks, navigation, rendezvous, and data exchange via FSM.  
- **Rendezvous Manager**: Calculates decentralized meeting points and coordinates convergence.  
- **Kinematics Controller**: Controls motion (rotation, translation) using Lyapunov-based control laws.  
- **Sensors Manager**: Manages simulated sensor data from Unity.  
- **Knowledge Register**: Stores sensor data in structured CSV files and manages updates across robots.  
- **Roboverse Dashboard**: Web UI to explore collected knowledge.

---

## 📐 Control theory

Roboverse implements a **two-phase controller**:

1. **Orientation** – Robot aligns with the target.  
2. **Translation with orientation correction** – Robot moves toward the target while adjusting heading.  

Stability is formally proven using **Lyapunov functions**, ensuring robust convergence.

---

## 🤝 Rendezvous Problem

- Each robot communicates its desired meeting pose.  
- An **iterative consensus algorithm** adjusts poses until convergence within a defined error threshold.  
- This ensures collective agreement without requiring a leader.

---

## 📊 Knowledge Dashboard

- Web-based dashboard (port **8080**) served via Docker/Nginx.  
- Hierarchical dataset browsing:  
  - `Unicycle <id>` → robot datasets.  
  - `Knowledge <id>` → data collected at a specific place.  
- Displays metadata (rows, columns, file size, last modified) and sensor readings in tables.

---

## 🧪 Simulation Scenario

- **Swarm size**: 3 unicycles.  
- **Places to explore**: 3 target locations.  
- **Environment**: Unity simulation of a city square with buildings, monuments, roads, and green areas.  
- **Process**:  
  1. Robots start from the center.  
  2. Each explores its assigned place.  
  3. Robots gather for rendezvous, exchange data, and continue exploration.

---

## 🚀 Getting Started

1. Clone the repository.  
2. Build the ROS workspace with `colcon build`.  
3. Launch the system with:  
   ```bash
   ./start.sh
   ```  
4. Open the dashboard at [http://localhost:8080](http://localhost:8080).

---

## 📖 Author & Credits

- **Author**: Ivan Sollazzo
- **University**: Università degli Studi di Palermo
- **Course**: Mobile Robotics Project
- **Academic Year**: 2024/2025
