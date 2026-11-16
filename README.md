# Cross-Domain Flow Control Visual Simulator & Research Platform
### Applying Vehicular Traffic Management Principles to Computer Network Traffic Engineering
**Course:** Computer Networks (24B15CS313) | **Institution:** Jaypee Institute of Information Technology (JIIT)  
**Academic Session:** 2026 | **Faculty Guide:** Ms. Akanksha

---

## 📌 Executive Summary

Vehicular traffic moving through an urban road grid and digital packets traversing router meshes are mathematically isomorphic: **both are dynamic flows through capacity-constrained graphs**. 

This project implements a standalone, interactive research simulator and automated experimental testbed that directly demonstrates this bridge. It proves how congestion-control algorithms originally conceived in packet switching (e.g., Tassiulas & Ephremides 1992 Back-Pressure) and modernized in intelligent transportation systems (e.g., Varaiya 2013 Max-Pressure, Wei et al. 2019 CoLight) transfer bidirectionally between urban traffic management and computer network traffic engineering.

---

## 🚀 Quick Launch (1-Click Run)

No compilation or complex dependencies needed. Double-click any of the provided batch launchers:

| Launcher File | Mode | Description |
|---|---|---|
| **`RUN_SIMULATOR.bat`** | 🖥️ **Native Win32 GUI** | Dual-screen visual simulator (1440×900) showing animated cars and packets side-by-side with 4 live strip charts. |
| **`RUN_WEB_DASHBOARD.bat`** | 🌐 **Web Browser** | Interactive HTML5 Canvas simulator with live sliders, queue bars, and visual network graph. |
| **`RUN_RESEARCH_EXPERIMENTS.bat`** | 📊 **Batch Engine** | Headless runner executing 24 controlled experimental scenarios (48,000 simulation ticks) with full CSV/JSON telemetry. |
| **`RUN_CLI_SUITE.bat`** | ⌨️ **CLI Console** | Full-featured command-line platform with Dijkstra routing, subnet addressing, and live experiments. |
| **`BUILD_ALL.bat`** | 🔨 **Compiler** | Statically compiles all C++ executables using MinGW g++ (`-O2 -static -mwindows`). |

---

## 📁 Project Structure

```
CN/
├── 🚀 1-Click Launchers
│   ├── RUN_SIMULATOR.bat               # Launch Win32 GUI visual simulator
│   ├── RUN_WEB_DASHBOARD.bat           # Open HTML5 Canvas web simulator
│   ├── RUN_RESEARCH_EXPERIMENTS.bat    # Execute 24-scenario automated research trial
│   ├── RUN_CLI_SUITE.bat               # Launch interactive CLI research platform
│   └── BUILD_ALL.bat                   # Rebuild all executables with static linking
│
├── 📦 Standalone Executables (Statically linked, zero external DLL dependencies)
│   ├── CN_VisualSimulator.exe          # Real-time GUI visual simulator
│   ├── CN_ResearchRunner.exe           # Automated headless research runner
│   └── CN.exe                          # Interactive console research suite
│
├── 📂 src/                             # Source code
│   ├── CN_VisualSimulator.cpp          # Win32 GDI dual-screen visual lab (1440x900)
│   ├── CN_ResearchRunner.cpp           # Headless 24-scenario research engine
│   ├── CN.cpp                          # Console research suite & simulator launcher
│   └── CN_original_vehicular.cpp       # Preserved baseline vehicular implementation
│
├── 📂 docs/                            # Research Documentation & Reports
│   ├── RESEARCH_REPORT.md              # 9-section report from 48,000 tick trial
│   ├── ARCHITECTURE.md                 # System architecture, state machine & diagrams
│   └── Computer_Networks_Research_Synopsis.pdf # Original JIIT project research synopsis
│
├── 📂 data/                            # Network Topology & Configuration CSVs
│   ├── road_network.csv                # Edge definitions, bandwidth, lengths, weights
│   ├── subnets.csv                     # Hierarchical /24 subnet definitions (A, B, C, D)
│   ├── nodes.csv                       # IP, MAC, node types, and buffer limits
│   ├── road_closures.csv               # Configured link failure / accident scenarios
│   ├── vehicles.csv                    # Baseline vehicular flow records
│   ├── emergency_vehicles.csv          # High-priority preemptive emergency flows
│   └── traffic_signals.csv             # Signal cycle durations & green splits
│
├── 📂 web/                             # Web-based visualization
│   └── visual_simulator.html           # HTML5 Canvas simulator with real-time charts
│
├── 📂 research_logs/                   # Empirical Datasets & Experimental Output
│   ├── summary.csv                     # Aggregated metrics for all 24 scenarios
│   ├── summary.json                    # Machine-readable JSON telemetry for dashboards
│   ├── raw_ticks.csv                   # 48,000 ticks of per-step queue & delay logs
│   └── hypothesis_eval.txt             # Automated mathematical evaluation of H1-H4
│
└── 📂 research_papers/                 # Downloaded Research Literature & RFCs
    ├── RESEARCH_SYNTHESIS.md           # Theoretical synthesis & literature review
    ├── Varaiya2013 / Zaidi2016 / Wei2019 (CoLight) / ArXiv Max-Pressure papers
    └── Classic Protocols: Jacobson1988, Floyd1993 (RED), RFC 793, RFC 3168 (ECN)
```

---

## 🎮 Visual Simulator Features

When running `CN_VisualSimulator.exe` (or `RUN_SIMULATOR.bat`):

### 1. Dual-Screen Isomorphic View (Top Half)
- **Left (Vehicular World):** 6-node road grid with intersections, queue counters, animated colored vehicles, and traffic signal heads changing color dynamically according to queue depth.
- **Right (Computer Network World):** Corresponding 6-node router mesh with dynamic communication links, buffer fill bars (blue → yellow → red), and packets flying across links.
- **Dynamic Link Coloring:** Active links smoothly shift color based on real-time link utilization ($U \in [0.0, 1.0]$) from cool teal to critical red.

### 2. Live Research Dashboard (Bottom Half)
- **4 Real-Time Strip Charts (GDI Polyline):**
  1. *Vehicular Total Queue Length* (cars)
  2. *Computer Network Buffer Occupancy* (packets)
  3. *Vehicular Throughput* (vehicles/s)
  4. *Network Throughput* (packets/s)
- **Live Metrics Telemetry:** Trips completed, mean travel delay, mean network latency, in-flight vehicle/packet counts.
- **Dynamic Hypothesis Card:** Automatically explains the research hypothesis active for the selected controller.
- **Cross-Domain Analogy Map:** Side-by-side mapping table rendered directly on the canvas.

### 3. Real-Time Keyboard Controls

| Key | Controller / Action | Theoretical Principle |
|---|---|---|
| **`[1]`** | **Fixed-Time (Baseline)** | Static round-robin scheduling without queue feedback. |
| **`[2]`** | **Max-Pressure** | Prioritizes movements maximizing queue differential: $P_{u \rightarrow v} = Q_u - Q_v$. |
| **`[3]`** | **CoLight (Cooperative)** | Incorporates 2-hop neighbor pressure: $\text{Score} = (Q_u - Q_v) + \alpha \sum (Q_v - Q_k)$. |
| **`[4]`** | **Predictive Lookahead** | Anticipates arrivals before buffer congestion: $P = Q_u - (Q_v + 1)$. |
| **`[F]`** | **Sever / Repair Link B–C** | Simulates a traffic accident / physical link severance to test rerouting. |
| **`[L]`** | **Cycle Traffic Load ($\rho$)** | Steps through Low ($\rho=0.35$) $\to$ Medium ($\rho=0.75$) $\to$ High ($\rho=1.20$). |
| **`[Space]`**| **Pause / Resume** | Freezes simulation state for inspection. |
| **`[R]`** | **Reset Simulation** | Clears all queues, particles, and telemetry counters. |

---

## 🔬 Key Research Findings (From 48,000-Tick Trial)

Our batch research engine (`CN_ResearchRunner.exe`) executed a 24-scenario matrix (4 controllers $\times$ 3 loads $\times$ 2 topologies $\times$ 2000 ticks). Full findings are detailed in [`RESEARCH_REPORT.md`](file:///c:/Users/anmol/Desktop/Projects/CN/RESEARCH_REPORT.md):

```
                        THROUGHPUT SUMMARY (High Load, Normal Network)
     ┌──────────────────────┬────────────────────────┬─────────────────────┬──────────────┐
     │ Controller           │ Network Throughput     │ Mean Veh Delay      │ Jain Fair    │
     ├──────────────────────┼────────────────────────┼─────────────────────┼──────────────┤
     │ [1] Fixed-Time       │ 684.0 pkts / 1000t     │ 85.0 ticks          │ 0.997        │
     │ [2] Max-Pressure     │ 679.5 pkts / 1000t     │ 101.9 ticks         │ 0.996        │
     │ [4] Predictive       │ 679.5 pkts / 1000t     │ 101.9 ticks         │ 0.996        │
     │ [3] CoLight          │ 265.5 pkts / 1000t     │ 39.6 ticks (biased) │ 0.423 (fail) │
     └──────────────────────┴────────────────────────┴─────────────────────┴──────────────┘
```

1. **H1 (Cross-Domain Equivalence): CONFIRMED.** Vehicular throughput and computer network throughput tracked within **0.5% - 1.2%** of each other across all 24 scenarios. Max-Pressure achieved **+4.3%** vehicular and **+3.2%** network throughput gain at low load, while holding queues slightly longer to establish stable gradients.
2. **H2 (CoLight Attention & Cooperation): REFUTED for high $\alpha$.** At $\alpha = 0.40$, CoLight amplified downstream queue differences, inducing severe routing loops and starvation (Jain's fairness collapsed to 0.42). Paradoxically, **severing link B-C restored stability** (Jain rose to 0.995) by physically breaking the feedback loop.
3. **H3 (Incident Recovery): TOPOLOGY-BOUND.** Under link severance, all controllers took exactly **101 ticks** to re-converge, proving that recovery time is bounded by graph diameter and reroute capacity, not scheduling policy. However, Fixed-Time had lower peak transient queues.
4. **H4 (Lookahead Predictive):** A 1-step lookahead ($Q_v + 1$) was insufficient to alter discrete routing decisions on a 6-node graph. Multi-step forecasting is required for tangible benefit.

---

## 📚 Baseline Literature

The simulation is grounded in foundational papers located in [`research_papers/`](file:///c:/Users/anmol/Desktop/Projects/CN/research_papers/):
- **Tassiulas & Ephremides (1992):** *Stability Properties of Constrained Queueing Systems* (Original back-pressure theorem).
- **Varaiya (2013):** *Max Pressure Control of a Network of Signalized Intersections* (Transportation Research Part C).
- **Zaidi, Kulcsar, Wymeersch (2016):** *Back-Pressure Traffic Signal Control with Fixed and Adaptive Routing* (IEEE T-ITS).
- **Wei et al. (2019):** *CoLight: Learning Network-Level Cooperation for Traffic Signal Control* (CIKM 2019).
- **Jacobson (1988) & Floyd (1993):** TCP Congestion Avoidance and Random Early Detection (RED).

---

## 🛠️ Manual Compilation Instructions

If you wish to recompile from source manually using MSYS2 / MinGW UCRT64:

```bash
# 1. Ensure MinGW is on PATH
set PATH=C:\msys64\ucrt64\bin;%PATH%

# 2. Compile Win32 Visual Simulator (GUI, Statically Linked)
g++ -std=c++17 -O2 -mwindows -static -static-libgcc -static-libstdc++ -o CN_VisualSimulator.exe src\CN_VisualSimulator.cpp

# 3. Compile Headless Research Engine (Console, Statically Linked)
g++ -std=c++17 -O2 -static -static-libgcc -static-libstdc++ -o CN_ResearchRunner.exe src\CN_ResearchRunner.cpp

# 4. Compile Interactive CLI Platform (Console, Statically Linked)
g++ -std=c++17 -O2 -static -static-libgcc -static-libstdc++ -o CN.exe src\CN.cpp
```
