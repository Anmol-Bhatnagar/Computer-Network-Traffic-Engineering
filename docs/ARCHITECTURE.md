# System Architecture & Cross-Domain Design Specification
### Cross-Domain Flow Control Visual Simulator (JIIT 24B15CS313)

This document details the architectural layout, algorithmic formulation, and data flow of the Cross-Domain Flow Control Simulator.

---

## 1. System Overview

The simulator models flow through capacity-constrained networks across two domains simultaneously using a shared dynamic state representation:
1. **Vehicular Domain:** Roads, intersections, traffic lights, and vehicles.
2. **Computer Network Domain:** Point-to-point links, routers, scheduling buffers, and packets.

```
                               ┌──────────────────────────────────────────────┐
                               │       Ingress Rate Controller (rho)          │
                               │   Generates Vehicle & Packet Particles       │
                               └──────────────────────┬───────────────────────┘
                                                      │
                       ┌──────────────────────────────┴──────────────────────────────┐
                       ▼                                                             ▼
         ┌───────────────────────────┐                                 ┌───────────────────────────┐
         │     VEHICULAR WORLD       │                                 │     COMPUTER NETWORK      │
         │   (Road Grid & Signals)   │                                 │    (Routers & Links)      │
         ├───────────────────────────┤                                 ├───────────────────────────┤
         │ • Intersections (A - F)   │                                 │ • Core / Edge Routers     │
         │ • Physical Roads (Edges)  │                                 │ • Communication Links     │
         │ • Vehicle Queues          │                                 │ • Buffer Packets          │
         │ • Signal Lights (Phases)  │                                 │ • Queuing Schedulers      │
         └─────────────┬─────────────┘                                 └─────────────┬─────────────┘
                       │                                                             │
                       └──────────────────────────────┬──────────────────────────────┘
                                                      │
                                                      ▼
                               ┌──────────────────────────────────────────────┐
                               │           Shared Policy Evaluator            │
                               │                                              │
                               │  [1] Fixed-Time (Static)                     │
                               │  [2] Max-Pressure (Varaiya 2013)             │
                               │  [3] CoLight (Wei et al. 2019)               │
                               │  [4] Predictive (Lookahead)                  │
                               └──────────────────────┬───────────────────────┘
                                                      │
                       ┌──────────────────────────────┴──────────────────────────────┐
                       ▼                                                             ▼
         ┌───────────────────────────┐                                 ┌───────────────────────────┐
         │     Win32 Native GUI      │                                 │     Headless Engine       │
         │  (Real-Time Visual Lab)   │                                 │   (Batch Experiments)     │
         │ • Double-buffered GDI     │                                 │ • 24 Experimental Runs    │
         │ • 4 Live Strip Charts     │                                 │ • 48,000 Recorded Ticks   │
         │ • Real-Time Telemetry     │                                 │ • CSV & JSON Telemetry    │
         └───────────────────────────┘                                 └───────────────────────────┘
```

---

## 2. Mathematical Formulation of Controllers

### 2.1 Max-Pressure Routing (Varaiya 2013 / Tassiulas 1992)
For an active directed edge $(u, v) \in E$, the pressure $P_{u \rightarrow v}$ is defined as the queue differential between the upstream and downstream nodes:
$$P_{u \rightarrow v} = Q_u - Q_v$$
The next hop $v^*$ for a particle at node $u$ is chosen by maximizing pressure:
$$v^* = \arg\max_{v \in \mathcal{N}(u), (u,v) \text{ active}} (Q_u - Q_v)$$

### 2.2 CoLight Cooperative Attention (Wei et al. 2019)
To prevent downstream congestion from forming, CoLight adds a discounted neighbor pressure term:
$$\text{Score}_{u \rightarrow v} = (Q_u - Q_v) + \alpha \sum_{k \in \mathcal{N}(v), (v,k) \text{ active}} (Q_v - Q_k)$$
where $\alpha \in [0, 1]$ represents the cooperation coefficient (default: $\alpha = 0.40$).

### 2.3 Predictive Lookahead
The predictive controller anticipates the immediate queue buildup from incoming arrivals:
$$P_{u \rightarrow v}^{\text{pred}} = Q_u - (Q_v + 1)$$

---

## 3. Class & Component Architecture

### 3.1 Core Data Structures (`CN_VisualSimulator.cpp` & `CN_ResearchRunner.cpp`)
- `SimNode`: Models an intersection / router. Tracks $(x, y)$ coordinates in both canvases, queue sizes (`vehQueue`, `netQueue`), and signal state.
- `SimEdge`: Models a bidirectional road / network link with capacity, active state, and utilization metric.
- `Particle`: Represents an active vehicle or network packet with current node $u$, target destination, progress $(0.0 \to 1.0)$, and travel age.
- `TickRecord` / `ScenarioResult`: Stores metrics including throughput per tick, end-to-end delays, peak queues, stddev queue stability, and Jain's Fairness Index.

### 3.2 Fairness Metric
Jain's Fairness Index across all $n$ nodes:
$$J(x) = \frac{\left( \sum_{i=1}^n x_i \right)^2}{n \sum_{i=1}^n x_i^2}$$
where $x_i$ is the throughput served at node $i$. $J \in [1/n, 1.0]$, where $1.0$ represents perfectly balanced service.

---

## 4. Cross-Domain Mapping Table

| Metric / Component | Vehicular Network | Computer Network |
|--------------------|-------------------|------------------|
| Node | Road Intersection | Packet Switch / Router |
| Edge | Street / Highway Lane | Transmission Link |
| Ingress Flow | Vehicles entering network | Packet injection rate ($\lambda$) |
| Unit Flow | Vehicle (Car / Bus) | Network Packet / Datagram |
| Node Buffer | Queue at Red Signal | Ingress / Egress Switch Buffer |
| Capacity | Road lane throughput (cars/hr) | Link Bandwidth ($C$, bits/sec) |
| Latency | Total trip travel time | End-to-end latency ($D = \text{queue} + \text{trans} + \text{prop}$) |
| Link Failure | Road closure / accident block | Fiber cut / port down |
| Control Mechanism | Traffic Signal Timing Plan | Packet Queue Scheduler & Active Queue Management |
