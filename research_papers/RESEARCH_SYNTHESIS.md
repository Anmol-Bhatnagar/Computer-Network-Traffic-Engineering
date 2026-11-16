# Research Synthesis: Vehicular Traffic Flow Theory & Computer Network Congestion Control

**Project Title:** Smart Packet Routing Simulation System (Research & Demonstration)  
**Course Code:** 24B15CS313 (Computer Networks)  
**Institution:** Jaypee Institute of Information Technology (JIIT)  
**Authors:** Anmol Bhatnagar, Krrish Rawat, Mitanshu Verma  
**Supervisor:** Ms. Akanksha  

---

## 1. Executive Summary & Research Objectives

Modern computer networks and vehicular transportation systems share a fundamental mathematical objective: **the efficient, safe, and stable transfer of discrete units (packets or vehicles) through a shared, capacity-constrained topological network (links/routers or highways/intersections)**.

This research and simulation project evaluates how vehicular traffic management concepts can inform, model, and improve computer network traffic engineering. Specifically, we investigate three central research questions formulated in the project synopsis:
1. **Hierarchical vs. Flat Routing:** How does dividing a large city network into subnets with hierarchical routing through gateways alter path length (hop count), routing table size, and end-to-end latency compared with flat routing?
2. **Transport Reliability & Error Recovery:** How do connection-oriented (TCP-like) and connectionless (UDP-like) delivery mechanisms behave under varying packet loss, bit corruption, and link failures when utilizing Stop-and-Wait, Go-Back-N, and Selective Repeat ARQ?
3. **Congestion Dynamics & Cross-Domain Control:** How do vehicular-inspired management mechanisms (e.g., dynamic signal timing/quantum scheduling, highway ramp metering/token bucket shaping, and emergency vehicle preemption/priority queuing) mitigate bufferbloat, shockwaves, and link overloads in packet networks?

---

## 2. Literature Review & Theoretical Foundations

### 2.1 The Foundational Traffic Flow Analogy: The Lighthill-Whitham-Richards (LWR) Model
In transportation physics, the macroscopic movement of vehicles is governed by the continuity equation proposed by Lighthill & Whitham (1955) and Richards (1956):
$$\frac{\partial k}{\partial t} + \frac{\partial q}{\partial x} = 0$$
where:
- $k(x, t)$ is traffic density (vehicles per kilometer or packets per buffer meter),
- $q(x, t)$ is traffic flow or throughput (vehicles or packets per second),
- $v(x, t) = \frac{q}{k}$ is the mean velocity or propagation speed.

**Greenshields' Fundamental Diagram of Traffic:**
$$v(k) = v_f \left(1 - \frac{k}{k_{\text{jam}}}\right) \implies q(k) = v_f \left(k - \frac{k^2}{k_{\text{jam}}}\right)$$
In computer networks, as link utilization approaches capacity ($k \to k_{\text{jam}}$), queuing delay grows non-linearly according to the $M/M/1$ queuing model:
$$T_{\text{queue}} = \frac{1}{\mu - \lambda} = \frac{1}{\mu(1 - \rho)}$$
where $\rho = \frac{\lambda}{\mu}$ is traffic intensity. When $\rho \to 1$, queue delay explodes, creating the digital equivalent of a "phantom traffic jam" or moving shockwave.

### 2.2 TCP Congestion Avoidance and Control (Jacobson, 1988)
Van Jacobson's landmark SIGCOMM '88 paper formulated the principle of **Conservation of Packets**: *a new packet is not injected into the network until an old packet has completed its journey and exited*.
- **Slow Start:** Probes available bandwidth by doubling the congestion window ($cwnd$) every Round-Trip Time (RTT):
  $$cwnd \leftarrow cwnd + 1 \quad \text{(per ACK received)}$$
- **Congestion Avoidance (AIMD):** Once $cwnd \ge ssthresh$, growth switches to Additive Increase:
  $$cwnd \leftarrow cwnd + \frac{1}{cwnd} \quad \text{(per ACK)} \implies cwnd \leftarrow cwnd + 1 \quad \text{(per RTT)}$$
- **Multiplicative Decrease:** Upon packet loss or 3 duplicate ACKs (Fast Retransmit):
  $$ssthresh \leftarrow \max\left(2, \frac{cwnd}{2}\right), \quad cwnd \leftarrow ssthresh + 3$$
This feedback loop is isomorphic to a driver moderating speed based on visible brake lights ahead.

### 2.3 Active Queue Management (AQM) & RED (Floyd & Jacobson, 1993)
Traditional **Drop-Tail** buffers allow queues to fill completely before dropping all incoming packets, triggering global TCP synchronization (where all senders throttle down simultaneously, leaving the pipe underutilized).
**Random Early Detection (RED)** computes an exponentially weighted moving average queue length:
$$avg \leftarrow (1 - w_q) \cdot avg + w_q \cdot q$$
Drop/Mark probability:
$$p_b = p_{\max} \cdot \frac{avg - min_{th}}{max_{th} - min_{th}}$$
This mirrors variable electronic highway message signs alerting drivers miles in advance to ease off the throttle, preventing shockwave collapse.

### 2.4 Kerner's Three-Phase Traffic Theory (Kerner, 2004)
Boris Kerner demonstrated that traffic does not transition abruptly from free flow to jam, but passes through an intermediate state:
1. **Free Flow ($F$):** High speed, unconstrained throughput.
2. **Synchronized Flow ($S$):** High density, continuous motion, but vehicle speeds are synchronized; highly vulnerable to perturbations.
3. **Wide Moving Jam ($J$):** Localized standstill wave propagating upstream.
In computer networks, this maps directly to:
- Uncongested buffers ($F$),
- Bufferbloat with high queuing delay but full line-rate throughput ($S$),
- Congestive Collapse with heavy packet drops and retransmission storms ($J$).

### 2.5 Highway Ramp Metering (ALINEA) vs. Token Bucket Traffic Shaping
The ALINEA algorithm (Papageorgiou et al., 1991) is a feedback control law for highway on-ramps:
$$r(t) = r(t - 1) + K_R (\hat{o} - o(t))$$
where $o(t)$ is measured highway occupancy and $\hat{o}$ is desired capacity occupancy.
In networks, **Token Bucket (RFC 2697/2698)** and **Leaky Bucket** enforce:
$$\text{Output Rate} \le r \quad \text{and} \quad \text{Maximum Burst} \le b$$
preventing bursty edge traffic from causing breakdown on core backbone links.

---

## 3. Cross-Domain Comparative Mapping

| Domain Concept | Vehicular Traffic Domain | Computer Network Domain | Mathematical / Algorithmic Mapping |
| :--- | :--- | :--- | :--- |
| **Atomic Entity** | Vehicle (Car, Truck, Bus) | Data Packet (IPv4, Frame) | Has Size, Source, Destination, TTL |
| **High Priority Entity** | Emergency Ambulance, Fire Truck | High-Priority Packet (ACK, OSPF, VoIP) | Expedited Forwarding, Strict Priority Queue |
| **Conduit / Pathway** | Road, Multi-lane Highway | Physical Link (Fiber, Twisted Pair) | Bandwidth $R$ (bps), Propagation Delay $d_{\text{prop}}$ |
| **Routing Entity** | Intersection, Roundabout | Router, L3 Switch, Gateway | Routing Table lookup, Output Buffer |
| **Traffic Signaling** | Traffic Light (Green/Yellow/Red) | Packet Scheduler (FIFO, DWRR, WFQ) | Time-quantum allocation per interface |
| **Admission Control** | On-Ramp Metering Light | Token Bucket / Leaky Bucket Shaper | Rate $r$ tokens/s, Burst $b$ bytes |
| **Congestion Indicator** | Visual brake lights, slowing speed | Packet drop, RTT inflation, ECN bit | Explicit Congestion Notification (RFC 3168) |
| **Rate Adaptation** | Driver accelerating/braking | TCP AIMD Congestion Control | $W_{t+1} = W_t + \alpha$ or $W_t \cdot \beta$ |
| **Detour Mechanism** | GPS / Waze dynamic detour | Dynamic Routing Reconvergence (Dijkstra) | Link-state update on overloaded cost |
| **Network Partitioning**| City Wards / Districts | Subnets (`/24`, Gateway Routers) | Hierarchical routing table reduction |

---

## 4. End-to-End Delay Formulation

The total delay experienced by packet $p$ traversing path $P = (e_1, e_2, \dots, e_k)$ is:
$$D_{\text{total}}(p) = \sum_{e_i \in P} \left( d_{\text{proc}}(e_i) + d_{\text{queue}}(e_i) + d_{\text{trans}}(e_i) + d_{\text{prop}}(e_i) \right)$$
where:
- **Transmission Delay:** $d_{\text{trans}}(e_i) = \frac{L}{R_i}$ ($L$ = packet length in bits, $R_i$ = link bandwidth in bps).
- **Propagation Delay:** $d_{\text{prop}}(e_i) = \frac{dist_i}{v_{\text{prop}}}$ ($v_{\text{prop}} \approx 2 \times 10^8 \text{ m/s}$).
- **Queuing Delay:** $d_{\text{queue}}(e_i) = \sum_{j \in \text{Queue}(e_i)} \frac{L_j}{R_i}$.
- **Processing Delay:** $d_{\text{proc}}(e_i) = \text{Route Table Lookup} + \text{Internet Checksum Verification}$.

---

## 5. Downloaded Research Papers in `research_papers/`

1. `Jacobson1988_Congestion_Avoidance_and_Control.pdf` (Van Jacobson, ACM SIGCOMM 1988)
2. `Floyd1993_Random_Early_Detection_Gateways.pdf` (Sally Floyd & Van Jacobson, IEEE/ACM ToN 1993)
3. `Cardwell2016_BBR_Congestion_Control.pdf` (Neal Cardwell et al., ACM Queue 2016)
4. `Kerner2004_Three_Phase_Traffic_Theory.pdf` (Boris S. Kerner, arXiv / Springer 2004)
5. `ArXiv2403_Congestion_Control_Network_Calculus.pdf` (Network Calculus Characterization, 2024)
6. `ArXiv2205_Network_Congestion_Signaling_Games.pdf` (Public Signals in Network Congestion Games, 2022)
7. `ArXiv2306_Vehicular_Traffic_Congestion_IoV.pdf` (Traffic Congestion in Internet of Vehicles, 2023)
8. `RFC793_Transmission_Control_Protocol.txt` (IETF Transmission Control Protocol Specification)
9. `RFC826_Ethernet_Address_Resolution_Protocol.txt` (IETF Address Resolution Protocol)
10. `RFC3168_Explicit_Congestion_Notification.txt` (IETF ECN for IP)
11. `RFC2698_Two_Rate_Three_Color_Marker.txt` (IETF Token Bucket Traffic Policing)
