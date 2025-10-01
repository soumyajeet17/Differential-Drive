# 1D Multi-Agent Consensus Control over ROS

[![Language](https://img.shields.io/badge/Language-Python-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

A ROS-based implementation of a 1D multi-agent consensus algorithm. This project demonstrates how a group of independent agents, communicating over a defined network topology, can converge to a single, agreed-upon state value (e.g., position, velocity, or opinion).

---

## 💡 Core Concepts

Consensus is a fundamental problem in multi-agent systems where a group must agree on a certain data value. This implementation uses a discrete-time, neighbor-based consensus protocol.

Each agent $i$ updates its state $x_i$ based on the states of its neighbors $\mathcal{N}_i$. The update rule is governed by the following equation:

$$
x_i[k+1] = x_i[k] + \epsilon \sum_{j \in \mathcal{N}_i} (x_j[k] - x_i[k])
$$

Where:
- $x_i[k]$ is the state of agent $i$ at timestep $k$.
- $\epsilon$ is the learning rate or coupling gain, a value between 0 and 1.
- $\mathcal{N}_i$ is the set of neighbors of agent $i$.

The final consensus value is the average of the initial states of all agents, i.e., $\frac{1}{N}\sum_{i=1}^{N} x_i[0]$.

---

## 🕸️ Communication Topology

The connections between agents determine how quickly they reach consensus. This implementation supports multiple topologies, such as a **line** or a **ring**, which can be configured.

## Implementation link:
[![LinkedIn](https://www.linkedin.com/posts/control-and-automation-ee-nit-rourkela_this-video-showcases-the-final-mini-project-activity-7341345566716563456-4zCC?utm_source=share&utm_medium=member_desktop&rcm=ACoAACOmVfgBdzWfQP6TXvyTixdY0e8QreT9K9c)


**Example: Ring Topology for 3 Agents**
```mermaid
graph LR
    A[Agent 1] --- B[Agent 2];
    B --- C[Agent 3];
    C --- A


