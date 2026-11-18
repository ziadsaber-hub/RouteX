# RouteX
### Dynamic and intelligent Emergency vehicle routing and traffic management  

RouteX is a research project focused on improving emergency vehicle navigation in urban environments by combining route planning, traffic-aware travel-time estimation, and traffic signal priority. The project aims to reduce emergency vehicle response time while minimizing disruption to ordinary traffic.

---

## Overview

Emergency vehicles navigating congested urban roads face delays caused by traffic congestion, road conditions, and intersections. Conventional navigation systems primarily focus on finding efficient routes, but emergency vehicle navigation introduces additional requirements, including dynamic traffic conditions and coordination with traffic signals.

RouteX explores a mathematical and simulation-based framework for emergency vehicle routing in Dokki, Greater Cairo, integrating road-network modeling, travel-time estimation, route optimization, and intersection signal priority.

## Research Objectives

- Develop a graph-based representation of the urban road network.
- Estimate road travel times under different traffic conditions.
- Compute efficient routes for emergency vehicles.
- Investigate traffic signal priority for emergency vehicles.
- Evaluate emergency vehicle travel-time improvements and their impact on ordinary traffic.

## System Architecture

RouteX is organized into four interconnected modules:

### Model A — Road Network Representation
Constructs and represents the urban road network as a directed graph, incorporating road connectivity and relevant road attributes.

### Model B — Travel-Time Estimation
Estimates road travel times using available observations and mathematical modeling to account for missing or incomplete traffic data.

### Model C — Emergency Vehicle Route Planning
Computes minimum-travel-time routes and supports rerouting under changing traffic conditions.

### Model D — Traffic Signal Priority
Simulates traffic signal priority for emergency vehicles at intersections while considering safety constraints and its impact on surrounding traffic.

## Methodology

The project combines mathematical modeling, graph theory, optimization, and traffic simulation.

The road network is represented using graph-based structures, while travel-time estimation and route optimization are used to support emergency vehicle navigation. Traffic signal priority is investigated through simulation to evaluate its potential benefits and traffic impact.

The framework is evaluated using scenarios designed to compare emergency vehicle performance under different traffic and signal-control conditions.

```

## Research Team

**RouteX — Linear Algebra Research**

- **[Ziad Saber]** — Research Team Leader
- [Mina Meilad] — Model A
- [Jana Ahmed] — Model A
- [Janna Islam] — Model B
- [Lojy Refaat] — Model B
- [Ali Salama] — Model C
- [Ziad Abosrea] — Model C
- [Salwa Hatem] — Model D
- [Haneen ismaiel] — Model D

## Research Status

RouteX is currently under development. Individual models and simulation components are being developed, tested, and integrated.


## Acknowledgments

This research project is as part of a Linear Algebra project at Cairo University.

