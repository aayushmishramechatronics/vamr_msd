# VAMR_MSD for RCAAI 2026

## Experimental Evaluation of Fault-Aware Mission Recovery in Multi-Goal Autonomous Mobile Robot Navigation

**Author:** Aayush Anil Mishra  
**Programme:** Mechatronics, Robotics and Automation, MIT Manipal  
**Platform:** ROS 2 Humble, Gazebo 11, Nav2, SLAM Toolbox, AMCL

---

## 1. Project Purpose

VAMR_MSD is a ROS 2 autonomous mobile-robot platform built around a differential-drive robot, LiDAR, camera sensors, SLAM, AMCL and Nav2. The original project supports mapping, localization, navigation and multi-goal mission execution.

For **RCAAI 2026**, the project is being used as a controlled research platform. The paper is not about building a new navigation stack or adding more features. It studies one specific problem:

> **When one navigation goal fails during a multi-goal mission, can a fault-aware mission-level recovery policy keep the overall mission running more reliably than immediate mission termination or fixed retry?**

The robot, map, goals and Nav2 configuration remain common across the experiments. The main variable is the **mission-level recovery policy**.

---

## 2. Existing VAMR_MSD Platform

The repository already provides:

- ROS 2 Humble and Gazebo 11
- Differential-drive robot model with ROS 2 Control
- 2D LiDAR, RGB camera and depth-camera support
- SLAM Toolbox mapping
- AMCL localization
- Nav2 navigation and obstacle avoidance
- Multi-goal mission queue
- Basic goal retry
- Pause/resume and mission cancellation
- YAML-based mission configuration

The project also contains a separate color-based visual tracking capability. It is kept as part of the general robot platform but is **not a research variable in the RCAAI paper**.

Docking, YOLO, semantic navigation, dynamic object following, energy optimization and other planned features are outside the current RCAAI scope.

---

## 3. RCAAI Research Contribution

The RCAAI branch adds a lightweight mission-level recovery layer **above Nav2**.

Nav2 continues to handle individual navigation goals and its own internal recovery behavior. The VAMR_MSD mission manager only takes over when Nav2 ultimately reports that a goal has failed.

```text
Multi-Goal Mission
        |
        v
  Mission Manager
        |
   NavigateToPose
        |
        v
       Nav2
     /     \
 SUCCESS   FAILURE
    |          |
 Next Goal  Diagnosis
               |
        +------+------+------+
        |             |      |
       F1            F2     F3
        |             |      |
     Replan       Defer    Retry
                     /        
                  Abort goal
                       |
                       v
               Continue Mission
```

The research contribution is therefore **mission-level fault handling**, not a replacement for SLAM, AMCL or Nav2.

---

## 4. Research Question and Hypothesis

### Research Question

> **How does fault-aware mission recovery affect mission completion reliability and recovery performance compared with no mission-level recovery and fixed mission-level retry under controlled navigation disturbances?**

### Hypothesis

A mission manager that identifies the failure condition and applies an appropriate recovery action should maintain mission continuity more effectively than immediate mission termination or blind retry.

The policy and experimental parameters are fixed before the main campaign and are not changed after observing results.

---

## 5. Experimental Policies

All three policies use the same robot, map, mission, Nav2 configuration and simulation settings.

### B1 — No Mission-Level Recovery

Nav2 performs its normal behavior. If the goal ultimately fails, the mission is terminated.

```text
Goal failure → Abort mission
```

### B2 — Fixed Mission-Level Retry

The same failed goal is resent up to a fixed retry limit. No diagnosis is performed.

```text
Goal failure → Retry same goal → Retry limit → Abort mission
```

### B3 — Fault-Aware Mission Recovery

The failure is classified using ROS 2 Humble-compatible diagnostics and a deterministic recovery action is selected.

```text
Goal failure
     ↓
Diagnosis
 ┌────┼────┐
 F1   F2   F3
 ↓    ↓    ↓
Replan Defer Retry
       ↓
   Continue mission
```

This is the proposed method being evaluated.

---

## 6. Fault Model

Only three fault classes are included in the current experiment.

| Fault | Condition | Expected response |
|---|---|---|
| **F1** | Persistent route blockage | Replan |
| **F2** | Structurally unreachable goal | Defer / abort goal |
| **F3** | Temporary navigation disturbance | Retry |

### F1 — Persistent Blockage

A static obstacle remains across the planned route for the trial. This tests whether the system can distinguish a persistent blockage from a temporary failure.

### F2 — Unreachable Goal

A mission goal is placed inside an occupied or lethal costmap region. This tests whether an impossible goal can be isolated without unnecessarily terminating the rest of the mission.

### F3 — Temporary Disturbance

A temporary obstacle is spawned during execution and removed after a fixed duration. This tests recovery from a transient navigation problem.

---

## 7. Fault Diagnosis on ROS 2 Humble

The experiment does not depend on newer Nav2 typed error codes. Diagnosis uses signals available in the Humble stack:

1. `GetCostmap` checks the goal cell.
2. A lethal goal cell is classified as **F2**.
3. Otherwise, `ComputePathToPose` checks current reachability.
4. No valid path is classified as **F1**.
5. A valid path after failure is classified as **F3**.

The classification procedure is deterministic and identical across trials.

---

## 8. Fixed Experimental Parameters

| Parameter | Value |
|---|---:|
| Retry limit | 3 |
| Replan limit | 2 |
| Defer limit | 1 |
| Goal timeout | 120 s |
| Mission timeout | 600 s |
| Nav2 lethal cost threshold | 253 |

These values are frozen before the main experiment and recorded with every run.

---

## 9. Experimental Campaign

A nominal test is performed first to confirm that all three policies behave consistently when no fault is present.

### N0 — Nominal

- B1: 20 trials
- B2: 20 trials
- B3: 20 trials

No fault is injected.

### F1 — Persistent Blockage

- B1: 20 trials
- B2: 20 trials
- B3: 20 trials

### F2 — Unreachable Goal

- B1: 20 trials
- B2: 20 trials
- B3: 20 trials

### F3 — Temporary Disturbance

- B1: 20 trials
- B2: 20 trials
- B3: 20 trials

### Total

```text
3 policies × 4 conditions × 20 trials = 240 missions
```

The campaign is automated to keep the initial conditions and experiment procedure consistent.

---

## 10. Experimental Controls

For a fair comparison, each run keeps the following fixed:

- map and robot model;
- starting pose;
- mission goals;
- Nav2 parameters;
- obstacle configuration;
- fault-injection timing;
- recovery limits;
- physics settings;
- software revision.

The policy and prescribed fault condition are the intended experimental variables.

Each run records the Git commit used for the experiment.

---

## 11. Metrics

### Primary

**Mission Completion Rate**

```text
successful missions / total missions
```

### Secondary

- Task completion rate
- Recovery success rate
- Recovery time
- Mission completion time
- Retry count
- Replan count
- Defer/abort count

Operator-required resets may be logged, but they are not a headline metric for this simulation-only study.

---

## 12. Data Logging and Analysis

Each trial produces one append-only CSV record containing the policy, fault condition, timestamps, success status, recovery actions, Git commit and timing information.

Key analysis includes:

- percentage and 95% Wilson confidence intervals for completion proportions;
- mean ± standard deviation or median/IQR for timing, depending on the distribution;
- Fisher's exact test for appropriate proportion comparisons;
- Mann-Whitney U for appropriate timing comparisons;
- significance level `alpha = 0.05`.

Results that are not statistically significant will be reported as such.

---

## 13. RCAAI Scope

### Included

- ROS 2 Humble
- Gazebo 11
- Nav2
- SLAM Toolbox
- AMCL
- Multi-goal mission execution
- Mission-level recovery
- Deterministic fault classification
- Controlled fault injection
- Repeated simulation trials
- Quantitative comparison

### Excluded from the paper

- Autonomous docking
- Battery/energy optimization
- YOLO or deep-learning perception
- Semantic navigation
- Dynamic object following
- Custom SLAM algorithms
- Replacement of Nav2 planning/control
- Reinforcement learning
- LLM-based control
- Physical-robot validation in the current campaign

These features may remain in the general VAMR_MSD project, but they are not part of the RCAAI research claim.

---

## 14. RCAAI Research Files

The research branch will add only the files required to implement and reproduce the experiment:

```text
vamr_msd/
├── docs/
│   └── recovery_policy.md
├── config/
│   ├── experiment_params.yaml
│   └── missions/
│       ├── mission.yaml
│       └── mission_f2.yaml
├── launch/
│   └── experiment_launch.py
├── scripts/
│   ├── goal_queue_node.py
│   ├── recovery_policy.py
│   ├── fault_classifier.py
│   ├── fault_injector_node.py
│   ├── trial_logger.py
│   ├── experiment_runner.py
│   └── analyze_results.py
├── worlds/
│   └── fault_f1_persistent.world
└── results/
    └── trials.csv
```

The existing robot description, SLAM, AMCL, Nav2 and general-purpose simulation files remain unchanged wherever possible.

---

## 15. Current Status

### Existing platform

- [x] ROS 2 Humble
- [x] Gazebo simulation
- [x] Differential-drive robot
- [x] LiDAR / SLAM
- [x] AMCL localization
- [x] Nav2 navigation
- [x] Multi-goal queue
- [x] Basic retry

### RCAAI research layer

- [x] Research question defined
- [x] B1/B2/B3 policies defined
- [x] F1/F2/F3 fault model defined
- [x] Recovery-policy module defined
- [ ] Integrate policy into mission manager
- [ ] Implement fault classifier
- [ ] Implement fault injection
- [ ] Implement trial logger
- [ ] Implement experiment runner
- [ ] Run N0, F1, F2 and F3 campaigns
- [ ] Analyze results
- [ ] Write final paper

---

## 16. Research Principle

The RCAAI branch is intentionally narrow. The goal is **not to make VAMR_MSD larger**. The goal is to run one controlled experiment and answer one question with reproducible evidence:

> **When an autonomous mobile robot fails to complete an individual navigation goal, does fault-aware mission-level recovery allow the overall multi-goal mission to continue more reliably than no mission-level recovery or fixed retry?**

Every code change, experiment and result in this branch should support that question.
