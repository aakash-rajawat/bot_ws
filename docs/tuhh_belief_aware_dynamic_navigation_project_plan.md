# Belief-Aware Dynamic Navigation — one-week research mini-project

**Status:** DESIGN ONLY — no implementation, trained policy, evaluation result, or CV claim yet.  
**Prepared:** 31 October 2026.  
**Application context:** TU Hamburg / MTEC, Research Associate world modeling for autonomous mobile robots (26926WE1).  
**Intended implementation location:** this ROS 2 Jazzy workspace; prototype code should be confined to a new package or separate clearly delimited module. Do not disturb existing SLAM bringup.  
**Time box:** 3–4 days for minimum viable demonstrator; absolute maximum 7 days.

## The research question

> Can a mobile robot improve navigation decisions around a moving obstacle by explicitly using uncertainty in robot state and obstacle predictions, rather than relying on point estimates alone?

**Hypothesis:** A decision policy receiving *belief-state features* (pose uncertainty, obstacle-track covariance, probability of encountering the obstacle, and local map confidence) can learn when to proceed, slow, or temporarily wait/reobserve. Compared with a certainty-equivalent policy, it may reduce collisions and near misses under uncertain observations, potentially at the cost of travel time. Improvement is **not presumed**; it must be tested.

This is a **small local probabilistic world representation and learned high-level navigation policy**, NOT a general-purpose learned world model, full POMDP solver, physical deployment, or demonstrated method from the advertised TUHH project.

## Link to existing bot_ws — reuse, do not rebuild

Existing documented components:
- ROS 2 Jazzy + Gazebo differential-drive robot, URDF/Xacro, stereo cameras, 360-degree LiDAR, IMU, wheel encoders.
- `src/bot_navigation`: Nav2 planner/controller/behavior-tree configuration.
- `src/bot_mapping`: SLAM Toolbox launch/configuration.
- `src/bot_localization`: GTSAM factor-graph fusion and marginal uncertainty (check actual output covariance fields and freshness before using).
- `src/bot_controller`: differential-drive commands, velocity limits, watchdog.
- `src/bot_evaluation`: ground-truth odometry error.

The first integration spike must verify timestamps, QoS, data fields, odometry covariance, `/scan` availability, Nav2 action interface and ability to spawn/move a single simulated obstacle. Existing sensor uncertainty **does not by itself** imply probabilistic action selection.

## Precise bounded scenario

- One differential-drive robot navigating to a fixed goal in a small mapped indoor environment.
- One moving pedestrian-like obstacle crosses a corridor; vary its velocity, trajectory, occlusion and sensor noise between runs.
- The robot gets only noisy observed state in the policy; Gazebo ground truth is hidden from the policy and used exclusively to compute outcomes and offline evaluation.
- Stationary obstacles can be read from an existing occupancy grid / Nav2 costmap. Do NOT implement a new complete SLAM system.
- A local dynamic-world state is estimated from LiDAR detections or simulated noisy detections in the first fast prototype. Use a small constant-velocity Kalman model for tracked obstacle position and covariance; propagate forward over a short prediction horizon. Document simplistic association and model limitations.
- Optional static-world uncertainty: occupied/unknown grid-cell probabilities or local entropy (only if the integration is straightforward). The *minimal* uncertainty inputs are robot-pose covariance plus predicted obstacle covariance.

## Decision-model specification (Powell, Chapters 9–11)

- **State/belief at time t:** goal distance/bearing, current speed, ego pose estimate and covariance summary, dynamic-obstacle position/velocity belief and covariance summary, predicted closest approach / approximate collision probability, optionally local occupancy uncertainty.
- **Decision:** high-level discrete choice from {proceed at nominal speed, proceed cautiously/slow down, wait to collect another observation}. Existing low-level Nav2/controller handles path following; a safety supervisor always enforces collision margins and command timeout.
- **New information:** noisy range/bearing detections, updated localization estimate, moving-obstacle behavior.
- **Transition:** simulated robot and obstacle motion plus Kalman predict/update for belief variables.
- **Objective:** successful goal arrival with low collision/near-miss rate and reasonable traversal time; optional small penalty for unnecessary waiting. Do not optimize reward on unavailable ground truth at inference.
- **Uncertainty types:** observational noise, inferred obstacle state uncertainty, prediction/transition uncertainty, localization uncertainty. Sensor covariance is a *part* of this, not the complete definition of world-model uncertainty.

**Illustrative risk feature**: compute a finite-horizon closest-approach distribution from relative robot/obstacle predicted positions and covariances. A simple normalized separation such as `d_hat / sqrt(n^T (Sigma_robot + Sigma_obstacle) n)` can indicate risk in a chosen separation direction. This is an approximation, **not** a calibrated collision probability; if displaying `p_collision`, obtain it using Monte Carlo samples with clearly specified geometry and time horizon.

**Critical information-flow rule:** select actions using only information available at the decision time. Ground truth must never leak into policy observations. Training-time reward may use simulator collision and success signals, but deployment decisions may not.

## Policies and empirical comparison

1. **Certainty-equivalent baseline:** normal navigation using point-estimated moving-object position / simple conservative stop radius, no covariance feature.
2. **Hand-engineered uncertainty-aware baseline:** same route controller but speed/wait choices depend explicitly on predicted risk and uncertainty. This tests whether uncertainty helps independently of RL.
3. **Learned policy (the actual RL component):** implement tabular Q-learning first over discretized risk/covariance/distance features, OR use Stable-Baselines3 DQN (compact numeric belief features, discrete high-level actions) if the fast simulator is stable by day 3. A DQN is a **stretch** if ROS integration or training threatens the one-week cap.
4. **Ablation:** trained policy with uncertainty features removed or zeroed, evaluated on the exact same held-out scenarios. To avoid invalid comparisons, retrain a separate policy without uncertainty features rather than merely zeroing inputs to a fixed network.

**Avoid**: claiming PPO/SAC/DQN experience unless actually implemented and trained; claiming 'world-model learning' from a fixed Kalman filter; presenting this as full autonomy or a hospital-validated system.

## Implementable architecture

```text
Gazebo moving-obstacle scenario
       │
LiDAR / robot odometry (ROS 2)
       │
Detection + noise model ───────────┐
       │                          │
Dynamic obstacle KF         Robot pose covariance
       │                          │
       └───── local belief / predicted encounter risk ─────┘
                             │
                   high-level decision
                   (rule, Q-table, DQN)
                             │
       safe Nav2 velocity/goal interface + watchdog
                             │
                    robot navigation
                             │
           logs + seed-controlled offline evaluation
```

Proposed new modules (names are suggestions, not yet created):
- `src/bot_belief_navigation/` ROS 2 package: obstacle belief estimator, short-horizon predictor, decision executor, logging.
- `experiments/belief_nav/` fast Python Gymnasium-compatible simulator, baselines, Q-learning trainer, optional DQN trainer, evaluation script, tests, and figures.
- `docs/` research statement, honest limitations and short screen-recorded demo.

Use fast headless non-ROS simulation for repeated RL training; use ROS 2/Gazebo for demonstration and final held-out testing. The two environments will not be identical; report transfer assumptions and discrepancies.

## What makes the project scientifically informative?

- Noisy sensory estimates actually enter the policy state and alter decisions.
- The environment changes: at least one moving obstacle with stochastic trajectory/observations.
- Both learned and hand-designed decision rules use the **same** perception inputs.
- Hold-out seeds and noise levels are separated from training.
- Conduct paired trials (same scenario seeds) across baselines; report medians and spreads as well as failure counts.
- A negative result is still scientifically usable if analyzed: e.g., learned policy over-waits; localization uncertainty is under-calibrated; sensor dropout defeats the model.

## Tests and success criteria (proposed, not results)

- Unit tests: KF predict/update, positive semidefinite covariance, increasing uncertainty during observation gaps, risk monotonicity for fixed geometry, no ground-truth leakage.
- End-to-end: the robot can navigate to a goal and correctly alter high-level behavior when approaching the moving obstacle under distinct uncertainty levels.
- Evaluate at least 30 paired held-out scenarios per policy and at least 3 sensor-noise/occlusion conditions **if runtime permits**. Clearly state actual number of trials.
- Report: goal-success %, collision %, minimum clearance / near misses, traversal time, number of waits, path length, and inference latency. Include uncertainty calibration if enough ground truth is available.
- Desired hypothesis test, **not acceptance threshold**: learned belief-aware policy reduces collision/near-miss incidence compared with a certainty-equivalent policy while preserving acceptable completion time. If no gains, publish the comparison and limits honestly.

## Hard time box

| Day | Deliverable and stop condition |
| --- | --- |
| 1 | Mathematical model; 1 moving-obstacle scenario; fast simulator with deterministic seeds and baselines. |
| 2 | Obstacle Kalman filter, covariance / risk outputs; tests; rule-based risk-aware policy. |
| 3 | Tabular Q-learning trainer, learning curve, held-out baseline comparison. **Minimum viable RL milestone.** |
| 4 | ROS 2 interface spike: publish/subscribe, inspect live covariance, execute safe speed/wait decisions in Gazebo. This ends the **3–4 day minimum version** if successful. |
| 5 | Better ROS integration; new moving-obstacle world; optional compact DQN if core is already stable. |
| 6 | Randomized held-out evaluations, noise ablation, plots, failure analysis. |
| 7 | README, reproducible commands, short video/GIF, comparison table, accurate CV-ready evidence. **Stop expanding scope.** |

**Fallback when time runs short:** retain the runnable tabular Q-learning + uncertainty-aware hand-designed baseline and Python simulator, show one ROS 2 example, and document that the learned policy was not demonstrated on hardware. Avoid a rushed deep-RL model without evaluation.

## Powell 2022 reading map (do not commit the copyrighted PDF)

*Warren B. Powell, Reinforcement Learning and Stochastic Optimization: A Unified Framework for Sequential Decisions* (Wiley 2022). Relevant parts:
- **Ch. 1**: problem before algorithm; state/decision/new information/transition/objective.
- **Ch. 2**: stochastic shortest paths, Markov decisions, RL, robust optimization.
- **Chs. 8–9**: state-dependent problems, belief state, decisions and transition.
- **Ch. 10**: observation, inferential, predictive, transition and implementation uncertainty; Monte Carlo.
- **Ch. 11**: compare policy classes instead of assuming RL is required.
- **Ch. 12**: parameterized/direct policy functions; comparator.
- **Ch. 14**: Markov decisions and Bellman equation.
- **Ch. 17**, especially 17.2.2 and 17.5: Q-learning and exploration/exploitation; primary RL method.
- **Ch. 19**, especially 19.3.1 and 19.4: managing risk and policy evaluation; optional risk/lookahead baseline.
- **Ch. 20.3**: POMDP perspective; theoretical extension, not a required implementation.

Book is an excellent **decision-modeling** base and covers classical Q-learning, but is **not** a practical ROS 2 tutorial or a complete modern DQN training recipe. Supplement with official Stable-Baselines3 DQN docs if using DQN: https://stable-baselines3.readthedocs.io/en/master/modules/dqn.html

## Lab-specific research context (independent verification)

- TUHH MTEC vacancy 26926WE1: new world-modeling project, uncertainty handling and RL *considered*, experimental mobile navigation on different robots; exact method/platform unspecified. Official job text also reproduced by Bundesagentur für Arbeit: https://www.arbeitsagentur.de/jobsuche/jobdetail/20536-0hemzrsqie-S
- MTEC-affiliated 2019 paper, **Gromniak & Stenzel, Deep Reinforcement Learning for Mobile Robot Navigation**: https://doi.org/10.1109/ACIRS.2019.8935944 ; simulated changing-obstacle navigation and differential/holonomic robot kinematics.
- MTEC **Grube et al., A mobile robotic approach to autonomous surface scanning in legal medicine** (published online 2025, volume 2026): https://doi.org/10.1007/s11548-025-03507-w ; differential-drive mobile base + UR3 6-joint arm + Azure Kinect, ROS 1/MoveIt! in the described setup.
- MTEC **Grube et al., An AI-Based Mobile Robot for Autonomous Ultrasound in Legal Medicine** (2026): https://doi.org/10.1109/ACCESS.2026.3713969 ; Robotnik RB-Theron mobile platform with UR3 arm (MORUSS).
- Other TUHH platforms such as the MoMo mecanum robot belong to other institutes; never attribute them to MTEC without explicit evidence.

## CV / application integrity

**Current status: NOT CV-ELIGIBLE.** Do not add as a completed achievement until working code, reproducible results and a demonstrable ROS integration exist.

If successfully implemented, an evidence-backed entry could eventually be:
> **Independent Project — Belief-Aware Dynamic Robot Navigation (ROS 2, Python, [actual RL method]):** Implemented a local probabilistic obstacle model and a learned high-level navigation policy using [verified uncertainty features]; evaluated against [verified baselines] across [number] held-out dynamic scenarios, reporting [measured outcomes].

Fill brackets only using measured results. Prefer improving the existing `bot_ws` project entry or adding one concise sub-entry rather than crowding the two-page TUHH CV. Do not imply the project was completed earlier than its actual date.

**Privacy/licensing:** do not upload the Powell PDF, private application files, certificates, research letters, or confidential ZEISS material into public GitHub.
