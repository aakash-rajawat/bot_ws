# Belief-Aware Dynamic Navigation — one-week research mini-project

**Status:** DESIGN ONLY — no implementation, trained policy, evaluation result, or CV claim yet.  
**Prepared:** 31 October 2026.  
**Application context:** TU Hamburg / MTEC, Research Associate world modeling for autonomous mobile robots (26926WE1).  
**Intended implementation location:** this ROS 2 Jazzy workspace; prototype code should be confined to a new package or separate clearly delimited module. Do not disturb existing SLAM bringup.  
**Time box:** 2–3 weeks for a rigorous experiment, with a working first milestone after 4–7 days; freeze new features by approximately 21 November 2026 to preserve time before the 11 December TUHH deadline.

## Design revision 2 — 31 October 2026 (authoritative)

**Decision:** Expand the initial one-week plan into a 2–3 week benchmark of four decision-making approaches: conventional navigation, hand-designed uncertainty-aware navigation, tabular Q-learning, and a PyTorch-based Deep Q-Network (DQN). DQN is a **core deliverable**, not merely a stretch. The original 3–4-day prototype becomes a milestone, not the completion date. Deadline for the application remains 11 December 2026. This is still **design only**; none of these methods has been newly implemented or evaluated.

**Why:** The applicant already has a probabilistic robot-estimation stack and uncertainty-aware LiDAR/vision point-cloud geometry. The project should test whether uncertainty features improve *actions*, and whether tabular versus deep RL differ under a common environment and action set. Implementing and training a compact PyTorch DQN (including replay buffer, target network and optimization) provides transferable deep-learning evidence; Stable-Baselines3 can serve as an independent reference/validation implementation.

**Scientific scope:** A *local* predictive world model, not a learned generative environment simulator. Keep the geometry/covariance estimation pipeline and Nav2 as existing modules. Add a moving-object tracker, a short-horizon probabilistic occupancy/risk representation, and an upper-level policy that selects one of {normal progress, slow progress, yield/wait}. A fixed collision supervisor remains in charge of hard stop/timeout safety. The effect of RL is evaluated on high-level sequential decisions, not on raw wheel torque.

**Important modeling distinction:** Individual uncertain 3D LiDAR/vision points are not automatically a calibrated Gaussian obstacle track or dynamic occupancy distribution. Document detection, data association, spatial covariance aggregation, tracking/prediction and frame alignment. Use a transparent simplified local sensor likelihood if point-cluster uncertainty aggregation cannot be established quickly. Train against the observations and beliefs, never simulator-truth features. Gazebo ground truth may be used to compute rewards during training and metrics during evaluation, not policy observations.

**Reusable machine-learning evidence:** PyTorch model definition, minibatch sampling, replay-buffer design, target-network update, optimizer and loss tracking, checkpointing, tensor shape/unit tests, seed control, hyperparameter/config files, evaluation and ablation scripts. A pretrained Stable-Baselines3 model alone is less persuasive than source-backed learning code, although SB3 can be used for comparison. It is legitimate to claim deep RL only once the neural policy has actually been trained and evaluated.

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

## Policies and empirical comparison (revision 2 — required)

**All four** see the same robot/obstacle scenario distributions and use the same base Nav2 low-level controller/safety limits. For tabular RL, numerical features must be discretized, and this information-loss is a reported limitation, not a different underlying sensing source.

1. **Conventional baseline:** Nav2 + standard obstacle observation, no explicit covariance-aware high-level decision rule. Do not allow it to be trivially unsafe; apply the same hard collision supervisor to all.
2. **Uncertainty-aware non-learning baseline:** a transparent rule that uses relative separation, estimated covariance, prediction horizon and risk to set normal/cautious/wait decisions. The main *uncertainty effectiveness* baseline.
3. **Classical RL:** tabular Q-learning over a controlled small discretization of goal-progress, obstacle proximity, predicted risk and covariance; 3 discrete high-level actions. Implement and test from first principles. Must genuinely learn from sequential rewards.
4. **Deep RL — CORE DELIVERABLE:** PyTorch DQN with a small MLP receiving the corresponding *continuous* normalized belief features (roughly 10–20 scalars). Implement replay buffer, online Q network, target network, epsilon-greedy exploration, TD target, minibatch optimizer, gradient clipping, checkpoints, and train/evaluation loop. Validate behavior against official SB3 DQN if useful. SB3 DQN provides vanilla DQN, not Double/Dueling/prioritized replay.
5. **Required ablations:** train separate RL versions *with and without covariance/predicted uncertainty features*. Never declare an ablation by feeding a trained network arbitrary zero covariance inputs without retraining, as that creates distribution shift. Optional: remove predicted obstacle motion and retain only current detections.
6. **Optional fifth learner after the main four are stable:** PPO (a different policy-gradient family), but only if the common observations/actions, experimental budget and quality controls can be preserved. No advantage to adding it merely to display another algorithm name.

**Model input (initial DQN):** normalized goal range/bearing; measured ego speed; position/yaw uncertainty scalars; tracked obstacle relative position/velocity; track covariance summaries; track age/missed detection flag; closest-approach / short-horizon risk from uncertainty-aware rollout; optional summarized occupancy sectors with confidence. Use current robot frame with documented covariance transforms and timestamp alignment.

**Action semantics:** {go at normal allowed speed, go at reduced speed, hold/yield}; actions influence Nav2 speed or a supervised command layer. ROS 2 Jazzy Nav2 supports a controller server speed-limit topic, but its SpeedLimit semantics must be obeyed (0 means **no restriction**, not stop). A separate safe stop/hold action is required, and the collision-monitoring supervisor must never be bypassed.

**Learning objective:** reward target-reaching and forward progress, penalize collisions and near-misses strongly, elapsed time and unnecessary waiting mildly. Avoid rewards from simulator ground-truth state that leak into the *policy observation*. Tuning the exact weights should be a documented experiment, not adjusted after seeing final test outcomes.

**Comparability:** training scenario distributions held constant; independent training seeds; frozen held-out obstacle trajectories/noise conditions; report compute/training interactions, elapsed time and model size. All controllers are evaluated on the *same* paired held-out episodes, with the same physical/navigation safety constraints.

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
       supervised Nav2 speed-limit / hold interface + collision monitor + watchdog
                             │
                    robot navigation
                             │
           logs + seed-controlled offline evaluation
```

Proposed new modules (names are suggestions, not yet created):
- `src/bot_belief_navigation/` ROS 2 package: obstacle belief estimator, short-horizon predictor, decision executor, logging.
- `experiments/belief_nav/` fast Python Gymnasium-compatible simulator, baselines, tabular Q-learning trainer, **PyTorch DQN trainer (required)**, checkpointing, policy comparison, ablation/evaluation scripts, tests and figures.
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

## Delivery schedule — revision 2 (2–3 weeks, finish well before 11 December)

| Phase | Working deliverables and gates |
| --- | --- |
| Days 1–3 | Verify live ROS 2 interfaces and sensor timestamps; define one moving-obstacle scenario; implement a fast seedable Gymnasium environment. Run `check_env`. |
| Days 4–6 | Dynamic-obstacle KF/short-horizon predictor; covariance feature validation; conventional and uncertainty-aware rule baselines. **First MVP**. |
| Days 7–9 | Implement, train and debug tabular Q-learning with train/test splits, learning curves and saved policy. |
| Days 10–13 | Implement **PyTorch DQN** from scratch: replay buffer, target network, epsilon-greedy loop, loss curves, checkpoint. Train on same scenario family. Use SB3 DQN only as reference if needed. |
| Days 14–16 | ROS 2/Gazebo inference bridge and supervised speed/hold execution for learned policies; compare navigation against the Python training environment; solve obvious interface mismatches. |
| Days 17–19 | Held-out paired evaluation under sensor-noise/occlusion/obstacle-motion shifts, covariance ablations, training seeds, plots, failure cases and basic statistics. |
| Days 20–21 | Clean experiment configs, scripts, README, demo video, exact limitations and measured CV evidence; **feature freeze**. |

**Realistic stop hierarchy:** (1) two correct deterministic baselines and honest sensor model; (2) trained tabular learner; (3) trained DQN; (4) ROS/Gazebo validation; (5) advanced point-cloud feature modeling / optional PPO. Do not sacrifice reliability, documentation or deadline buffer to add a third learning algorithm.

**Data throughput strategy:** run RL episode loops in a lightweight headless Python/Gymnasium dynamics and sensor simulator (seconds/minutes), and run deployment-like held-out scenarios in ROS 2/Gazebo. Record the simulator-to-Gazebo abstraction gap; do not claim a policy was trained end-to-end in ROS if only inference occurs there.

**Evaluation design:** at least ~50 paired held-out navigation scenarios per policy × several sensor-quality regimes where compute allows; 3–5 independent RL training seeds if feasible. Report sample sizes and spreads, goal success %, collision %, near-miss %, minimum clearance, time to goal, path length, unnecessary waiting and inference latency. Train-without-uncertainty RL ablations on comparable budgets. Report negative or mixed results without embellishment.

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

**Current status: NOT CV-ELIGIBLE (revision 2, before training).** Do not add as a completed achievement until working code, reproducible results and a demonstrable ROS integration exist.

If successfully implemented, an evidence-backed entry could eventually be:
> **Independent Project — Belief-Aware Dynamic Robot Navigation (ROS 2, Python, [actual RL method]):** Implemented a local probabilistic obstacle model and a learned high-level navigation policy using [verified uncertainty features]; evaluated against [verified baselines] across [number] held-out dynamic scenarios, reporting [measured outcomes].

Fill brackets only using measured results. Prefer improving the existing `bot_ws` project entry or adding one concise sub-entry rather than crowding the two-page TUHH CV. Do not imply the project was completed earlier than its actual date.

**Privacy/licensing:** do not upload the Powell PDF, private application files, certificates, research letters, or confidential ZEISS material into public GitHub.
