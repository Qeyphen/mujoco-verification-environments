# MuJoCo Verification Environments

A collection of hermetic, scriptable evaluation frameworks built using the MuJoCo Python API. Instead of optimizing robotics control loops or rendering visual environments, this repository focuses entirely on the engineering backend of automated task verification—building deterministic "digital cages" that isolate physical specifications and programmatically eliminate agent reward hacking.

## 🛠️ Core Capabilities

* **Multi-Strata Evaluation Pipelines:** Sequential validation layered across **Structural** constants (blueprints), **Static** kinematics passes (`mj_forward`), dynamic **Rollout** tracking (`mj_step`), and parameter **Robustness** sweeps (mass/friction perturbations).
* **Anti-Cheat Architecture:** Programmatic boundaries (Feasibility Shells) designed to catch degenerate LLM or RL agent solutions, such as unjointed structural clones, sky-welded torsos, and ground-wedging clipping exploits.
* **Numerical Sanity Defense:** Automated runtime interception that flags simulation instability, identifying system explosions, velocity spikes, contact abnormalities, and matrix crashes (`NaN`/`Inf` collapses) before process termination.
* **Strict Determinism Enforcers:** Bit-for-bit reproducible trajectory verification lines built on isolated memory state cache management, precise full-state specification, and rigid time-step tracking primitives.

## 📦 Project Directory

* `/tools/model_inspector.py`: A native command-line utility that extracts contiguous memory array allocations and structural constants directly from compiled physics layout objects.
* `/tools/sanity_checker.py`: An automated frame-by-frame diagnostic engine that monitors dynamic rollouts for conservation flaws, deep penetrations, and mechanical degradation.
* `/tasks/damped_pendulum/`: A full implementation of a high-fidelity system specification task, complete with precision log-decrement damping analysis, peak tracking, and adversarial verification cases.
* `/tasks/hopper/`: A complex 3-joint kinematic tree grading module featuring weighted multi-criteria rubrics and mid-run environmental shock testing.
