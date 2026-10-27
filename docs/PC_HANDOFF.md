# Handoff Prompt for the Linux PC (Antigravity)

Antigravity conversations don't sync between machines. After `git pull` on the PC, open this repo in Antigravity and paste the prompt below as the first message. Fill in the `[...]` parts first.

---

```text
Hi! I'm Hao, a student in a 5-person team in the KTH EdgeAI Robotics Challenge
(autonomous multi-robot search-and-rescue: Gazebo Harmonic + ROS 2 Jazzy managed
by Pixi). I'm new to ROS 2, Gazebo and Pixi, so explain what each step does and
why. Prefer running and verifying commands over just telling me what to run.

## Context (read these first)
- docs/PLAN.md: team plan, milestone requirements, workstreams, conventions.
- docs/competition/text/: plain-text Rulebook V0, kick-off slides, and the
  organizer's Simulation & SLAM guide (the original .docx/.pptx are next to it).
- README.md: how the organizer's baseline is built and run.
- The mid-term milestone is probably ~Wed 4 Nov 2026 (to be confirmed). It
  needs: autonomous exploration + SLAM, ArUco detection with map-frame poses,
  map-based navigation, one Normal Rescue, and 2 robots connected. A simulation
  demo is enough.

## My machine
- Linux Mint 22.3 (based on Ubuntu 24.04 "noble"), x86_64, bash.
- `lsb_release -cs` returns a Mint codename, NOT "noble". Any Ubuntu apt repo
  instructions must use the UBUNTU_CODENAME from /etc/os-release instead.
- The Pixi environment already contains Gazebo Harmonic (gz-sim8, gz-tools2,
  ros-jazzy-gz-sim-vendor) and OpenCV 4.12, so don't install Gazebo on the host
  unless the Pixi one fails. Don't source /opt/ros/*.
- Git: origin = my fork https://github.com/hao2z/edgeai_robot_ws (my dev
  branch is `hao_dev`); upstream = organizer repo
  https://github.com/dongjw-hit/edgeai_robot_ws. The team's shared repo/branch
  is: [fill in after team agreement].

## Step 1: get the baseline running (verify each step before moving on)
1. Check the host: /etc/os-release, `uname -m`, GPU and driver (`nvidia-smi` or
   `glxinfo -B`). If OpenGL looks broken (llvmpipe on a machine with an NVIDIA
   GPU), help me install the right driver via Mint's Driver Manager first.
2. Install the basics (curl, git, build-essential) and Pixi if missing.
3. In the repo: `pixi install`, then `pixi run gz sim --version` (expect 8.x),
   then `pixi run gz sim shapes.sdf` as a graphics test.
4. `pixi run colcon build --symlink-install`, then `./start_sim.sh`.
5. Walk me through docs/competition/text/simulation_and_slam_guide.txt
   Part I (verify topics, drive slowly and stop explicitly) and Part III
   (SLAM Toolbox mapping, save a map to /maps/, which is gitignored).
   Explain topics, TF frames (map -> odom -> base_footprint -> sensors),
   use_sim_time and QoS as we go.

## Step 2: my workstream
My workstream is: [e.g. "2 - Perception & Arena" / "4 - Coordinator"; see
docs/PLAN.md §3].
Once the baseline works, help me:
- create my own ROS 2 Python package under src/ (don't modify
  edgeai_robot_description unless necessary),
- build the smallest working version for the milestone,
- write a short README section on how to run it and what topics it uses.

## Rules for working in this repo
- Don't change pixi.toml / pixi.lock without telling me first. One team member
  owns them, and lockfile changes conflict across branches. If a package is
  missing, tell me which one so I can ask the owner.
- Make small commits on my branch with clear messages. Don't push; I'll push.
- Never commit build/, install/, log/, .pixi/, rosbags, /maps/, or .env.
- If something in the organizer docs is TBD (ArUco dictionary, rescue-zone
  geometry, dwell time), use a clearly marked placeholder in a config file
  rather than hard-coding it.
```

---

## What transfers through git, and what doesn't

| Item | How it gets to the PC |
|---|---|
| Source code, `pixi.toml`, `pixi.lock`, `start_sim.sh` | `git pull` |
| Team plan (`docs/PLAN.md`) and this prompt | `git pull` |
| Competition docs (originals and text copies) | `git pull` (`docs/competition/`) |
| Pixi environment (`.pixi/`) | **Not transferred.** Recreate with `pixi install` on the PC |
| Build outputs (`build/`, `install/`, `log/`) | **Not transferred.** Rebuild with `pixi run colcon build --symlink-install` |
| Antigravity chat history | **Not transferred.** Use the prompt above |
| `.vscode/settings.json` | **Not transferred** (gitignored; it held a Mac-only path) |
