# Team Plan: KTH EdgeAI Robotics Challenge

*Last updated: Tue 27 Oct 2026, by Hao. Source documents are in `docs/competition/`. Plain-text copies for searching and AI tools are in `docs/competition/text/`.*

## 1. Timeline

| Date | Event | Status |
|---|---|---|
| Wed 7 Oct 2026 | Kick-off (week 0) | Done |
| **~Wed 4 Nov 2026** | **Mid-term milestone (week 4)** | ⚠️ Inferred from the slides; the Rulebook says TBD. **Confirm with the teacher.** |
| ~Wed 2 Dec 2026 | Final competition (week 8) | Inferred; TBD |

### Milestone requirements (Rulebook §9; a simulation demo is enough)
1. Autonomous exploration plus SLAM. No teleop and no manual goals.
2. ArUco target detection, with the pose given in the `map` frame. Output a target list.
3. Autonomous map-based navigation.
4. At least **one Normal Rescue**: enter the rescue zone and stay there continuously.
5. At least **two robots** connected, with their states available to the system.
6. Basic robot–edge communication, plus basic task planning and allocation.

## 2. Current state of the code

| Item | Status |
|---|---|
| Gazebo world (5×5 m, 3 obstacles), diff-drive robot, LiDAR, RGB-D, bridge, RViz | ✅ Organizer baseline |
| SLAM Toolbox config (manual driving) | ✅ Organizer baseline |
| Full Nav2 stack (planner/controller/BT navigator) | ❌ Not in `pixi.toml` yet |
| Frontier exploration | ❌ |
| ArUco markers in the world, plus a detector node | ❌ (OpenCV 4.12 is already in the Pixi env) |
| Multi-robot (namespaces, 2–4 robots) | ❌ (one robot, global topic names) |
| Coordinator, rescue-zone logic, LLM | ❌ |

## 3. Workstreams (5 people)

| # | Workstream | Milestone deliverable | Owner |
|---|---|---|---|
| 1 | **Navigation & Exploration** | Nav2 on the live SLAM map; frontier exploration (e.g. [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) or our own) | _TBD_ |
| 2 | **Perception & Arena** | Test worlds with ArUco markers; detector → pose in `map` → de-duplicated target list | _TBD_ |
| 3 | **Multi-robot simulation** | Spawn 2+ namespaced robots (`/robot1/...`, `/robot2/...`); multi-robot SLAM / map merge for Task 1 | _TBD_ |
| 4 | **Coordinator & Rescue logic** | Robot/target state manager, task allocation, rescue-zone dwell monitor, run records | _TBD_ |
| 5 | **Edge comms, LLM & Integration** | Robot↔edge interface, top-level launch files, logging/rosbag; later the LLM → structured mission plan (Task 2B) | _TBD_ |

## 4. Team conventions (agree on these and edit as needed)

- **Repos and branches:** each person develops on their own branch or fork. `main` in the shared repo is the integration branch and is only updated through PRs. `upstream` = organizer repo `dongjw-hit/edgeai_robot_ws`; pull organizer updates from there.
- **`pixi.toml` / `pixi.lock` have one owner.** Request packages from them instead of editing the lockfile on your own branch (lockfile merges conflict). Packages we expect to need:
  - `ros-jazzy-navigation2`
  - `ros-jazzy-nav2-bringup`
  - `ros-jazzy-cv-bridge`
  - `ros-jazzy-tf2-geometry-msgs`
  - `ros-jazzy-teleop-twist-keyboard`
- **New code goes in new packages** under `src/` (e.g. `edgeai_perception`, `edgeai_navigation`, `edgeai_coordinator`). Avoid editing `edgeai_robot_description` unless needed, so organizer updates merge cleanly.
- **Interface contract.** These are placeholders until the organizer publishes the official rules:
  - Target list: `id`, `type` (`normal` / `coop`), `x`, `y`, `yaw` in the `map` frame. Topic name _TBD_.
  - Robot namespaces: `robot1` … `robot4`.
  - Rescue zone: placeholder rule = a circle of radius _TBD_ at distance _TBD_ in front of the marker.
- **Never commit** secrets (`.env`), build outputs, rosbags, or local maps in `/maps/`. See `.gitignore`.

## 5. Plan until the milestone

| Day | Goal |
|---|---|
| Tue 27 Oct | Assign roles; everyone sets up their Linux PC and runs the baseline (SLAM Guide Parts I & III) |
| Wed 28 Oct | Everyone has the baseline running. The `pixi.toml` owner adds the shared packages. Each person starts a package skeleton |
| Thu 29 Oct | Workstreams in progress. **Meeting with the teacher/mentor** (Thu or Fri) |
| Fri 30 Oct | **Team sync** (30–45 min): each person demos their part; agree on the merge order |
| Sat 31 Oct–Sun 1 Nov | PRs into `main`; first integration |
| Mon 2 Nov | **Integration session** (in person): explore → map → detect → navigate → one rescue, with 2 robots connected |
| Tue 3 Nov | **Dress rehearsal**; record a backup video and rosbag; freeze the code |
| ~Wed 4 Nov | Milestone demo |

**After the milestone:** cooperative rescue, 4-robot allocation and replanning, local LLM on the edge node (Task 2B), real-robot validation (book slots early), submission records, and the technical report.

## 6. Questions for the teacher/organizer
1. What is the exact milestone date and format? Is a simulation demo with video/rosbag acceptable?
2. Which ArUco dictionary, marker size, and ID → target-type mapping? Will there be an official simulated arena with markers?
3. Is there a provisional rescue-zone geometry rule and dwell time?
4. Will the organizer provide multi-robot (namespaced) simulation support?
5. What are the edge-node specs, and which LLMs/runtimes are allowed for Task 2B?
6. How and from when can we book physical robot slots?

## 7. Learning resources
- ROS 2 Jazzy: [Beginner CLI tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html), [Beginner client libraries](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html), [tf2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [Pixi](https://pixi.sh/latest/) and [RoboStack](https://robostack.github.io/)
- [Gazebo Harmonic tutorials](https://gazebosim.org/docs/harmonic/tutorials)
- [Nav2 docs](https://docs.nav2.org/): "Navigating while Mapping", Simple Commander API
- [OpenCV ArUco detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
