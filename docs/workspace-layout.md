# Unified workspace
All editable sources, Windows entry points and competition reference documents
live in this repository. See the root README for the directory table and
tests/docs/test-observation.md for live logs and screenshot handling.
ROS package resources remain in src/edgeai_robot_description.
Build the same source tree with scripts/setup/build-workspace.sh; generated
build/install/log files live in EDGEAI_BUILD_ROOT, not a second editable source copy.
Machine-local state under runtime and tests/results is not versioned.
