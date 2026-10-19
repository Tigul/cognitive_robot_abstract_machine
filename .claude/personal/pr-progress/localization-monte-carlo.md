## localization-monte-carlo — PR Tigul/cognitive_robot_abstract_machine#14 (base: localization-raytracer-open3d)

Third PR of the SemDT lidar localization stack (design doc:
https://claude.ai/code/artifact/c8058971-9468-4c9c-a6f6-65c6875262e7).
Stack: semdt-localization <- #12 <- #13 <- #14 this <- localization-ros-node.

Plan: ROS-free semantic_digital_twin.localization: ParticleSet, PoseHistogram,
motion models (diff/omni), BeamMeasurementModel, AdaptiveResampling (KLD),
RandomParticleInjection (augmented MCL), ScanAlignment (planar ICP),
MonteCarloLocalization + PoseEstimate. RayTracer.measure_distances and
find_closest_surface_points; SimulatedLidarSource uses measure_distances.

Done (2026-10-19): commit cfe1b32296, pushed, draft PR #14. Tests in
test/semantic_digital_twin_test/test_localization (room scenario in room.py).
Deviation from AMCL flagged in PR: product of beam probabilities (sum of logs)
instead of 1+sum p^3; injection fed the geometric mean beam probability under
posterior weights. 10 seeds x 3 scenarios all end within 7 cm.

2026-10-19: apartment comparison (scratchpad apartment_compare.py, not in PR):
1+sum p^3 fails everything (also with per-update resampling, without /N);
product 8/10 global (wrong niche), product^0.2 30/30. Commit d592bcccfc adds
BeamMeasurementModel.likelihood_exponent=0.2 (fit stays untempered), pushed,
PR body updated. Room sweep now within 3 cm. Design doc still to update.

Next: localization-ros-node (milestone 4) on top of this branch: LidarReading
stamp, stamped odometry history, LocalizationNode (twin copy, tf map->odom,
/initialpose), motion model chosen by drive connection type.
