## ra-fetch-room-navigation-demo: Tiago drives from a start room to a sampled target room
Plan: /home/jdech/.claude/plans/i-want-a-demo-keen-sunrise.md (approved 2026-10-29).
Done 2026-10-29, uncommitted:
- ARISE office: per-room Floor + Room (AriseOfficeRoom StrEnum), `get_room` replaces
  `ground_floor_slab`; `Room.floor_centre`, `Floor.extent`, `Floor.under`.
- `HasPlanarFreeSpace` mixin (Floor via HasSupportingSurface, Level via its region);
  PathPlanningNavigateAction plans over the Level holding the robot (`Level.holding`).
- `Elevator.pose_in_front_of_doors` replaces the waiting demo's `waiting_pose`.
- experiments/tiago_room_navigation_demo: NavigateToRoomAction, demo, trials (+report).
- Tests pass; 50-trial run: 50/50 succeeded. ORM regenerated (new HasPlanarFreeSpaceDAO).
Known: ra_fetch_elevator_people 2 failures and test_elevator_navigation[stretch] fail on baseline.
Next: commit / draft PR when the user asks.
