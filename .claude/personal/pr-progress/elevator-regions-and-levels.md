## elevator-regions-and-levels (Tigul#20, stacked on #19 door-leaf)

Plan:
- New tests on the multi_story_building fixture; port HasPlanarFreeSpace, Elevator areas/poses, Level/Floor/Room additions, room kinds.

Done:
- Committed 82ab960851 and pushed; PR description updated.
- test_elevator_areas.py (28 tests); whole semantic_digital_twin_test passes; coraplex navigation/door/elevator tests pass.
- Decisions: HasPlanarFreeSpace uses the frame its ground carries (keeps main's surface-frame free space); Level.holding passes the ratio to InsideOf; new tests instead of the map-based milestone ones.

Next:
- Review on the fork; land after #19, then promote to cram2 (new-feature + api-break for the ORM schema).
