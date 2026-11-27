## door-leaf (Tigul#19, plan ra-fetch-upstream)

Plan:
- Port the frame + leaf door from ra-fetch-milestone onto main's API; update old-contract tests (api-break).

Done:
- Implementation and tests committed (db31f66be4) and pushed; PR description updated with the migration (door.root is the frame; geometry on door.leaf / body "<name>_leaf").
- Whole semantic_digital_twin_test package passes; coraplex/giskardpy/sage door tests pass.
- Decisions: entity_holding lives on HasParts (PartWholeRelationship is gone on main); robocasa doors use Door.from_leaf like the milestone.

Next:
- User reviews the draft PR on the fork; then promote to cram2 with /stacked-pr-maintenance and set api-break + new-feature there.
