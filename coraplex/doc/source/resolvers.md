# Resolving Designators

A designator description expresses intent and constraints; resolving (grounding) it turns it into a concrete,
executable instance based on the current world and robot state. In CoraPlex this happens automatically while a plan is
performed, so there is usually no need to resolve designators by hand.

## How resolution works

Resolution is deferred to execution time so that the query always sees the correct world state. When a plan reaches an
underspecified step, the {class}`~coraplex.plans.executables.UnderspecifiedExecutable` grounds it only after every
preceding executable has run and mutated the world. Candidates are tried in order until one executes without raising a
{class}`~coraplex.plans.failures.PlanFailure`; if none succeed, the step fails. This late grounding is what lets a plan
adapt to objects that moved, a torso that was already raised, or an object already held in the gripper.

```{figure} _static/images/underspecified-template.png
---
width: 800px
align: center
alt: A designator template whose domain is a location, expanded into concrete candidates
---
A description stands for many concrete instances: one per combination of its fields' domains. A
domain does not have to be a list — a {class}`~coraplex.locations.base.Location` is an iterable of
poses, so the costmap keeps generating candidates lazily.
```

```{figure} _static/images/lazy-resolution.png
---
width: 800px
align: center
alt: Pulling one candidate at a time, retrying the next one after a failure
---
Candidates are pulled one at a time and tried. A pose only counts as reachable once an earlier
step of the same plan has made it so, which is why grounding cannot happen up front.
```

## Location designators

Location designators are resolved into 6D poses by the pose-generator backends in {mod}`coraplex.locations`. The
backends in {mod}`coraplex.locations.backends` build and combine costmaps (see {doc}`costmap`) for criteria such as
reachability, visibility and occupancy, and the factories in {mod}`coraplex.locations.factories` assemble the location
for a given task. Sampling a costmap yields candidate poses, which are then validated by the validators in
{mod}`coraplex.locations.pose_validator` (for example {class}`~coraplex.locations.pose_validator.IsObjectReachableBy`).

```{figure} _static/images/location-pipeline.png
---
width: 800px
align: center
alt: From target and arm, through merged costmaps, to validated candidate poses
---
A location designator turns a target into a lazy stream of candidate poses: costmaps are merged
into a map to sample from, and each candidate is validated before it is yielded.
```

## Customising resolution

To change how a particular kind of location is generated, provide or extend a pose-generator backend in
{mod}`coraplex.locations` rather than adding a separate resolver module. Custom resolution logic should keep the same
interface as the designator it grounds so it stays a drop-in replacement.
