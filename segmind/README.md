# Segmind

Segmind is a Python library for segmenting simulation episodes of robotic activities by detecting physical interactions and spatial events. It uses a statechart-based approach to monitor simulation data and trigger events like pick-ups, placings, and containment.

## Core Concepts

Segmind revolves around three main components: **Events**, **Detectors**, and **StateCharts**.

### Events

Events represent significant occurrences in the simulation. They are categorized into several types:

- **Atomic Events**: Basic physical changes.
    - `ContactEvent` / `LossOfContactEvent`: Changes in physical contact between bodies.
    - `TranslationEvent` / `StopTranslationEvent`: Start and end of linear motion.
    - `RotationEvent` / `StopRotationEvent`: Start and end of rotational motion.
- **Spatial Relation Events**: Changes in semantic spatial relations.
    - `SupportEvent` / `LossOfSupportEvent`: When an object starts/stops being supported by another.
    - `ContainmentEvent` / `LossOfContainmentEvent`: When an object enters/leaves a container.
- **Coarse (Interaction) Events**: Higher-level activities composed of atomic events.
    - `PickUpEvent`: Combination of a `TranslationEvent` and a `LossOfSupportEvent`.
    - `PlacingEvent`: Combination of a `StopMotionEvent` and a `SupportEvent`.
    - `InsertionEvent`: Detected when an object passes through a "hole" and becomes contained.

### Detectors

Detectors are the logic units responsible for identifying events. They process the simulation state (poses, contacts) at each tick.

- **Atomic Detectors**: `ContactDetector`, `MotionDetector` (Translation/Rotation).
- **Spatial Detectors**: `SupportDetector`, `ContainmentDetector`, `InsertionDetector`.
- **Coarse Detectors**: `PickUpDetector`, `PlacingDetector`.

### StateCharts

The `SegmindStatechart` orchestrates multiple detectors. It acts as a container that ticks all registered detectors against a shared `SegmindContext`.



## Example Usage

The following example demonstrates how to set up a `SegmindStatechart` to detect events in a simulation world.

```python
from cramph.context import StatechartContext
from segmind.detectors.base import SegmindContext
from segmind.statecharts.segmind_statechart import SegmindStatechart
from segmind.episode_segmenter import EpisodeSegmenterExecutor

# 1. Setup Context and Executor, which adds the SegmindContext with its event logger
context = StatechartContext(world=your_simulation_world)
executor = EpisodeSegmenterExecutor(context=context)
logger = context.require_extension(SegmindContext).logger

# 2. Build and compile the Statechart
statechart = SegmindStatechart().build_statechart()
executor.compile(statechart)

# 3. Simulation Loop
while simulation_running:
    # Update your world state here
    # ...
    executor.tick()

# 4. Retrieve detected events
for event in logger.get_events():
    print(f"Detected {type(event).__name__} at {event.timestamp}")
```

Enjoy segmenting!

