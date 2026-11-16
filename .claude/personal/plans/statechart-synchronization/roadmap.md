# Synchronizing statecharts across processes and frameworks

## Why this plan exists

A cramph `Statechart` is sent to Giskard once, as an action goal, and Giskard
reports back through action feedback. That covers states and a few structure
changes, but not all of them. The goal is full synchronization, as
semantic digital twin does for worlds (`doc/world_synchronization.rst`):

- **model updates** that add or remove nodes and change their structure;
- **state updates** that carry every node's life cycle, observation and last
  observation state.

Long term, several frameworks share *one* statechart (user, 2026-11-16).
Example: a perception framework provides and ticks perception nodes in its own
tick loop, next to Giskard's motion nodes. So the design has to work for N
peers, not just a client and a server. Where it fits, it shares a base with
the world synchronizer.

## What exists today (checked against source, 2026-11-16, cram2/main 437a7621f6)

**World.**
- `@atomic_world_modification` records replayable `WorldModification`s
  (`apply`/`revert`). The outermost `modify_world()` closes a
  `WorldModelModificationBlock` (`world.py`).
- `WorldModelManager` holds the version, the model change callbacks, the
  pending publications and the publication order.
- `Synchronizer` (`adapters/ros/world_synchronizer.py`) publishes JSON on one
  ROS topic. Each message carries `MetaData` and a sequence number, and the
  synchronizer keeps per-publisher received/applied positions
  (`StreamPosition`, `has_applied`, `record_applied`) and detects lost
  messages.
- `WorldSynchronizer` sends model and state on one topic, so they stay in
  order, and can hold incoming messages back to apply them later.

**Statechart.**
- `ActionFeedbackPublisher` (`giskardpy/middleware/ros2/feedback_publisher.py`)
  sends:
  - the full state arrays on every change;
  - new nodes as a diff that only grows (`nodes_from_to_json` from
    `sent_node_count` on).
- `StatechartStateUpdate` (`state_updates.py`) applies states *by index*. It
  tolerates the client holding more nodes than Giskard (child choices) and
  raises `StateUpdateAheadOfStatechartError` when Giskard holds more. So a
  failure handling strategy that inserts nodes on Giskard breaks the client;
  hence the "only works in a locally ticked statechart" warnings in
  `failure_handling.py`.
- Child choices go client → Giskard on a side topic (`child_choices.py`),
  checked by `first_node_index`.
- Ordering against the world stream is hand-written in giskardpy:
  `PUBLISHED_POSITION`, `required_position`, `CatchingUpWithGiskard`.
- `ReportedStatechart` (cramph) rebuilds a structure copy from the feedback
  for the statechart inspector.

## How a statechart differs from a world, and what that decides

1. **Ownership per node.** A world is peer to peer and every peer replays
   everything. A statechart node is ticked in exactly one place, so every node
   gets an owner:
   - *tick owner*: evaluates its transitions and observation, runs its
     callbacks, and is the only one publishing its state;
   - *structure owner*: may change a composite node's children.

   The two can differ: a choosing node is ticked on Giskard, but the client
   chooses its child. Default tick owner: the peer whose context provides the
   node's `required_context_extensions`.
2. **Atomic changes sit below expansion.** `add_node` expands composites, and
   expansion depends on the context and the world, so replaying it could give
   a different result. The atomic changes are: register a node, set a
   transition condition, set a composite's children, restore chosen children,
   remove nodes, request a reset, request a rebuild. The outermost `modify()`
   block (`StatechartModification`, which already nests and rolls back) is
   the counterpart of a `WorldModelModificationBlock`.
3. **Indices stay local.** Peers add nodes concurrently and `_renumber`
   renumbers on removal. Everything on the wire names nodes by `_node_id`,
   and a state update names the structure position it belongs to.
4. **Every peer holds the whole structure, but only its own nodes as real
   classes.** Unknown classes deserialize as stand-ins, like the nodes
   `create_structure_copy` builds.
5. **Dependencies between streams.** Nodes refer to world entities, so a
   statechart change requires a world position. That becomes a field of the
   message.
6. **Transport is a topic per statechart, not action feedback.** Feedback is a
   1:1 channel. The action can stay for handing a statechart over and for the
   result.
7. **Settling within a tick is local.** `pass_limit` settling holds only among
   a peer's own nodes; across peers a change arrives with the next message.

## Gap found while investigating (by reading the code; not yet proven by a test)

`_insert_at` → `_wire_children` rewrites the start condition of the existing
next sibling in a `Sequence` (`composites.py`). `nodes_from_to_json` writes
only the conditions of the *new* nodes, so a copy keeps the old condition.
`statechart-modification-records` starts with a failing test for this.

## What generalizes

| Part | Generic? | Home |
|---|---|---|
| MetaData (origin id), StreamPosition, Message + sequence number, applied/received bookkeeping, lost messages, required positions | Yes, cleanly | krrood (ROS-free) |
| Modification (apply/revert), blocks, nested modification log | In shape only; the world's is tied to `_world_lock`, FK compilation, `delete_orphaned_dofs`, the memoization cache | krrood, deferred |
| ROS topic synchronizer | Yes, behind an interface for: lock, deserialization kwargs, publication order, origin id | `semantic_digital_twin.adapters.ros` |
| Change callbacks | In principle; bound to `_world` via `WorldEntityWithClassBasedID` | follows the synchronizer |

## Cheap now, expensive later (do in the first PRs)

- Node ids on the wire; indices never leave the process.
- Statechart id and origin on every message.
- An owner on every node, instead of "server" and "client" roles in the
  protocol.
- Stand-ins for unknown node classes.
- A topic synchronizer instead of feedback payloads.

## Open decisions (to discuss with colleagues)

1. Who owns the structure: the plan executor for everything it sent, or each
   framework for the subtrees it ticks?
2. Should conditions across peers be restricted to life cycle states?
3. `MetaData.world_id` → a generic origin id changes a serialized format
   (api-break). Rename it, or keep the name?
4. Where does the statechart synchronizer live, given that ROS is a soft
   dependency of cramph?
5. Is the generic modification log (`generic-modification-log`) in scope at
   all?

Code owners to involve: cramph, giskardpy and semantic digital twin
(`.github/CODEOWNERS`).

## Conventions for this plan

- Every item is a pull request into cram2/main, opened as a draft from the
  Tigul fork.
- Items marked api-break describe the break and how to migrate before review.
- No tracking issue was created: the user's notes rule out subscribing to
  pull request or issue activity.
