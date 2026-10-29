## ra-fetch-milestone: publish to a topic from a plan (Tiago TTS)
Done (2026-10-29, uncommitted):
- Fast-forwarded to aicor-solutions/ra-fetch-milestone (brings TiagoTopic).
- `TiagoTopic.TEXT_TO_SPEECH = "tts"` (std_msgs/String) in robots/tiago.py.
- `PublishMessageMotion(topic_name, message)` in coraplex motions/misc.py, wrapping
  giskard's existing `PublishOnStart` (no new giskard task needed).
- Tests: test/coraplex_test/test_designator/test_publish_message_motion.py (pass);
  ORM regenerated.
Next: commit when the user asks; check on the real Tiago with `ros2 topic echo /tts`.
