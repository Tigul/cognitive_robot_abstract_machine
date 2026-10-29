## ra-fetch-milestone: Tiago says its belief state while using the elevator
Done:
- 149ae77cc8 (user commit): `PublishMessageMotion` + `TiagoTopic.TEXT_TO_SPEECH`.
  Its test file test/coraplex_test/test_designator/test_publish_message_motion.py is
  still untracked.
- 2026-10-29, uncommitted: `AbstractRobot.text_to_speech_topic` (Tiago -> tts),
  `Level.spoken_name`, coraplex `SayAction` (core/speech.py; silent without a topic or
  ROS node), `PassDoorAction.waiting_announcement`, `ElevatorRideAnnouncement` with 4
  sentences in `ElevatorNavigation`, `ElevatorBoardingAnnouncement` in the waiting
  demo. Fixed the unused `tiago_apartment_world` fixture (duplicate Tiago). Tests pass.
Known: test_elevator_navigation[stretch] fails on the baseline too (time limit).
Next: commit when the user asks; try on the real Tiago (`ros2 topic echo /tts`).
