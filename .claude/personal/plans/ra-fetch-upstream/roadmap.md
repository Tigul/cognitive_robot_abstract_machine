# Upstreaming the RA-FETCH milestone

- Take each item's code from ra-fetch-milestone, never by cherry-picking its history: cut the branch off the fork's main and bring over only the item's files and tests.
- Bring the Tigul fork's main up to cram2/main before cutting the first item's branch; each item is reviewed on the fork, then promoted to cram2 with /stacked-pr-maintenance.
- Never carry the .claude/ hooks and skills over: cram2/main removed them in favour of the basstler plugin.
- Leave CHANGELOG.md's spatial-types lines as cram2/main has them: ra-fetch-milestone's version is a search-and-replace mistake.
- Bring every touched class and method in line with AGENTS.md while porting it, and ask before departing from a rule.
- Bug fixes go in their own PRs with the bug label, one root cause each; features carry new-feature, and api-break where they rename or change documented behaviour.
- A test that needs ROS, Gazebo or the real Tiago is skipped in CI and paired with a mocked test.
