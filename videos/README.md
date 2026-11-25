# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Link | Notes |
|---|---|---|---|---|
| 2026-11-25 | S3 | Detect and record hump | <https://youtu.be/wJ2n_ZApeKov> | Hump detected, height recorded |
| 2026-11-25 | S2, S3 | Line tracking full run, then lift and stop | <https://youtu.be/qosbVwFpzFA> | Full run on the line; car lifted, motors stop |
| 2026-11-25 | S5 | Obstacle detection, width and avoidance | <https://youtu.be/EHOQbQYNOlM> | Stops at the obstacle, width recorded, goes round and finds the line |
| 2026-11-25 | S2, S5 | Barcode, then left turn at the junction | <https://youtu.be/yLLNN38ZBSk> | Decoded letter mapped to left |
| 2026-11-25 | S2, S5 | Barcode, then right turn at the junction | <https://youtu.be/B7ACqwCMzwQ> | Decoded letter mapped to right |
| 2026-11-25 | S2, S5 | Barcode, then U-turn at the junction | <https://youtu.be/MOKXbKRIUso> | Decoded letter mapped to U-turn |
