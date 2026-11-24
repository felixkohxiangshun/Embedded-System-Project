# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds (a full-track run may take longer). One test case per video.
- **Say what it is.** Show or say the test case number (e.g. `V3`) and its
  name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
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

## Test case videos

One video per test case. After uploading, replace *link to be added* with the
unlisted YouTube link and fill in the date and commit. Each row says what the
video must show for the test to count as passed.

### Required test cases

| # | Test case | Set-up | What the video must show (pass) | Date | Commit | Link |
|---|---|---|---|---|---|---|
| V1 | Detect and record hump | Straight line with a hump on it; dashboard open | Car drives over the hump without stopping; the console prints `[IMU] hump ... mm`; the dashboard's IMU card shows Last hump and Tallest hump with the height, and Humps measured goes up by 1. Measure the real hump with a ruler on camera to compare | | | *link to be added* |
| V2 | Line tracking full run, then detect lift and stop | Full track loop; dashboard open | Car is started (button or dashboard Start) and follows the whole line, through bends, without leaving it. Then pick the car up while it drives: the wheels stop at once, the dashboard shows Lifted = yes and the IMU state LIFTED | | | *link to be added* |
| V3 | Obstacle detection, width and avoidance | A box of known width (say it on camera) on the line; dashboard open | Car stops about 30 cm from the box; the servo sweeps (centre, left, back, right); the dashboard shows the obstacle width and the decision (go LEFT / go RIGHT); the car drives round the box on the 45° dog-leg without touching it and picks the line up again (Last return leg: stopped on the line). Car card shows Last obstacle width | | | *link to be added* |
| V4 | Barcode then LEFT turn at the junction | Mapping sent first with a letter for Left (show the dashboard's Junction letters); that barcode before a cross junction | Car slows and reads the barcode; the dashboard shows Last read and Stored for next junction = that letter; at the junction the car turns left and follows the new line | | | *link to be added* |
| V5 | Barcode then RIGHT turn at the junction | As V4, with the letter mapped to Right | As V4, but the car turns right | | | *link to be added* |
| V6 | Barcode then U-TURN at the junction | As V4, with the letter mapped to U-turn | As V4, but the car turns round (180°) and follows the line back | | | *link to be added* |

### Suggested extra test cases

| # | Test case | Why it is worth showing | What the video must show (pass) | Date | Commit | Link |
|---|---|---|---|---|---|---|
| V7 | Junction mapping changed over MQTT | The mapping must come from the PC, not be hard-coded (instructor's requirement) | Run the same barcode twice: first mapped to Left, then change the boxes on the dashboard, press Send mapping (the console prints `[map] ...`), and the same barcode now gives Right. Shows the letters are not fixed in the code | | | *link to be added* |
| V8 | Barcode read in the other direction | Code 39 is read both ways; the track may be driven either way | The car crosses a barcode backwards (turn the code round, or drive the loop the other way) and still reads the same letter (dashboard Last read) | | | *link to be added* |
| V9 | Straight-ahead letter, and an unmapped letter | Covers the fourth junction move and the safe default | A barcode mapped to Straight, then one not in the mapping: at the junction the car goes straight on both times | | | *link to be added* |
| V10 | Line lost and found again | Shows line recovery (faded or broken line, sharp bend) | Cover part of the line with white paper: the car keeps going, searches, and picks the line up again; the dashboard Line mode goes COAST/SWEEP and back to TRACK | | | *link to be added* |
| V11 | Sensor or link failure handled safely | The requirement change: a part that fails or gives invalid data must not make the car unsafe | (a) Unplug the IMU while the car runs: the dashboard shows the IMU fault (Faults: IMU, IMU state SENSOR_FAULT) and the car keeps following the line; plug it back in and the IMU recovers. (b) Stop the broker: the page shows Broker offline, the car keeps driving, and the data returns when the broker is back | | | *link to be added* |
| V12 | Start and Stop from the dashboard | Remote control over MQTT | Press Start on the page: the car drives; press Stop: it stops. The Car card state goes IDLE, LINE_FOLLOW, IDLE | | | *link to be added* |
| V13 | No way round, then creep and rescan | Shows the obstacle fallback | An obstacle with no room at first (e.g. between two walls): the scan reports no way round, the car creeps 10 cm closer, sweeps again, and goes round on the side with more room | | | *link to be added* |

## Video index (other videos)



Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1 | S1-U03 | `abc1234` | | <https://youtu.be/...> | LED blinks at 2 Hz |
