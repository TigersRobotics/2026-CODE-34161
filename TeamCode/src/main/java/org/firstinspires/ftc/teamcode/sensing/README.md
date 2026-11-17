# Robot sensing system

Tells the rest of the code two things:

- **Where the robot is**, by blending the goBILDA Pinpoint (odometry pods + gyro) with a webcam.
- **Which way each hive is tipped**, by looking at the AprilTags under the hive's cells.

Nothing here is wired into `TeleOpMain` yet. Use the two OpModes in the "Sensing" group to set it up and test it first.

## How it works

| Part | File | What it does |
|---|---|---|
| Front door | `RobotSensing.java` | The only class other code needs. `init`, `update`, `stop`, and getters. |
| Camera | `HiveCamera.java` | Runs the webcam and the SDK's AprilTag detector on the camera thread. |
| Maths | `HiveSensingCore.java` | Turns each tag sighting into a hive vote and a position fix. |
| Hive memory | `HiveTracker.java` | Smooths the votes and remembers each hive's state. |
| Blending | `PoseFusion.java` | Adds camera corrections on top of the odometry. |
| Numbers | `constants/SensingConstants.java` | Every value to measure or tune. |

Things worth knowing about BIOBUZZ:

- The only AprilTags on the field are on the **underside of the hive cells**, four tags per cell (IDs 30-45). The camera has to look **upward**.
- The tags **move when a hive tips**. So each cluster has two known places, cell up and cell down.
- The system works out up or down from the **height** of the tags, which does not depend on knowing where the robot is. Only then does it use that cluster's known place to correct the position.
- Heading always comes from the Pinpoint gyro. A webcam cannot beat it.

## Setting it up, in order

1. **Mount the camera** on the turret, tilted up so it sees the underside of the cells from where the robot shoots.
2. **Fill in sections 1 and 2 of `SensingConstants`**: camera name, resolution, lens calibration, and where the camera sits. The pitch angle and lens calibration matter most.
3. **Run "Hive Tag Calibration"** and follow the steps at the top of that file. It prints the 8 rows for section 3.
4. **Run "Sensing Test"**. Tip a hive by hand and check the state flips and the tip count goes up by one. Park on a known spot and check the pose with a tape measure.
5. **Tune section 5** only if needed. The comments say what each number does.

Until step 3 is done the system still runs, on odometry alone, and reports every hive as `UNKNOWN`.

## Choosing a camera

Any USB (UVC) webcam works. Check the current competition manual for the vision-camera rule before buying.

- **Recommended: Arducam OV9281 global-shutter USB camera** (1280x800, monochrome, about 100 fps). Global shutter takes the whole picture at one instant, so tags stay sharp while the robot or turret is moving. It needs its lens calibrated once.
- **Easiest: Logitech C920.** The SDK has its calibration built in at 640x480. Rolling shutter, so it blurs more on the move. Turn autofocus off.
- **Cheapest: Logitech C270.** Fixed focus and built-in calibration, but the weakest picture.

## Using it from an OpMode

```java
// init()
RobotSensing.init(hardwareMap);
RobotSensing.setAlliance(RobotSensing.Alliance.RED);

// loop(), straight after odometry.update()
Pose2D p = odometry.getPose();
RobotSensing.update(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH),
        p.getHeading(AngleUnit.RADIANS), PollenTurretController.getCurrentYaw());

double x = RobotSensing.getX();                    // inches
double y = RobotSensing.getY();
HiveTracker.State red = RobotSensing.getHiveState(RobotSensing.Alliance.RED);
if (RobotSensing.hasTarget()) {
    double distance = RobotSensing.getTargetDistanceIn();
    double yaw = RobotSensing.getTargetYawDeg();   // counter-clockwise positive: negate for PollenTurretController
}

// stop()
RobotSensing.stop();
```

## What has and has not been tested

The maths was run against a simulated robot on a laptop: a camera with 1-2 inches of noise, 70 ms of delay, bad frames, a drifting odometry, a hive tipping, and the robot being shoved. It has **not** been run on the real robot, and the real accuracy depends on the measurements in `SensingConstants`.
