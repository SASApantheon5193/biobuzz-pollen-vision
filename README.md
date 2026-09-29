# BIOBUZZ Pollen Vision for Limelight 3A

Find yellow pollen with a Limelight 3A and drive to it. Built for the FTC 2026-27 **BIOBUZZ** season and shared by FTC Teams 5193 and 10653.

- **Pollen Chase**: hold the right bumper and the robot turns toward the best *group* of pollen, drives up, and stops just short of the pile.
- **A trained pollen detector** for the Limelight 3A (neural network, about 11 FPS).
- **Two test OpModes** to check your camera before anything moves.

## What's in this repo

| Path | What it is |
|---|---|
| `TeamCode/.../PollenChase.java` | Drives a mecanum robot to pollen while you hold the right bumper |
| `TeamCode/.../PollenDetectorTest.java` | Shows neural detector results on telemetry. Motors don't move |
| `TeamCode/.../YellowPollenTest.java` | Same idea using a plain color pipeline, no neural network needed |
| `limelight/model/` | The pollen detector: model files, labels, and [MODEL.md](limelight/model/MODEL.md) with training details and test results |

## What you need

- FTC SDK 12.0 (tested; the BIOBUZZ season SDK)
- A Limelight 3A plugged into the Control Hub's USB port, named `limelight` in your robot configuration
- For Pollen Chase: a mecanum drivetrain
- A computer and USB cable to set up the Limelight once

## Step 1: Load the pollen detector onto your Limelight

1. Plug the Limelight into your computer with USB and open `http://limelight.local:5801` or `http://172.29.0.1:5801` in a browser.
   Ours showed up at `http://172.28.0.1:5801` instead. If none of these work, check which IP address your computer got on the Limelight's USB network adapter; the camera is the `.1` address on that subnet.
2. Pick a pipeline slot. The code uses **pipeline 2**. If you use a different slot, change `DETECTOR_PIPELINE` in the code.
3. Set **Pipeline Type** to **Neural Detector**.
4. Upload the files from `limelight/model/`:
   - **TFlite/HEF Model File**: `limelight_neural_detector_8bit.tflite`
   - **Labels File**: `limelight_neural_detector_labels.txt`
5. Set these on the same pipeline:

   | Setting | Value | Why |
   |---|---|---|
   | Detector Runtime | CPU | The Limelight 3A runs the network on its CPU |
   | Confidence Threshold | 0.3 | The code filters again at 0.5 |
   | Exposure (.01 ms) | 2000 (20 ms) | Brighter images score much higher, see below |
   | Sensor Gain | 15 | |

6. Save the pipeline. Put a ball in front of the camera: it should get a box labeled **pollen**.

> **Exposure matters more than anything else.** At 6 ms our detector scored balls around 0.45. At 20 ms the same balls scored 0.80-0.85. Under very bright venue lights, lower the exposure until the balls look bright but not washed out.

Using a different Limelight? With a Google Coral, upload `limelight_neural_detector_coral.tflite` instead and set Detector Runtime to Coral (we haven't been able to test this file). A Limelight 4 needs a Hailo `.hef` model, see [Train your own model](#train-your-own-model).

## Step 2: Add the code to your project

1. Copy the three `.java` files from `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` into the same folder in your project. They use the standard `org.firstinspires.ftc.teamcode` package, so nothing else needs to change.
2. Open `PollenChase.java` and set the **Robot configuration** block at the top to match your robot:

   ```java
   private static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
   private static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
   private static final String BACK_LEFT_MOTOR = "backLeftMotor";
   private static final String BACK_RIGHT_MOTOR = "backRightMotor";
   private static final DcMotor.Direction LEFT_DIRECTION = DcMotor.Direction.REVERSE;
   private static final DcMotor.Direction RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
   private static final String LIMELIGHT_NAME = "limelight";
   private static final int DETECTOR_PIPELINE = 2;
   ```

3. Make sure the Limelight is in your active robot configuration with the name `limelight` (or change `LIMELIGHT_NAME`).
4. Build and install to the robot as usual.

## Step 3: Check the camera (motors don't move)

Run **Pollen Detector Test** (TeleOp, group Vision) and put a ball in front of the robot. Telemetry shows the number of detections, and for the closest pollen: `tx` (degrees left or right of center), `ty`, the box area, and the confidence.

A ball in clear view should score above 0.5. If it doesn't, raise the exposure (Step 1).

## Step 4: Chase pollen

Run **Pollen Chase**.

| Gamepad 1 | What happens |
|---|---|
| Sticks | Normal robot-centric mecanum driving at half power |
| **Hold right bumper** | Chase the best group of pollen |
| Release the bumper or touch a stick | Stops the chase immediately |

The robot also stops if it hasn't seen pollen for half a second.

**First run:** put one ball about 4 ft in front of the robot and a little to one side, with nothing else in the way. Hold the right bumper and be ready to let go. The robot should turn toward the ball, drive forward slowly, and stop just short of it.

- If the sticks don't drive correctly, fix `LEFT_DIRECTION` / `RIGHT_DIRECTION` first.
- If the sticks work but the chase turns *away* from the ball, your Limelight is probably mounted upside down. Change the sign of `TURN_KP`.

Telemetry shows the mode (`MANUAL`, `CHASE`, `ARRIVED` or `NO TARGET`), how many balls and groups it sees, the target group, and the drive and turn power.

## How it picks where to go

Every detection above 0.5 confidence is sorted into groups: two balls are in the same group when their centers are less than about 3 ball-widths apart in the image. Each group scores **1 point per ball plus 8 × the image area of its nearest ball**. The robot aims at the center of the highest-scoring group and stops when that group's nearest ball reaches `STOP_AREA`.

- When it picks a new target, a pair beats a single ball unless the single is almost at the stopping point.
- Between groups of the same size, the closer one wins.
- Once it's chasing a group, a different group has to score at least 25% more to pull it away. That stops it from flipping back and forth between similar piles, but it also means it can keep chasing a close single ball after a pair comes into view.

## Tuning

All of these are constants at the top of `PollenChase.java`.

| Constant | Default | What it does |
|---|---|---|
| `STOP_AREA` | 0.14 | Stop when the nearest ball fills this fraction of the image. Bigger stops closer: 0.18 is about 12% closer, 0.20 about 16% closer. Depends on where your camera is mounted |
| `MAX_DRIVE` | 0.35 | Top forward power while chasing |
| `MAX_TURN` | 0.30 | Top turning power while chasing |
| `TURN_KP` | 0.02 | Turning power per degree the target is off center |
| `FAR_AREA` | 0.02 | Drives at full `MAX_DRIVE` while the ball is smaller than this, then slows down until `STOP_AREA` |
| `MIN_CONFIDENCE` | 0.5 | Ignore detections below this |
| `LOST_TIMEOUT_S` | 0.5 | How long to keep going after losing sight of the target |
| `SCORE_PER_BALL` | 1.0 | Raise it to prefer bigger groups more strongly |
| `SCORE_PER_AREA` | 8.0 | Raise it to prefer closer targets |
| `GROUP_GAP` | 3.3 | How far apart (in ball-widths) balls can be and still count as one group |
| `MANUAL_SCALE` | 0.5 | Power cap for stick driving |

## No neural network? Use the color pipeline

**Yellow Pollen Test** reads a plain color pipeline in slot 1. It needs no model and runs at about 90 FPS, but yellow and orange things (and some wood floors) can fool it, and touching balls merge into one blob.

| Setting | Value |
|---|---|
| Pipeline Type | Color/Retroreflective |
| Hue | 21-33 |
| Saturation | 170-255 |
| Value | 165-255 |
| Exposure (.01 ms) | 600 |
| Sensor Gain | 15 |
| Erosion Steps / Dilation Steps | 1 / 1 |
| Area (% of image) | minimum 0.1 |
| Fullness (% of blue rect) | minimum 60 |
| W/H Ratio (yellow rect) | leave wide open |

It also estimates distance from the ball's size. That estimate was calibrated on our camera, so check it against a tape measure on yours.

## Things we learned the hard way

- **Don't filter balls by W/H ratio on the color pipeline.** When we set it to 0.5-2.0 (through the Limelight's API), it rejected our balls every time. Leave it wide open and use Fullness to reject odd shapes.
- **Target area units differ.** `ColorResult.getTargetArea()` and `DetectorResult.getTargetArea()` return a fraction of the image (0-1). `LLResult.getTa()` returns a percent (0-100).
- **If the neural pipeline runs at full frame rate with low CPU, the model isn't running.** With the model loaded, a Limelight 3A drops to about 11 FPS.

## Train your own model

We trained with Limelight's free [Neural Network Trainer](https://tools.limelightvision.io/neural-network-trainer):

1. Build or fork a dataset on [Roboflow](https://roboflow.com). [MODEL.md](limelight/model/MODEL.md) lists the datasets we combined.
2. Export a dataset version as **TensorFlow TFRecord** and copy the download link.
3. Paste the link into the trainer, choose the platform (**Coral / CPU** for a Limelight 3A, **Hailo-8L** or **Hailo-8** for a Limelight 4), and train for 20,000 steps. A 4,000-step run is a quick test.
4. Upload the resulting model and labels to your Limelight as in Step 1.

Adding photos taken by your own camera on a real field is the best way to improve it.

## Credits

Shared by FTC Teams 5193 and 10653.

The model was trained on these Roboflow Universe datasets, both licensed CC BY 4.0:

- [Biobuzz Pollen](https://universe.roboflow.com/junipers-workspace/biobuzz-pollen) by Junipers Workspace
- [FTC BIOBUZZ Grounded Game Pieces](https://universe.roboflow.com/calin-cspro/ftc-biobuzz-grounded-game-pieces) by Calins Workspace

If you share the model, or a model you trained from it, please credit those datasets too.

## License

The code is under the [MIT License](LICENSE): use it, change it and share it, and keep the copyright notice.
