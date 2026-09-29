package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Collections;
import java.util.List;

/**
 * Pollen Chase: drives a mecanum robot toward yellow BIOBUZZ pollen, using a
 * Limelight 3A running the pollen neural detector (see README).
 *
 * Controls (gamepad 1):
 *   Sticks            normal robot-centric mecanum driving, capped at MANUAL_SCALE.
 *   HOLD right bumper chase: turn toward the best group of pollen, drive forward,
 *                     slow down while closing in, stop just short of the pile.
 * Releasing the bumper or touching a stick takes over instantly. The robot also
 * stops if no pollen has been seen for LOST_TIMEOUT_S seconds.
 *
 * Before the first run, set the "Robot configuration" block below to match your
 * robot's configuration names and motor directions.
 */
@TeleOp(name = "Pollen Chase", group = "Vision")
public class PollenChase extends LinearOpMode {

    private static final String TAG = "PollenChase";

    // ===== Robot configuration: change these to match your robot =====
    private static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
    private static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
    private static final String BACK_LEFT_MOTOR = "backLeftMotor";
    private static final String BACK_RIGHT_MOTOR = "backRightMotor";
    // Most mecanum drivetrains need one side reversed. If the robot drives backward
    // or spins when you push the left stick forward, swap these two.
    private static final DcMotor.Direction LEFT_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int DETECTOR_PIPELINE = 2; // Limelight pipeline slot holding the pollen model

    private static final String POLLEN_CLASS = "pollen";
    private static final double MIN_CONFIDENCE = 0.5;

    // Chase tuning
    private static final double MAX_DRIVE = 0.35;   // forward power cap while chasing
    private static final double MAX_TURN = 0.30;    // turn power cap while chasing
    private static final double TURN_KP = 0.02;     // turn power per degree of tx
    private static final double TX_DEADBAND_DEG = 2.0;
    private static final double FAR_AREA = 0.02;    // at or below this box area: full MAX_DRIVE
    private static final double STOP_AREA = 0.14;   // box area when the ball is at the bumper: stop
    private static final double LOST_TIMEOUT_S = 0.5;

    private static final double MANUAL_SCALE = 0.5; // stick driving power cap in this test OpMode
    private static final long LOG_INTERVAL_MS = 200;

    // Group preference. Two balls belong to the same group when their centers are closer
    // than GROUP_GAP ball-widths apart. A group scores SCORE_PER_BALL per ball plus
    // SCORE_PER_AREA * (largest box area) so that, all else equal, closer groups win.
    // Example: a single ball at the bumper (area 0.14) scores 1 + 1.1 = 2.1; a pair at
    // mid range (area 0.03 each) scores 2 + 0.2 = 2.2 and wins.
    private static final double GROUP_GAP = 3.3;
    private static final double SCORE_PER_BALL = 1.0;
    private static final double SCORE_PER_AREA = 8.0;
    private static final double SWITCH_HYSTERESIS = 1.25; // new group must beat the current one by this factor
    // Angular width of a detection box in degrees = DEG_PER_SQRT_AREA * sqrt(area).
    // 48 = sqrt(54.5 * 42.2), the Limelight 3A field of view in degrees.
    private static final double DEG_PER_SQRT_AREA = 48.0;

    /** A cluster of nearby pollen detections. */
    private static class Group {
        int count;
        double sumWeightedTx, sumWeight; // area-weighted center
        double maxArea, maxConf;
        double centerTx() { return sumWeight > 0 ? sumWeightedTx / sumWeight : 0; }
        double score() { return SCORE_PER_BALL * count + SCORE_PER_AREA * maxArea; }
    }

    /** Single-linkage clustering of detections in image space. */
    private static List<Group> groupDetections(List<LLResultTypes.DetectorResult> dets) {
        int n = dets.size();
        int[] parent = new int[n];
        for (int i = 0; i < n; i++) parent[i] = i;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                LLResultTypes.DetectorResult a = dets.get(i), b = dets.get(j);
                double widthA = DEG_PER_SQRT_AREA * Math.sqrt(Math.max(a.getTargetArea(), 1e-6));
                double widthB = DEG_PER_SQRT_AREA * Math.sqrt(Math.max(b.getTargetArea(), 1e-6));
                double dx = a.getTargetXDegrees() - b.getTargetXDegrees();
                double dy = a.getTargetYDegrees() - b.getTargetYDegrees();
                double dist = Math.sqrt(dx * dx + dy * dy);
                if (dist < GROUP_GAP * 0.5 * (widthA + widthB)) {
                    int ra = find(parent, i), rb = find(parent, j);
                    if (ra != rb) parent[ra] = rb;
                }
            }
        }
        java.util.Map<Integer, Group> groups = new java.util.HashMap<>();
        for (int i = 0; i < n; i++) {
            int root = find(parent, i);
            Group g = groups.get(root);
            if (g == null) { g = new Group(); groups.put(root, g); }
            LLResultTypes.DetectorResult d = dets.get(i);
            g.count++;
            g.sumWeightedTx += d.getTargetXDegrees() * d.getTargetArea();
            g.sumWeight += d.getTargetArea();
            g.maxArea = Math.max(g.maxArea, d.getTargetArea());
            g.maxConf = Math.max(g.maxConf, d.getConfidence());
        }
        return new java.util.ArrayList<>(groups.values());
    }

    private static int find(int[] parent, int i) {
        while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    }

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR);
        backLeftMotor = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR);
        frontRightMotor = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR);
        backRightMotor = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR);

        frontLeftMotor.setDirection(LEFT_DIRECTION);
        backLeftMotor.setDirection(LEFT_DIRECTION);
        frontRightMotor.setDirection(RIGHT_DIRECTION);
        backRightMotor.setDirection(RIGHT_DIRECTION);
        // Brake so the robot stops crisply when the chase ends
        for (DcMotor m : new DcMotor[] {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(DETECTOR_PIPELINE);
        limelight.start();

        telemetry.addLine("Hold RIGHT BUMPER to chase pollen. Sticks drive otherwise.");
        telemetry.update();
        waitForStart();
        RobotLog.ii(TAG, "started");

        ElapsedTime sinceSeen = new ElapsedTime();
        long lastLogMs = 0;
        double lastTx = 0, lastArea = 0, lastScore = 0, lastConf = 0;
        int lastGroupSize = 0, groupCount = 0;
        boolean everSeen = false;

        while (opModeIsActive()) {
            // ---- Vision: confident pollen detections ----
            LLResult result = limelight.getLatestResult();
            boolean valid = result != null && result.isValid() && result.getStaleness() < 200;
            List<LLResultTypes.DetectorResult> detections =
                    valid ? result.getDetectorResults() : Collections.<LLResultTypes.DetectorResult>emptyList();
            List<LLResultTypes.DetectorResult> pollen = new java.util.ArrayList<>();
            for (LLResultTypes.DetectorResult d : detections) {
                if (!POLLEN_CLASS.equalsIgnoreCase(d.getClassName())) continue;
                if (d.getConfidence() < MIN_CONFIDENCE) continue;
                pollen.add(d);
            }

            // Prefer groups: cluster the balls and pick the best-scoring cluster
            List<Group> groups = groupDetections(pollen);
            groupCount = groups.size();
            Group best = null;
            for (Group g : groups) {
                if (best == null || g.score() > best.score()) best = g;
            }
            if (best != null) {
                // Hysteresis: keep chasing the group we were already heading for unless
                // the new best is clearly better (avoids flip-flopping between equals)
                if (everSeen && sinceSeen.seconds() < LOST_TIMEOUT_S) {
                    Group current = null;
                    for (Group g : groups) {
                        double off = Math.abs(g.centerTx() - lastTx);
                        if (off < 10 && (current == null || off < Math.abs(current.centerTx() - lastTx))) current = g;
                    }
                    if (current != null && current != best && best.score() < SWITCH_HYSTERESIS * current.score()) {
                        best = current;
                    }
                }
                sinceSeen.reset();
                everSeen = true;
                lastTx = best.centerTx();
                lastArea = best.maxArea;     // nearest ball of the group drives the stop distance
                lastScore = best.score();
                lastGroupSize = best.count;
                lastConf = best.maxConf;
            }
            // Short grace period so a missed detector frame does not stop the robot
            boolean haveTarget = everSeen && sinceSeen.seconds() < LOST_TIMEOUT_S;

            // ---- Decide drive command (robot-centric: y forward, x strafe, rx turn) ----
            double y, x, rx;
            String mode;
            boolean chase = gamepad1.right_bumper;
            boolean stickActive = Math.abs(gamepad1.left_stick_y) > 0.05
                    || Math.abs(gamepad1.left_stick_x) > 0.05
                    || Math.abs(gamepad1.right_stick_x) > 0.05;

            if (chase && !stickActive) {
                if (haveTarget && lastArea < STOP_AREA) {
                    // Turn toward the ball
                    double turn = Math.abs(lastTx) < TX_DEADBAND_DEG ? 0 : TURN_KP * lastTx;
                    rx = Math.max(-MAX_TURN, Math.min(MAX_TURN, turn));
                    // Drive forward, slowing as the ball fills more of the image
                    double t = (lastArea - FAR_AREA) / (STOP_AREA - FAR_AREA); // 0 far .. 1 at stop
                    t = Math.max(0, Math.min(1, t));
                    y = MAX_DRIVE * (1.0 - t);
                    // Do not drive forward while badly misaligned
                    if (Math.abs(lastTx) > 15) y *= 0.3;
                    x = 0;
                    mode = "CHASE";
                } else if (haveTarget) {
                    y = 0; x = 0; rx = 0;
                    mode = "ARRIVED";
                } else {
                    y = 0; x = 0; rx = 0;
                    mode = "NO TARGET";
                }
            } else {
                y = -gamepad1.left_stick_y * MANUAL_SCALE;
                x = gamepad1.left_stick_x * MANUAL_SCALE;
                rx = gamepad1.right_stick_x * MANUAL_SCALE;
                mode = "MANUAL";
            }

            // ---- Robot-centric mecanum mixing ----
            double leftFront = y + x + rx;
            double rightFront = y - x - rx;
            double leftBack = y - x + rx;
            double rightBack = y + x - rx;
            double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                    Math.max(Math.abs(leftBack), Math.abs(rightBack)));
            if (max > 1.0) {
                leftFront /= max; rightFront /= max; leftBack /= max; rightBack /= max;
            }
            frontLeftMotor.setPower(leftFront);
            frontRightMotor.setPower(rightFront);
            backLeftMotor.setPower(leftBack);
            backRightMotor.setPower(rightBack);

            // ---- Telemetry and log ----
            telemetry.addData("Mode", mode);
            telemetry.addData("Pollen seen", "%d in %d group(s)", pollen.size(), groupCount);
            if (haveTarget) {
                telemetry.addData("Target group", "%d ball(s), score %.1f", lastGroupSize, lastScore);
                telemetry.addData("tx (deg)", "%.1f", lastTx);
                telemetry.addData("nearest area (%)", "%.1f", lastArea * 100);
                telemetry.addData("conf", "%.2f", lastConf);
            }
            telemetry.addData("drive y / turn rx", "%.2f / %.2f", y, rx);
            telemetry.update();

            long now = System.currentTimeMillis();
            if (now - lastLogMs >= LOG_INTERVAL_MS) {
                lastLogMs = now;
                RobotLog.ii(TAG, "%s seen=%d groups=%d target=%d balls score=%.1f tx=%.1f area=%.3f conf=%.2f y=%.2f rx=%.2f",
                        mode, pollen.size(), groupCount, lastGroupSize, lastScore, lastTx, lastArea,
                        lastConf, y, rx);
            }
        }

        frontLeftMotor.setPower(0); frontRightMotor.setPower(0);
        backLeftMotor.setPower(0); backRightMotor.setPower(0);
        RobotLog.ii(TAG, "stopped");
        limelight.stop();
    }
}
