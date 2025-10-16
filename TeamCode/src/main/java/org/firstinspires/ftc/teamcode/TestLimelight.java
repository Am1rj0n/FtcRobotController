package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import org.json.JSONObject;
import org.json.JSONArray;

@TeleOp(name="Limelight Test", group="Competition")
public class TestLimelight extends OpMode {

    // Hardware
    private DcMotorEx launcherMotor1;
    private DcMotorEx launcherMotor2;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private IMU imu;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();

    // State machine
    private enum LauncherState {
        IDLE,
        ALIGNING,
        ALIGNED_READY,
        LAUNCHING
    }
    private LauncherState state = LauncherState.IDLE;

    // Launcher parameters
    private double targetVelocity = 0;

    // ============================================================================
    // CALIBRATION SECTION - ADJUST THESE AFTER TESTING
    // ============================================================================

    // LIMELIGHT NETWORK CONFIGURATION
    private static final String LIMELIGHT_IP = "172.29.0.1";  // ADJUST: Try "limelight.local" or your actual IP
    private static final int LIMELIGHT_PORT = 5807;
    private static final int QUERY_INTERVAL_MS = 50;  // Query Limelight every 50ms (20Hz)

    // DISTANCE TO RPM FORMULA COEFFICIENTS
    private static final double BASE_RPM = 800.0;
    private static final double RPM_PER_INCH = 25.0;
    private static final double RPM_PER_INCH_SQUARED = 0.3;

    // MOTOR SPECIFICATIONS
    private static final double MOTOR_MAX_RPM = 4600.0;
    private static final double SAFE_MAX_RPM = 4400.0;
    private static final double MOTOR_WEAR_FACTOR = 1.0;

    // CAMERA MOUNTING (for distance calculation from ty angle)
    private static final double CAMERA_HEIGHT = 8.0;        // Height of Limelight lens from ground (inches)
    private static final double CAMERA_MOUNT_ANGLE = 25.0;  // Upward tilt angle of Limelight (degrees)
    private static final double TARGET_HEIGHT = 36.0;       // Height of AprilTag center from ground (inches)

    // APRILTAG IDS FOR GOALS
    private static final int RED_GOAL_TAG_ID = 24;
    private static final int BLUE_GOAL_TAG_ID = 20;

    // ALLIANCE SELECTION
    private static final int TARGET_TAG_ID = BLUE_GOAL_TAG_ID;  // ADJUST: Change to RED_GOAL_TAG_ID for red alliance
    private static final boolean ACCEPT_ANY_TAG = true;  // TEMPORARY: Set to false in competition

    // AUTO-ALIGNMENT PARAMETERS
    private static final double ALIGNMENT_KP = 0.035;
    private static final double ALIGNMENT_TOLERANCE = 1.5;
    private static final double MIN_TURN_POWER = 0.15;
    private static final double MAX_TURN_POWER = 0.6;

    // DISTANCE RANGE LIMITS
    private static final double MIN_DISTANCE = 10.0;
    private static final double MAX_DISTANCE = 120.0;

    // FILTERING AND STABILITY
    private static final double DISTANCE_FILTER = 0.25;
    private static final int STABLE_FRAMES_REQUIRED = 5;

    // MOTOR DRIFT CORRECTION
    private static final double FRONT_LEFT_MULTIPLIER = 1.0;
    private static final double FRONT_RIGHT_MULTIPLIER = 1.0;
    private static final double BACK_LEFT_MULTIPLIER = 1.0;
    private static final double BACK_RIGHT_MULTIPLIER = 1.0;

    // ============================================================================
    // END OF CALIBRATION SECTION
    // ============================================================================

    // Runtime variables
    private double lastValidDistance = 36.0;
    private double filteredDistance = 36.0;
    private int stableFrameCount = 0;
    private boolean lastAlignPressed = false;
    private boolean lastLaunchPressed = false;
    private boolean hasVibrated = false;

    // Limelight data
    private double limelightTx = 0;  // Horizontal offset in degrees
    private double limelightTy = 0;  // Vertical offset in degrees
    private boolean targetValid = false;
    private int detectedTagId = -1;
    private Thread limelightThread;
    private volatile boolean threadRunning = true;

    // Debug variables
    private String lastError = "No errors yet";
    private int successfulQueries = 0;
    private int failedQueries = 0;
    private long lastSuccessTime = 0;
    private String rawJsonSample = "No data yet";

    @Override
    public void init() {
        // Initialize launcher motors
        launcherMotor1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        // Configure launcher motors
        launcherMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        launcherMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure drive motors
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(imuParams);

        // Start Limelight query thread
        startLimelightThread();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Limelight IP", LIMELIGHT_IP + ":" + LIMELIGHT_PORT);
        telemetry.addData("Target Tag", TARGET_TAG_ID);
        telemetry.addData("Waiting for", "Limelight connection...");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Button edge detection
        boolean alignPressed = gamepad1.a && !lastAlignPressed;
        boolean launchPressed = gamepad1.b && !lastLaunchPressed;
        boolean cancelPressed = gamepad1.x;

        lastAlignPressed = gamepad1.a;
        lastLaunchPressed = gamepad1.b;

        // State machine logic
        switch (state) {
            case IDLE:
                if (alignPressed && targetValid && detectedTagId == TARGET_TAG_ID) {
                    state = LauncherState.ALIGNING;
                    stableFrameCount = 0;
                    hasVibrated = false;
                } else if (alignPressed) {
                    // Give feedback when trying to align without valid target
                    gamepad1.rumble(100);
                }
                manualDrive();
                break;

            case ALIGNING:
                if (cancelPressed) {
                    state = LauncherState.IDLE;
                    stopDrive();
                    stopLauncher();
                    break;
                }

                if (targetValid && detectedTagId == TARGET_TAG_ID) {
                    // Calculate distance from ty angle
                    double distance = calculateDistanceFromTy(limelightTy);

                    // Filter distance
                    filteredDistance = (DISTANCE_FILTER * distance) +
                            ((1 - DISTANCE_FILTER) * lastValidDistance);
                    lastValidDistance = filteredDistance;

                    // Auto-align rotation
                    if (Math.abs(limelightTx) > ALIGNMENT_TOLERANCE) {
                        autoAlign(limelightTx);
                        stableFrameCount = 0;
                    } else {
                        stopDrive();
                        stableFrameCount++;

                        if (stableFrameCount >= STABLE_FRAMES_REQUIRED) {
                            state = LauncherState.ALIGNED_READY;
                            targetVelocity = calculateLauncherSpeed(filteredDistance);
                            setLauncherSpeed(targetVelocity);
                            gamepad1.rumble(500);
                            hasVibrated = true;
                        }
                    }
                } else {
                    stopDrive();
                    stableFrameCount = 0;
                    // Lost target, return to idle
                    state = LauncherState.IDLE;
                    gamepad1.rumble(100);
                }
                break;

            case ALIGNED_READY:
                if (cancelPressed) {
                    state = LauncherState.IDLE;
                    stopDrive();
                    stopLauncher();
                    break;
                }

                if (targetValid && detectedTagId == TARGET_TAG_ID) {
                    double distance = calculateDistanceFromTy(limelightTy);
                    filteredDistance = (DISTANCE_FILTER * distance) +
                            ((1 - DISTANCE_FILTER) * lastValidDistance);
                    lastValidDistance = filteredDistance;

                    targetVelocity = calculateLauncherSpeed(filteredDistance);
                    setLauncherSpeed(targetVelocity);

                    if (Math.abs(limelightTx) > ALIGNMENT_TOLERANCE * 2) {
                        state = LauncherState.ALIGNING;
                        stableFrameCount = 0;
                        hasVibrated = false;
                    }
                } else {
                    // Lost target
                    state = LauncherState.IDLE;
                    stopLauncher();
                    gamepad1.rumble(100);
                }

                if (launchPressed) {
                    state = LauncherState.LAUNCHING;
                    runtime.reset();
                }
                break;

            case LAUNCHING:
                if (runtime.seconds() > 0.5) {
                    state = LauncherState.IDLE;
                    stopLauncher();
                }
                break;
        }

        // ENHANCED TELEMETRY
        telemetry.addLine("╔════ CONNECTION STATUS ════╗");
        telemetry.addData("│ Limelight IP", LIMELIGHT_IP + ":" + LIMELIGHT_PORT);
        telemetry.addData("│ Thread Running", threadRunning ? "✓ YES" : "✗ NO");
        telemetry.addData("│ Success / Fail", successfulQueries + " / " + failedQueries);

        if (successfulQueries > 0) {
            double secondsAgo = (System.currentTimeMillis() - lastSuccessTime) / 1000.0;
            telemetry.addData("│ Last Success", String.format("%.1fs ago", secondsAgo));
        } else {
            telemetry.addData("│ Last Success", "NEVER");
        }

        telemetry.addData("│ Last Error", lastError);
        telemetry.addLine("╚═══════════════════════════╝");

        telemetry.addLine("");
        telemetry.addLine("╔═══ LIMELIGHT DATA ════╗");
        telemetry.addData("│ Target Valid", targetValid ? "✓ YES" : "✗ NO");
        telemetry.addData("│ Detected Tag", detectedTagId == -1 ? "NONE" : String.valueOf(detectedTagId));
        telemetry.addData("│ Target Tag", TARGET_TAG_ID);
        telemetry.addData("│ Match", (targetValid && detectedTagId == TARGET_TAG_ID) ? "✓ MATCH" : "✗ NO MATCH");
        telemetry.addData("│ TX (horizontal)", String.format("%.2f°", limelightTx));
        telemetry.addData("│ TY (vertical)", String.format("%.2f°", limelightTy));
        telemetry.addLine("╚═══════════════════════════╝");

        telemetry.addLine("");
        telemetry.addLine("╔═══ ROBOT STATE ═══════╗");
        telemetry.addData("│ State", state.toString());
        telemetry.addData("│ Can Align", (targetValid && detectedTagId == TARGET_TAG_ID) ? "✓ YES" : "✗ NO");
        telemetry.addData("│ Distance", String.format("%.1f in", filteredDistance));
        telemetry.addData("│ Target RPM", String.format("%.0f", targetVelocity));
        telemetry.addData("│ Motor 1 RPM", String.format("%.0f", launcherMotor1.getVelocity()));
        telemetry.addData("│ Motor 2 RPM", String.format("%.0f", launcherMotor2.getVelocity()));
        telemetry.addLine("╚═══════════════════════════╝");

        telemetry.addLine("");
        telemetry.addData("🎮 Controls", "A: Align | B: Launch | X: Cancel");

        telemetry.update();
    }

    /**
     * Start background thread to query Limelight
     */
    private void startLimelightThread() {
        limelightThread = new Thread(() -> {
            while (threadRunning && !Thread.currentThread().isInterrupted()) {
                try {
                    queryLimelight();
                    Thread.sleep(QUERY_INTERVAL_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Continue on errors
                }
            }
        });
        limelightThread.setDaemon(true);
        limelightThread.start();
    }

    /**
     * Query Limelight for AprilTag data via REST API - DEBUG VERSION
     */
    private void queryLimelight() {
        try {
            URL url = new URL("http://" + LIMELIGHT_IP + ":" + LIMELIGHT_PORT + "/results");
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");
            conn.setConnectTimeout(200);
            conn.setReadTimeout(200);

            int responseCode = conn.getResponseCode();

            if (responseCode == 200) {
                BufferedReader reader = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                StringBuilder response = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    response.append(line);
                }
                reader.close();

                String jsonString = response.toString();
                parseLimelightData(jsonString);

                successfulQueries++;
                lastSuccessTime = System.currentTimeMillis();
                lastError = "✓ Connected";

                // Store sample for debugging
                if (rawJsonSample.equals("No data yet")) {
                    rawJsonSample = jsonString.length() > 100 ?
                            jsonString.substring(0, 100) + "..." : jsonString;
                }
            } else {
                failedQueries++;
                lastError = "HTTP Error: " + responseCode;
                targetValid = false;
                detectedTagId = -1;
            }

        } catch (java.net.UnknownHostException e) {
            failedQueries++;
            lastError = "✗ Cannot find Limelight - Check IP";
            targetValid = false;
            detectedTagId = -1;
        } catch (java.net.ConnectException e) {
            failedQueries++;
            lastError = "✗ Connection refused - Is Limelight on?";
            targetValid = false;
            detectedTagId = -1;
        } catch (java.net.SocketTimeoutException e) {
            failedQueries++;
            lastError = "✗ Timeout - Limelight not responding";
            targetValid = false;
            detectedTagId = -1;
        } catch (Exception e) {
            failedQueries++;
            lastError = "✗ Error: " + e.getClass().getSimpleName();
            targetValid = false;
            detectedTagId = -1;
        }
    }

    /**
     * Parse JSON response from Limelight
     */
    private void parseLimelightData(String jsonString) {
        try {
            JSONObject json = new JSONObject(jsonString);

            // Check if target is valid
            targetValid = json.optBoolean("v", false);

            if (!targetValid) {
                detectedTagId = -1;
                return;
            }

            // Get targeting data
            limelightTx = json.optDouble("tx", 0);
            limelightTy = json.optDouble("ty", 0);

            // Try to get AprilTag ID from Fiducial results
            if (json.has("Fiducial")) {
                JSONArray fiducials = json.getJSONArray("Fiducial");
                if (fiducials.length() > 0) {
                    JSONObject firstTag = fiducials.getJSONObject(0);
                    detectedTagId = firstTag.optInt("fID", -1);
                } else {
                    detectedTagId = -1;
                }
            } else {
                detectedTagId = -1;
            }

        } catch (Exception e) {
            targetValid = false;
            detectedTagId = -1;
            lastError = "JSON Parse Error: " + e.getMessage();
        }
    }

    /**
     * Calculate distance from Limelight ty angle using trigonometry
     */
    private double calculateDistanceFromTy(double ty) {
        double angleToTarget = CAMERA_MOUNT_ANGLE + ty;
        double angleRadians = Math.toRadians(angleToTarget);

        if (angleRadians <= 0.01) {
            angleRadians = 0.01;
        }

        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRadians);
        return Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
    }

    /**
     * Calculate launcher speed based on distance
     */
    private double calculateLauncherSpeed(double distance) {
        double rpm = BASE_RPM +
                (RPM_PER_INCH * distance) +
                (RPM_PER_INCH_SQUARED * distance * distance);

        rpm *= MOTOR_WEAR_FACTOR;
        rpm = Math.max(0, Math.min(SAFE_MAX_RPM, rpm));

        return rpm;
    }

    /**
     * Auto-align the robot to face the target
     */
    private void autoAlign(double tx) {
        double turnPower = tx * ALIGNMENT_KP;

        if (Math.abs(turnPower) < MIN_TURN_POWER) {
            turnPower = Math.signum(turnPower) * MIN_TURN_POWER;
        }

        turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));

        frontLeft.setPower(-turnPower);
        backLeft.setPower(-turnPower);
        frontRight.setPower(turnPower);
        backRight.setPower(turnPower);
    }

    /**
     * Apply deadzone to joystick input
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        return value;
    }

    /**
     * Manual field-centric driving
     */
    private void manualDrive() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double robotAngle = Math.toRadians(yaw);

        double axial = applyDeadzone(-gamepad1.left_stick_y, 0.05);
        double lateral = applyDeadzone(gamepad1.left_stick_x, 0.05);
        double yawInput = applyDeadzone(gamepad1.right_stick_x, 0.05);

        double temp = axial * Math.cos(robotAngle) - lateral * Math.sin(robotAngle);
        lateral = axial * Math.sin(robotAngle) + lateral * Math.cos(robotAngle);
        axial = temp;

        double leftFrontPower = axial + lateral + yawInput;
        double rightFrontPower = axial - lateral - yawInput;
        double leftBackPower = axial - lateral + yawInput;
        double rightBackPower = axial + lateral - yawInput;

        double max = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        );
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        frontLeft.setPower(leftFrontPower * FRONT_LEFT_MULTIPLIER);
        frontRight.setPower(rightFrontPower * FRONT_RIGHT_MULTIPLIER);
        backLeft.setPower(leftBackPower * BACK_LEFT_MULTIPLIER);
        backRight.setPower(rightBackPower * BACK_RIGHT_MULTIPLIER);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void setLauncherSpeed(double rpm) {
        launcherMotor1.setVelocity(rpm);
        launcherMotor2.setVelocity(rpm);
    }

    private void stopLauncher() {
        launcherMotor1.setVelocity(0);
        launcherMotor2.setVelocity(0);
    }

    @Override
    public void stop() {
        threadRunning = false;
        if (limelightThread != null) {
            limelightThread.interrupt();
        }
        stopDrive();
        stopLauncher();
    }
}