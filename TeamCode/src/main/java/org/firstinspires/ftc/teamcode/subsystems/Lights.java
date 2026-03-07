package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Lights — goBILDA RGB Indicator Light subsystem.
 *
 * goBILDA RGB indicator servo positions (500–2500µs, FTC 0.0–1.0):
 *
 *   FTC pos │ ~µs   │ Color
 *   ────────┼───────┼───────────────────────────────
 *   0.000   │  500  │ OFF (black)
 *   0.277   │ 1100  │ Red
 *   0.333   │ 1200  │ Orange
 *   0.444   │ 1400  │ Sage / Yellow-green
 *   0.500   │ 1500  │ Green
 *   0.611   │ 1700  │ Blue
 *   0.722   │ 1900  │ Violet / Pink
 *   1.000   │ 2500  │ White
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  COLOR SCHEME:
 *    CLOSE mode → BLUE         (solid when active, blink when aligned)
 *    FAR   mode → PINK         (solid when active, blink when aligned)
 *    AUTO  mode → BLUE→PINK    (gradient: close=blue, far=pink, blink when aligned)
 *    OFF         → light off
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  TUNING:
 *  The goBILDA indicator has hardware blink modes built in.
 *  Blink positions are typically offset from solid — adjust by ±0.05 at a time
 *  using a servo tester until you see a clean blink vs solid distinction.
 *  The constants below are starting points from the FTC position chart.
 * ─────────────────────────────────────────────────────────────────────────────
 */
public class Lights {

    // =========================================================================
    //  SERVO POSITION CONSTANTS — tune if colors are wrong
    // =========================================================================

    /** Off / shooter inactive. */
    public static final double POS_OFF = 0.00;

    // ── CLOSE mode → BLUE ─────────────────────────────────────────────────
    /** CLOSE mode solid — Blue (~1700µs). */
    public static final double POS_BLUE_SOLID = 0.611;
    /** CLOSE mode aligned — Blue blink. Tune ±0.05 from solid. */
    public static final double POS_BLUE_BLINK = 0.65;

    // ── FAR mode → PINK / VIOLET ──────────────────────────────────────────
    /** FAR mode solid — Violet/Pink (~1900µs). */
    public static final double POS_PINK_SOLID = 0.722;
    /** FAR mode aligned — Pink blink. Tune ±0.05 from solid. */
    public static final double POS_PINK_BLINK = 0.76;

    // ── AUTO mode → gradient (reuses BLUE and PINK constants above) ──────────
    // No separate constants needed — AUTO lerps between POS_BLUE_* and POS_PINK_*.

    // =========================================================================
    //  HARDWARE
    // =========================================================================
    private final Servo lightsServo;

    public Lights(HardwareMap hardwareMap) {
        lightsServo = hardwareMap.servo.get("Lights");
        lightsServo.setPosition(POS_OFF);
    }

    // =========================================================================
    //  PUBLIC API
    // =========================================================================

    // ── AUTO mode distance range (meters) — matches RPM_TABLE bounds ─────────
    private static final double AUTO_DIST_MIN = 1.59;  // closest shot → blue
    private static final double AUTO_DIST_MAX = 3.89;  // farthest shot → pink

    /**
     * Call every loop.
     *
     * @param shooterActive   true if flywheel is running
     * @param rpmMode         Shooter.RPMMode (CLOSE, FAR, AUTO, MANUAL)
     * @param isAligned       true if turret OR drivetrain is aligned to goal
     * @param distanceMeters  current shot distance in meters (used for AUTO gradient)
     */
    public void update(boolean shooterActive, Shooter.RPMMode rpmMode,
                       boolean isAligned, double distanceMeters) {
        if (!shooterActive) {
            set(POS_OFF);
            return;
        }
        switch (rpmMode) {
            case CLOSE:
                set(isAligned ? POS_BLUE_BLINK : POS_BLUE_SOLID);
                break;
            case FAR:
                set(isAligned ? POS_PINK_BLINK : POS_PINK_SOLID);
                break;
            case AUTO: {
                // Lerp servo position from blue (close) → pink (far) based on distance.
                // Clamp t to [0,1] so out-of-range distances stay at the endpoints.
                double t      = (distanceMeters - AUTO_DIST_MIN) / (AUTO_DIST_MAX - AUTO_DIST_MIN);
                t             = Math.max(0.0, Math.min(1.0, t));
                double solid  = POS_BLUE_SOLID + t * (POS_PINK_SOLID - POS_BLUE_SOLID);
                double blink  = POS_BLUE_BLINK + t * (POS_PINK_BLINK - POS_BLUE_BLINK);
                set(isAligned ? blink : solid);
                break;
            }
            default:
                set(POS_OFF);
                break;
        }
    }

    /**
     * Overload without distance — for CLOSE/FAR/MANUAL modes where distance is irrelevant.
     * AUTO mode will show blue solid as a fallback.
     */
    public void update(boolean shooterActive, Shooter.RPMMode rpmMode, boolean isAligned) {
        update(shooterActive, rpmMode, isAligned, AUTO_DIST_MIN);
    }

    /** Turn off immediately. Call in stop(). */
    public void off() {
        set(POS_OFF);
    }

    private void set(double position) {
        lightsServo.setPosition(position);
    }
}