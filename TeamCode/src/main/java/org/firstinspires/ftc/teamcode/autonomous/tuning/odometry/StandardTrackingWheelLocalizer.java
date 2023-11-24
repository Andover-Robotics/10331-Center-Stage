package org.firstinspires.ftc.teamcode.autonomous.tuning.odometry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.test.Encoder;

import java.util.Arrays;
import java.util.List;


@Config
//public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
public class StandardTrackingWheelLocalizer {

    /*
    TICKS_PER_REV is the number of "ticks" the encoders will count per revolution.
    You will find the specs of your encoders on your manufacturer's site.
    Be sure to find the Counts Per Revolution or CPR.

    WHEEL_RADIUS is the radius of the dead wheel

    GEAR_RATIO is the ratio of the output (wheel) speed to input (encoder) speed.
    If you are not gearing you encoders, leave this at 1

    LATERAL_DISTANCE is the distance from the left and right wheels.

    FORWARD_OFFSET is the distance from the center of rotation to the middle wheel.
    The FORWARD_OFFSET is positive when in front of the wheels and negative when behind the wheels (closer to the back).
*/


    //values determined via LocalizationTest

    /*
    Process for localization test
    You will begin the physical tuning process. Clear a straight line for your bot to travel in.
    Set your bot at the beginning of this stretch, facing forward.
    Run the LocalizationTest opmode. Do not touch the controller.
    Slowly drag your bot along this stretch. Keep the bot as straight as possible.
    Once you reach the end of your stretch, stop. Measure the distance traveled. Then look at the distance reported on the telemetry on the RC.
    Your multiplier will be the Measured Distance / Telemetry Distance Traveled .
    Repeat process 3 times for the forward direction to get the average multiplier.
    Then, set the X_MULTIPLIER to this value.
    Repeat the same process but in the strafing direction.
    Set Y_MULTIPLIER to the calculated strafe multiplier.
     */

  /*  public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction


    public static double TICKS_PER_REV = 537.7;
    public static double WHEEL_RADIUS = 0.75;
    public static double GEAR_RATIO = 1;


    //tbd
    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel
    //can measure it at workshop today

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }

   */
}