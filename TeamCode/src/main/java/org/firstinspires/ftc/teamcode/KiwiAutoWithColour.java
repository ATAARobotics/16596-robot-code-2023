package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Kiwi_auto_colour1  ", group="autonomous")
public class KiwiAutoWithColour extends LinearOpMode {
    // Declare OpMode motors objects.
    private Motor motor_left = null;
    private Motor motor_right = null;
    private Motor motor_slide = null;

    private Motor elevator_motor= null;
    private ColorSensor color = null;
    private DistanceSensor distance = null;



    // Inertial Measurement Unit
    private IMU imu = null;

    // Holonomic Drive
    private HDrive drive = null;

    // servos (for the claw)
    private Servo servo_claw_left = null;
    private Servo servo_claw_right = null;

    // setup timer for auto steps
    private ElapsedTime runtime = new ElapsedTime();
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // initialize IMU
        IMU.Parameters imu_params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imu_params);

        // initialize motors - connect to hardware
        motor_left = new Motor(hardwareMap, "left");
        motor_right = new Motor(hardwareMap, "right");
        motor_slide = new Motor(hardwareMap, "slide");
        elevator_motor = new Motor(hardwareMap, "elevator");

        motor_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elevator_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        motor_left.setRunMode(Motor.RunMode.RawPower);
        motor_right.setRunMode(Motor.RunMode.RawPower);
        motor_slide.setRunMode(Motor.RunMode.RawPower);
        elevator_motor.setRunMode(Motor.RunMode.RawPower);

        motor_left.setInverted(false);
        motor_right.setInverted(false);
        motor_slide.setInverted(false);
        elevator_motor.setInverted(false);

        servo_claw_left = hardwareMap.get(Servo.class, "servo_left");
        servo_claw_right = hardwareMap.get(Servo.class, "servo_right");

        elevator_motor.resetEncoder();

        // initialize holonomic drive

        // first three arguments are the motors themselves, the next
        // three numbers are 'angles' for the motors -- this is their
        // mounting angle, relative to "0" being "forward"

        // NOTE NOTE!
        //    0. The angle "0" is straight ahead
        //    1. Angles are "right-hand coordinate" so "20" means "20 degress counter-clockwise"
        //    2. The motors ARE NOT IN counter-clockwise order! (you specify left, then right)
        //    3. Most angles are in RADIANS internally in ftclib (including these)

        drive = new HDrive(
                motor_left, motor_right, motor_slide,
                Math.toRadians(60), Math.toRadians(300), Math.toRadians(180)
        );
        drive.setMaxSpeed(0.75); // 0.0 to 1.0, percentage of "max"
        imu.resetYaw();

        waitForStart();
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "color");

        distance = hardwareMap.get(DistanceSensor.class, "color");
        // turn off LED
        color.enableLed(false);

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for X seconds
        // drive values are: strafeSpeed, forward speed, turn, heading
       double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        heading = 0;
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            drive.driveFieldCentric(0, 0.25, 0, heading);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            double distance_mm = distance.getDistance(DistanceUnit.MM);
            update_colour();
            // get encoder colour values:
            telemetry.addData("detected:  ", update_colour());
            telemetry.addData("heading :  ",heading);
            telemetry.addData("distance:  ", distance_mm);

            telemetry.update();
        }


    }
    private String update_colour() {
        String detected_colour = "unknown";
        int r = color.red();
        int g = color.green();
        int b = color.blue();
        int max = Math.max(color.alpha(), Math.max(b, Math.max(r, g)));
        if (r == max) {
            detected_colour = "red";
        }
        if (g == max) {
            detected_colour = "green";
        }
        if (b == max) {
            detected_colour = "blue";
        }
        telemetry.addData(
                "colour",
                String.format("colour: red=%d green=%d blue=%d max=%d detected=%s", r, g, b, max, detected_colour)
        );
        return detected_colour;
    }
}
