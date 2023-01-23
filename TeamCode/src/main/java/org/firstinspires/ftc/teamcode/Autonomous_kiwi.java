package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Autonomous_kiwi extends LinearOpMode {
    // Declare OpMode motors objects.
    private Motor motor_left = null;
    private Motor motor_right = null;
    private Motor motor_slide = null;

    private Motor elevator_motor= null;
    //private NormalizedColorSensor colorSensor = null;

    // Inertial Measurement Unit
    private IMU imu = null;

    // Holonomic Drive
    private HDrive drive = null;

    // servos (for the claw)
    private Servo servo_claw_left = null;
    private Servo servo_claw_right = null;


    @Override
    public void runOpMode() throws InterruptedException {
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
        elevator_motor= new Motor(hardwareMap,"elevator");

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

        waitForStart();
        while (opModeIsActive())
        {
            // put auto drive commands here
            // drive ahead to parking spot

            double heading = - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // drive values are: strafeSpeed, forward speed, turn, heading
            drive.driveFieldCentric(0, .5,0,heading);


            sleep(2000); // this is drive time

            // do next command or stop

            drive.driveFieldCentric(0, 0,0,heading);
        }

    }
}
