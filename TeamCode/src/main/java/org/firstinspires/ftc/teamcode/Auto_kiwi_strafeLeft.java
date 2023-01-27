package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Kiwi_auto_StrafeLeft_1", group="Opmode")
public class Auto_kiwi_strafeLeft extends LinearOpMode {

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


        Timing.Timer autoTime = new Timing.Timer((long) 1.25, TimeUnit.SECONDS);
        //Timing.Timer autoTime = new Timing.Timer(2000);
        waitForStart();
        autoTime.start();
        // put auto drive commands here
        // drive ahead to parking spot

       // double heading = - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while(!autoTime.done()) {
            // drive values are: strafeSpeed, forward speed, turn, heading
            double heading = - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            drive.driveFieldCentric(-0.75, 0, 0, heading);

        }


        // do next command or stop
        //double heading = - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
      //  drive.driveFieldCentric(0, 0,0,heading);


    }
}


