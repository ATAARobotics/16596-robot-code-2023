package org.firstinspires.ftc.teamcode;
//  to connect/push code to DS phone  via wifi you must:
// (1) connect DS phone to laptop via usb
// (2) go to "tools-external" and run " enable ADB over TCPIP"
//  (3) if successful (exit code =0) then run " connect to ADB over wifi direct"

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;

// this is just a dumb wrapper
//import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;


@TeleOp(name="Kiwi_comp_2_newclaw", group="Opmode")
public class KiwiDrive extends OpMode {
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

    // time-tracking
    private double last_time = 0.0;

    // controller-mode
    private int mode = 0;  // default

    // setup for various controls
    private GamepadEx gamepadex1 = null; // this is driver?
    private GamepadEx gamepadex2 = null; // this is operator?
    private ButtonReader bump_left = null;
    private ButtonReader bump_right = null;
    private ButtonReader button_a = null;
    private TriggerReader trigger_left = null;
    private TriggerReader trigger_right = null;
    private TriggerReader trigger_left2 = null;
    private TriggerReader trigger_right2 = null;

    @Override
    public void init() {
        telemetry.addData("status", "startup");
        telemetry.update();

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

        // setup some controller listeners
        gamepadex1 = new GamepadEx(gamepad1);
        gamepadex2 = new GamepadEx(gamepad2);

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
        drive.setMaxSpeed(0.90); // 0.0 to 1.0, percentage of "max"

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Executed repeatedly after a user presses INIT but before a
        // user presses Play (▶) on the Driver Station
    }

    @Override
    public void start() {
        // Executed once immediately after a user presses Play (▶) on
        // the Driver Station

        // make sure robot starts at correct position
        imu.resetYaw();
        elevator_motor.resetEncoder();
       // servo_claw_right.setPosition(0);
        servo_claw_left.setPosition(0.35);


    }

    @Override
    public void loop() {
        // Executed repeatedly after a user presses Play (▶) but
        // before a user presses Stop (◼) on the Driver Station

        double diff = time - last_time;
        last_time = time;

       //servo_claw_left.setPosition(0.5);// temporary for testing
       //servo_claw_right.setPosition((0.5));

        telemetry.addData("time", time);
        telemetry.addData("left servo:  ",servo_claw_left.getPosition()); // temporary for testing
        telemetry.addData("right servo:  ",servo_claw_right.getPosition()); // temporary for testing

        // let FTCLib update its button status
        gamepadex1.readButtons();
        gamepadex2.readButtons();
// Variables for elevator limits, drive train limits
         int poslow = 672;
         int posmed = 902;
         int poshigh = 1142;
         double elevSpeed = 0.65; // range is 0 to 1
         int lowlim = 0;
         int highlim = 970;
         //servo variables only needed if we cant get Srs programmer to work;  if it works, then we'd set servos to 0 or 1, i.e. their new min/max angles
        // int servoLeftMaxAngle = 0;
        // int servoLeftMinAngle = 0;
      //  int servoRightMaxAngle = 0;
       // int servoRightMinAngle = 0;

        // open claw fingers while button is pressed; confirm with drivers OR do binary: press once to grab, press again to release
         //if(gamepadex2.getButton(GamepadKeys.Button.LEFT_BUMPER))// need to put in proper button for claw...
            //  the direction of these needs to be tested before putting the fingers on the servos!!
            //servo_claw_left.setPosition(0);

//====================  disabled elevattor presets ===========================================
        /*if (gamepadex2.getButton(GamepadKeys.Button.X)) {
            if (elevator_motor.getCurrentPosition() > lowlim) {
                elevator_motor.set(-elevSpeed);
            } else if (elevator_motor.getCurrentPosition() <= lowlim) {
                elevator_motor.set(0);
            }
        }

         if (gamepadex2.getButton(GamepadKeys.Button.A)) {
             if (elevator_motor.getCurrentPosition() < poslow) {
                 elevator_motor.set(elevSpeed*2);
             } else if (elevator_motor.getCurrentPosition() >= poslow) {
                 elevator_motor.set(0);
             }
         }

        if (gamepadex2.getButton(GamepadKeys.Button.B)) {
            if (elevator_motor.getCurrentPosition() < posmed) {
                elevator_motor.set(elevSpeed);
            } else if (elevator_motor.getCurrentPosition() >= posmed) {
                elevator_motor.set(0);
            }
        }*/

        telemetry.addData("elevator_position", elevator_motor.getCurrentPosition());
        if (gamepadex2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            if (elevator_motor.getCurrentPosition() <= highlim) {
                elevator_motor.set(elevSpeed);
                telemetry.addData("elevator","up");
            }
        } else if (gamepadex2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            if (elevator_motor.getCurrentPosition() >= lowlim) {
                elevator_motor.set(-.7*elevSpeed);
                telemetry.addData("elevator","down");
            }
        } else {
            elevator_motor.set(0);
            telemetry.addData("elevator","stop");
        }


        /*// left / right BUMPERs switch mode
        if (gamepadex1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            mode -= 1;
        } else if (gamepadex1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            mode += 1;
        }
        if (mode < 0) mode = 2;
        if (mode > 2) mode = 0;*/
        mode = 2;
        telemetry.addData("mode", mode);

        // allow us to reset the yaw?
      //  if (gamepadex1.wasJustPressed(GamepadKeys.Button.A)) {
      //      imu.resetYaw();
     //   }
        // speed controls (percentage of max)
        double max_speed = 0.45;
        if (gamepadex1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
            // if left-trigger "pressed"
            max_speed = 0.30;
        } else if (gamepadex1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
            // if ONLY right-trigger "pressed"
            max_speed = 1;
        }
        drive.setMaxSpeed(max_speed);
        telemetry.addData("max_speed", max_speed);
        telemetry.addData("Right_trigger", gamepadex1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        telemetry.addData("Left trigger", gamepadex1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        if (mode == 0) {
            // simple at first: left-strick forward/back + turn
            drive.driveRobotCentric(
                0.0, // strafe speed
                gamepad1.left_stick_y,  // forward/back (only) from left stick
                gamepad1.right_stick_x / 2.0 // turn from right stick, but less input
           );
        } else if (mode == 1) {
            drive.driveRobotCentric(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x / 2.0
            );
        } else if (mode == 2) {
            double heading = - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("heading", heading);

            drive.driveFieldCentric(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x /2,
                heading
            );
        }
// Grab a cone:
        if (gamepadex2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
          //  servo_claw_right.setPosition(0.41);
        //    servo_claw_left.setPosition(0.4);
              servo_claw_right.setPosition(1);
              servo_claw_left.setPosition(0);
              telemetry.addData("claw","has cone");
        }

// Release a cone:
        if (gamepadex2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
           // servo_claw_right.setPosition(0.45); old claw
           // servo_claw_left.setPosition(0.25); old claw
            servo_claw_right.setPosition(0);
            servo_claw_left.setPosition(1);
            telemetry.addData("claw","can take cone");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        // Executed once immediately after a user presses Stop (◼) on
        // the Driver Station

        // servo reset  to release cone, if has cone zero
        servo_claw_right.setPosition(0.45);
        servo_claw_left.setPosition(0.25);
    }
}
