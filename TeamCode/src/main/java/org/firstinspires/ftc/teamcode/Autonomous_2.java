package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoPeriod_2", group="Autonomous")

public class Autonomous_2 extends LinearOpMode {
    private DemoRobotInterface robotui = null;

    public float MotorsSpeed = 0.1f;

    public float wait = 3.0f;



    @Override
    public void runOpMode() throws InterruptedException {
        robotui = new DemoRobotInterface(hardwareMap, telemetry);

        waitForStart();

        robotui.elevatorm.setTargetPosition(1300);
        robotui.elevatorm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotui.elevatorm.setPower(-0.5f);

        robotui.frm.setPower(0.2);
        robotui.flm.setPower(-0.2);

        if(robotui.color.argb() == 100)
        {
            //first move
        }
        else if(robotui.color.argb() == 300)
        {
            //second move
        }
        else if(robotui.color.argb() == 999)
        {
            //third move
        }

        while (opModeIsActive())
        {
            telemetry.addData("Color", robotui.color.argb());
            telemetry.update();
        }
    }
}
