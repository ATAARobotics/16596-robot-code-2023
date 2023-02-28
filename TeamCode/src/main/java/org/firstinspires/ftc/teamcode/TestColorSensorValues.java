package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Test Colour Sensor", group="TeleOp")

public class TestColorSensorValues extends OpMode {
    //Define variable for color sensor
    private ColorSensor color = null;
    private DistanceSensor distance = null;
    private String found_colour = "unknown";

    @Override
    public void init() { // called when o
        telemetry.addData("status", "startup");
        telemetry.update();

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(true);

        distance = hardwareMap.get(DistanceSensor.class, "color");

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
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "color");

        distance = hardwareMap.get(DistanceSensor.class, "color");
        // turn off LED
        color.enableLed(false);
    }
    private String update_colour() {
        String detected_colour = "unknown";
        int r = color.red();
        int g = color.green();
        int b = color.blue();
        int max = Math.max(color.alpha(), Math.max(b, Math.max(r, g)));
        if (r == max) { detected_colour = "red"; }
        if (g == max) { detected_colour = "green"; }
        if (b == max) { detected_colour = "blue"; }
        telemetry.addData(
                "colour",
                String.format("colour: red=%d green=%d blue=%d max=%d detected=%s", r, g, b, max, detected_colour)
        );
        return detected_colour;
    }

        @Override
        public void loop() {
            // Executed repeatedly after a user presses Play (▶) but
            // before a user presses Stop (◼) on the Driver Station

           // turn on LED
            color.enableLed(true);
            double distance_mm = distance.getDistance(DistanceUnit.MM);
            update_colour();
            // get encoder colour values:
            telemetry.addData("detected:  ",update_colour());
            // most reliable at distance of

           /* telemetry.addData("RED",color.red());
            telemetry.addData("Green",color.green());
            telemetry.addData("blue: ",color.blue());*/
            telemetry.addData("distance= ",distance_mm);
            telemetry.update();

        }

    }
     //color.enableLed(false)

