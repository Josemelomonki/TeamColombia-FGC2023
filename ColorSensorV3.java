package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SensorColor extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();

            if (color.red() > color.green() && color.red() > color.blue()) {
                telemetry.addData("Dominant Color", "Red");
                telemetry.update();
            }

            if (color.green() > color.red() && color.green() > color.blue()) {
                telemetry.addData("Dominant Color", "Green");
                telemetry.update();
            }

            if (color.blue() > color.green() && color.blue() > color.red()) {
                telemetry.addData("Dominant Color", "Blue");
                telemetry.update();
            }
        }
    }
}


