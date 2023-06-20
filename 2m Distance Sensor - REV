package org.firstinspires.ftc.teamcode;
 
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 
@TeleOp
public class SensorDistance extends LinearOpMode {
    DistanceSensor distance;
    DcMotor motor;
    
    @Override
    public void runOpMode() {
        // Get the distance sensor and motor from hardwareMap
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        motor = hardwareMap.get(DcMotor.class, "Motor");
        
        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // If the distance in centimeters is less than 6, set the power to 0.3
            if (distance.getDistance(DistanceUnit.CM) < 7.5) {
                motor.setPower(0.3);
            } else {  // Otherwise, stop the motor
                motor.setPower(0);
            }
        }
    }
}


