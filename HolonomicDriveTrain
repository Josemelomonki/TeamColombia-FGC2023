package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "DriveTrainv11")
public class DriveTrainv11 extends LinearOpMode {
    private DcMotor[] motors = new DcMotor[4];
    private double motorPower = 0.5;
    private double motorPowerTurn = 1;
    private double acceleration = 0.3; // Test to define the final number
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, "Motor" + (i + 1));
            if (i < 2)
                motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
        }
       
        // Initialize the IMU sensor
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set up IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;

        // Initialize the IMU parameters
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get the current orientation angles from the IMU
            Orientation angles = imu.getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Extract the individual orientation angles
            double roll = angles.firstAngle;
            double pitch = angles.secondAngle;
            double yaw = angles.thirdAngle;

            double leftX = gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;

            double frontLeft = leftY + leftX + rightX;
            double frontRight = leftY - leftX - rightX;
            double backLeft = leftY - leftX + rightX;
            double backRight = leftY + leftX - rightX;

            if (leftX != 0 || leftY != 0) {
                // Calcula la diferencia entre las velocidades actuales y las velocidades deseadas
                double deltaFrontLeft = limit(frontLeft) - motors[3].getPower();
                double deltaFrontRight = limit(frontRight) - motors[0].getPower();
                double deltaBackLeft = limit(backLeft) - motors[2].getPower();
                double deltaBackRight = limit(backRight) - motors[1].getPower();

                // Ajusta las velocidades actuales gradualmente
                motors[3].setPower(motors[3].getPower() + deltaFrontLeft * acceleration);
                motors[0].setPower(motors[0].getPower() + deltaFrontRight * acceleration);
                motors[2].setPower(motors[2].getPower() + deltaBackLeft * acceleration);
                motors[1].setPower(motors[1].getPower() + deltaBackRight * acceleration);
            } else if (rightX != 0) {
                // Calcula la diferencia entre las velocidades actuales y las velocidades deseadas
                double deltaFrontLeft = turn(frontLeft) - motors[3].getPower();
                double deltaFrontRight = turn(frontRight) - motors[0].getPower();
                double deltaBackLeft = turn(backLeft) - motors[2].getPower();
                double deltaBackRight = turn(backRight) - motors[1].getPower();

                // Ajusta las velocidades actuales gradualmente
                motors[3].setPower(motors[3].getPower() + deltaFrontLeft * acceleration);
                motors[0].setPower(motors[0].getPower() + deltaFrontRight * acceleration);
                motors[2].setPower(motors[2].getPower() + deltaBackLeft * acceleration);
                motors[1].setPower(motors[1].getPower() + deltaBackRight * acceleration);
            } else if (rightX == 0 && leftX == 0 && leftY == 0) {
                // Los motores no se moverán
                motors[3].setPower(frontLeft);
                motors[0].setPower(frontRight);
                motors[2].setPower(backLeft);
                motors[1].setPower(backRight);
            }

            if ((gamepad1.left_trigger > 0 && gamepad1.left_trigger <1 ) || (gamepad1.right_trigger >0 && gamepad1.right_trigger <1 )) {
                if (gamepad1.left_trigger > 0) {
                    motorPower = 1; // Test to define the final number
                } else if (gamepad1.right_trigger > 0) {
                    motorPower = 0.2;
                    motorPowerTurn = 0.2;
                }
            }
            else if(gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0 ) {
                motorPower = 0.5;
                motorPowerTurn = 1;
            }

            if(gamepad1.right_bumper == true){
                // Los motores no se moverán
                if(roll >=-1 && roll >= 1){
                    motors[3].setPower(0);
                    motors[0].setPower(0);
                    motors[2].setPower(0);
                    motors[1].setPower(0);

                   
                }
                else{
                    motors[3].setPower(-0.5);
                    motors[0].setPower(0.5);
                    motors[2].setPower(-0.5);
                    motors[1].setPower(0.5);
                }
            }
            else if(gamepad1.left_bumper == true){
                if(roll >=-89 && roll >= 91){
                    motors[3].setPower(0);
                    motors[0].setPower(0);
                    motors[2].setPower(0);
                    motors[1].setPower(0);
                   

                }
                else{
                    motors[3].setPower(0.5);
                    motors[0].setPower(-0.5);
                    motors[2].setPower(0.5);
                    motors[1].setPower(-0.5);
                }

            }

            // En el driver hub se verá la posición de los joysticks
            telemetry.addData("Motor Positions", "%d, %d, %d, %d",
                    motors[0].getCurrentPosition(), motors[1].getCurrentPosition(),
                    motors[2].getCurrentPosition(), motors[3].getCurrentPosition());
            telemetry.addData("Joystick Values", "LeftX: %.2f, LeftY: %.2f, RightX: %.2f",
                    leftX, leftY, rightX);
            telemetry.addData("Roll", roll);
            telemetry.addData("Pitch", pitch);
            telemetry.addData("Yaw", yaw);
            telemetry.update();
        }
    }

    private double limit(double value) {
        if (value > motorPower) return motorPower;
        if (value < -motorPower) return -motorPower;
        return value;
    }

    private double turn(double value) {
        if (value > motorPowerTurn) return motorPowerTurn;
        if (value < -motorPowerTurn) return -motorPowerTurn;
        return value;
    }
}


