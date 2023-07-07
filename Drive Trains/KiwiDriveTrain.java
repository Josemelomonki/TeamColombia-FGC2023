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

@TeleOp(name = "KiwiDrivev4")
public class KiwiDrivev4 extends LinearOpMode {
    private DcMotor[] motors = new DcMotor[3];
    private double motorPower = 0.5;
    private double motorPowerTurn = 1;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        for (int i = 0; i < 3; i++) {
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

        // Calibrate the IMU
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        sleep(2000); // Espera 2 segundos para asegurarse de que el IMU esté calibrado completamente

        telemetry.addData("Status", "IMU Calibrated");
        telemetry.update();
        sleep(100);

        telemetry.addData("Status", "IMU Calibration Complete");
        telemetry.update();

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

            //Anti Turbo
            if (gamepad1.left_trigger > 0) {
                motorPower = 0.3;
                motorPowerTurn = 0.3;
            //Turbo
            } else if (gamepad1.right_trigger > 0) {
                motorPower = 1;
            //Normal velocity
            } else {
                motorPower = 0.5;
            }

                // Rotate the robot
               double turnPower = -rightX;
               motors[0].setPower(motors[0].getPower() + turnPower);
               motors[1].setPower(motors[1].getPower() + turnPower);
               motors[2].setPower(motors[2].getPower() + turnPower);
}

        //Foward
            if (leftY > 0 && leftX == 0) {
                motors[0].setPower(0);
                motors[1].setPower(motorPower);
                motors[2].setPower(motorPower);

            }

        //Backwards
            if (leftY < 0 && leftX == 0) {
                motors[0].setPower(0);
                motors[1].setPower(-motorPower);
                motors[2].setPower(-motorPower);
                Control(leftX, leftY, rightX);

        
            } else if (leftY != 0 || leftX != 0 || rightX != 0) {
                Control(leftX, leftY, rightX);

            }
            else if (leftY == 0 || leftX == 0 || rightX == 0) {
                motors[0].setPower(0);
                motors[1].setPower(0);
                motors[2].setPower(0);
            }


            telemetry.addData("Motor Positions", "%d, %d, %d",
                    motors[0].getCurrentPosition(), motors[1].getCurrentPosition(),
                    motors[2].getCurrentPosition());
            telemetry.addData("Joystick Values", "LeftX: %.2f, LeftY: %.2f, RightX: %.2f",
                    leftX, leftY, rightX);
            telemetry.addData("Roll", roll);
            telemetry.addData("Pitch", pitch);
            telemetry.addData("Yaw", yaw);
            telemetry.update();
        }
    }

public void Control(double leftX, double leftY, double rightX) {
    double targetPowerFront = -leftY;
    double targetPowerBackLeft = (Math.sqrt(3) / 2) * leftX + (1.0 / 2) * leftY;
    double targetPowerBackRight = -(Math.sqrt(3) / 2) * leftX - (1.0 / 2) * leftY;

    double currentPowerFront = motors[0].getPower();
    double currentPowerBackLeft = motors[1].getPower();
    double currentPowerBackRight = motors[2].getPower();

    double powerChangeRate = 0.1; // Adjust this value to control the acceleration/deceleration rate

    // Gradually adjust the power of each motor towards the target power
    currentPowerFront += (targetPowerFront - currentPowerFront) * powerChangeRate;
    currentPowerBackLeft += (targetPowerBackLeft - currentPowerBackLeft) * powerChangeRate;
    currentPowerBackRight += (targetPowerBackRight - currentPowerBackRight) * powerChangeRate;

    // Assign the updated power values to the motors
    motors[0].setPower(currentPowerFront);
    motors[1].setPower(currentPowerBackLeft);
    motors[2].setPower(currentPowerBackRight);
}
