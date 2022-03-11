package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// 2.4 rotations for top, 1/2 rotation for safe zone
@Disabled
@Autonomous( name="BaseAutoOpMode", group="AutoOpModes")
public class BaseAutoOpMode extends LinearOpMode {
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    private int allowedAngleDiff = 5;
    protected DejaVuBot robot = new DejaVuBot();
    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    public void turnToPID(double targetAngle) {
        robot.gyroInit();
        robot.chassisEncoderOff();
        if(targetAngle < 0){
            turnToPIDForNegativeAngle(targetAngle);
        }else {
            //PIDUtils pid = new PIDUtils(targetAngle, 0.0015, 0, 0.003);
            PIDUtils pid = new PIDUtils(targetAngle, 0.0025, 0.0, 0.002);
            telemetry.setMsTransmissionInterval(50);
            // Checking lastSlope to make sure that it's not "oscillating" when it quits
            double absoluteAngle = getAbsoluteAngle();
            telemetry.addData(" turnToPID start abs angle = ", absoluteAngle);
            telemetry.addData(" turnToPID start diff = ", Math.abs(targetAngle - Math.abs(absoluteAngle)));
            telemetry.addData(" turnToPID start slope = ", pid.getLastSlope());
            telemetry.update();

            while (opModeIsActive()
                    && (Math.abs(targetAngle - Math.abs(getAbsoluteAngle())) > allowedAngleDiff
                        || pid.getLastSlope() > 0.75
            ))
            {
                //absoluteAngle = getAbsoluteAngle();
                double motorPower = pid.update(absoluteAngle);
                telemetry.addData("Absolute Angle:", absoluteAngle);
                telemetry.addData("MotorPower:", motorPower);
                robot.leftFrontMotor.setPower(motorPower);
                robot.leftBackMotor.setPower(motorPower);
                robot.rightFrontMotor.setPower(-motorPower);
                robot.rightBackMotor.setPower(-motorPower);

                telemetry.addData(" turnToPID loop abs angle = ", getAbsoluteAngle());
                telemetry.addData(" turnToPID angle difference = ", Math.abs(targetAngle -
                        Math.abs(getAbsoluteAngle())));
                telemetry.addData(" turnToPID slope = ", pid.getLastSlope());
                telemetry.update();
            }

            robot.setPowerToAllMotors(0);
        }

    }


    public void driveForwardByInches(int distance, double driveVelocity) {
        robot.stopRobot();
        int targetInput = (int) ((48/41)*(distance * DejaVuBot.COUNT_PER_INCH));
        telemetry.addData("Target Position Set to:", targetInput);
        telemetry.update();
        robot.leftFrontMotor.setTargetPosition(targetInput);
        robot.rightFrontMotor.setTargetPosition(targetInput);
        robot.leftBackMotor.setTargetPosition(targetInput);
        robot.rightBackMotor.setTargetPosition(targetInput);
        robot.chassisEncoderOn();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready for run");
        telemetry.update();
        robot.setVelocityToAllMotors(driveVelocity);
        while (opModeIsActive() && robot.leftFrontMotor.isBusy()) {
            telemetry.addData("left", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("right", robot.rightFrontMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addLine("Done moving....");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    public void turnToPIDForNegativeAngle(double targetAngle) {
        //PIDUtils pid = new PIDUtils(targetAngle, 0.0015, 0, 0.003);
        PIDUtils pid = new PIDUtils(targetAngle, 0.0025, 0.0, 0.002);
        //telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not "oscillating" when it quits
        telemetry.addData(" turnToPIDForNegativeAngle s abs angle = ", getAbsoluteAngle());
        telemetry.addData("  turnToPIDForNegativeAngle s  diff = ",
                Math.abs(Math.abs(targetAngle) - getAbsoluteAngle()) );
        telemetry.addData("  turnToPIDForNegativeAngle  start slope = ", pid.getLastSlope());
        telemetry.update();
        //sleep(5*1000);
        //changed here >>>>

        while (opModeIsActive()
                && (Math.abs(Math.abs(targetAngle) - getAbsoluteAngle()) > allowedAngleDiff
                || pid.getLastSlope() > 0.75
        )) {
            double motorPower = pid.update(getAbsoluteAngle());

            robot.leftFrontMotor.setPower(-motorPower);
            robot.leftBackMotor.setPower(-motorPower);
            robot.rightFrontMotor.setPower(motorPower);
            robot.rightBackMotor.setPower(motorPower);

            telemetry.addData(" turnToPIDForNegativeAngle loop abs angle = ",
                    getAbsoluteAngle());
            telemetry.addData(" turnToPIDForNegativeAngle angle difference = ",
                    Math.abs(Math.abs(targetAngle) - getAbsoluteAngle()) );
            telemetry.addData(" turnToPIDForNegativeAngle slope = ", pid.getLastSlope());
            telemetry.update();
        }

        robot.setPowerToAllMotors(0);
    }

    public void spinForOneDuck(boolean clockwise) {
        robot.duckSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(!clockwise)
            robot.duckSpinner.setDirection(DcMotorEx.Direction.FORWARD);
        else
            robot.duckSpinner.setDirection(DcMotorEx.Direction.REVERSE);

        robot.duckSpinner.setTargetPosition(DejaVuBot.ONE_DUCK_SPIN_TARGET_LENGTH);
        robot.duckSpinner.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.duckSpinner.setVelocity(DejaVuBot.TPS);

        telemetry.addData(" target :",robot.duckSpinner.getTargetPosition());
        telemetry.update();

        while (opModeIsActive() && robot.duckSpinner.isBusy()) {
            telemetry.addData(" bot needs to move inches  :",
                    DejaVuBot.ONE_DUCK_SPIN_TARGET_LENGTH);
            telemetry.addData(" spinner position =", robot.duckSpinner.getCurrentPosition());
            telemetry.update();
        }
        //Stop the motor
        robot.duckSpinner.setPower(0);
    }
    public void strafeDirection(boolean left, int milliseconds) {
        robot.setModeForAllMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(left) {
            robot.leftFrontMotor.setPower(-1.0);
            robot.leftBackMotor.setPower(1.0);
            robot.rightFrontMotor.setPower(1.0);
            robot.rightBackMotor.setPower(-1.0);
        } else {
            robot.leftFrontMotor.setPower(1.0);
            robot.leftBackMotor.setPower(-1.0);
            robot.rightFrontMotor.setPower(-1.0);
            robot.rightBackMotor.setPower(1.0);
        }
        sleep(milliseconds);
        robot.setModeForAllMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForAllMotors(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void strafe45Direction(DejaVuBot bot, boolean left, boolean directionForwards, int milliseconds) {
        bot.setModeForAllMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int direction = 0;
        if(left) {
            if (directionForwards) {
                direction = 1;
            } else {
                direction = -1;
            }
            bot.leftFrontMotor.setPower(direction * 1.0);
            bot.leftBackMotor.setPower(0);
            bot.rightFrontMotor.setPower(0);
            bot.rightBackMotor.setPower(direction * 1.0);
        } else {
            if (directionForwards) {
                direction = 1;
            } else {
                direction = -1;
            }
            bot.leftFrontMotor.setPower(0);
            bot.leftBackMotor.setPower(direction * 1.0);
            bot.rightFrontMotor.setPower(direction * 1.0);
            bot.rightBackMotor.setPower(0);
        }
        sleep(milliseconds);
        bot.setModeForAllMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.setModeForAllMotors(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

