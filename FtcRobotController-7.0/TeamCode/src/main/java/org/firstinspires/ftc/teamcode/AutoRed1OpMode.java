package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
    Red1 and Blue 1 strategy.--- 26 points
    Robot will spin the duck table 1 time -10
    Robot will deliver the freight at the top level - 6 points
    Park in warehouse- 10 points
*/
/*  Action Items In Order Of Priority

    Advanced Paths -> RoadRunner
    Strafe -> Gyro sensor
    Vision -> OpenCV
*/
@Autonomous(name="AutoRed1OpMode", group="AutoOpModes")
public class AutoRed1OpMode extends BaseAutoOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    String name = "AutoBlue1OpMode";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        robot.arm.closeBucketPos();
        // Send telemetry message to signify robot waiting;
        telemetry.addData(name, " Robot ready for run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Notes: after dropping, ram into wall, go back 3 inches, turn, move to duck spinner

        driveForwardByInches(43, robot, DejaVuBot.TPS);
        turnToPID(-90,robot);
        telemetry.addData(name, "Turned to hub  ");
        telemetry.update();
        //changed from 8->9
        driveForwardByInches(-14/2, robot, DejaVuBot.TPS);

        //Drop the piece here and reset the arm to initial position
        robot.arm.moveArmToLevel(2);
        sleep(500);
        robot.arm.bucketServo.setPosition(0.113);
        sleep(1000);
        robot.arm.bucketServo.setPosition(0.885);
        sleep(500);
        robot.arm.moveArmToLevel(0);
        telemetry.addData(name, " Dropped the freight ");
        telemetry.update();
        //changed from 71 -> 72
        //Move the robot to spin the duck
        driveForwardByInches(72/2, robot, DejaVuBot.TPS*1.2);
        turnToPID(90,robot);
        telemetry.addData(name, " Driving to wall ");
        telemetry.update();

        driveForwardByInches(-31, robot, DejaVuBot.TPS*2);
        driveForwardByInches(-2, robot, DejaVuBot.TPS/2);
        turnToPID(50, robot);
        //f
        driveForwardByInches(-3/2, robot, DejaVuBot.TPS/2);
        spinForOneDuck(robot, true);
        turnToPID(-40,robot);
        //changed from 50 -> 60
        //changed from 55->40 2/2/22
        turnToPID(-40, robot);



        telemetry.addData(name, " Duck spinned ");
        telemetry.update();

        strafeDirection(robot, false, 400);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(name, "Parked in warehouse");
        telemetry.update();
    }
}