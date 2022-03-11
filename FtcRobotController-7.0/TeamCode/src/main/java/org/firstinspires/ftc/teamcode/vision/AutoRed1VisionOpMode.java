package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DejaVuArm;
import org.firstinspires.ftc.teamcode.DejaVuBot;

/**
 * This class represents the autonomous run from Red1 position
 */
@Autonomous(name="AutoRed1VisionOpMode", group="AutoOpModes")
public class AutoRed1VisionOpMode extends BaseAutoVisionOpMode {
    private String TAG = "AutoRed1VisionOpMode";
    private ElapsedTime runtime = new ElapsedTime();
    private Thread levelFinderThread;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        robot.arm.closeBucketPos();
        currentLevel = -1;
        redFlag = true;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.8, 16.0/9.0);
        }
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        runtime.reset();

        Log.d(TAG, "starting thread 1");
        levelFinderThread = new Thread(levelFinderRunnable);
        levelFinderThread.start();

        // now wait for the threads to finish before returning from this method
        Log.d(TAG, "waiting for threads to finish...");
        levelFinderThread.join();
        Log.d(TAG, "thread joins complete");
        //46
        driveForwardByInches(40,  DejaVuBot.TPS);
        turnToPID(-90);


        telemetry.addLine("Turned 90 degrees to align");
        telemetry.addLine("Moving Arm to level: " + currentLevel);
        telemetry.update();
        driveForwardByInches(-9,  DejaVuBot.TPS);

        robot.arm.moveArmToLevel(currentLevel);
        sleep(500);
        robot.arm.openBucketPos();
        sleep(1000);
        robot.arm.closeBucketPos();
        sleep(1000);
        robot.arm.moveArmToLevel(0);
        telemetry.addData("name", " Dropped the freight ");
        telemetry.update();

        driveForwardByInches(74/2, DejaVuBot.TPS);
        turnToPID(90);
        turnToPID(90);
        turnToPID(50);
        telemetry.addData("AutoRed1VisionOpMode:", " Driving to wall ");
        telemetry.update();

        driveForwardByInches(-31, DejaVuBot.TPS*2);
        driveForwardByInches(-2, DejaVuBot.TPS/2);
        turnToPID(-50);

//
//        //Move the robot to warehouse for second point
//        driveForwardByInches(2, robot, DejaVuBot.TPS);
//        strafeDirection(robot, false, 920);
//
//        robot.arm.closeBucketPos();
//        //robot.intake();
//        driveForwardByInches(45, robot, DejaVuBot.TPS);
//        strafeDirection(robot, true, 500);
//
//        telemetry.addData("name", "Parked in warehouse");
//        telemetry.update();
        /*
        //Move to 135 degree and drive forward.
        turnToPID(135,robot);
        Log.i(TAG, " Turned to hub ");
        driveForwardByInches(-24, robot, DejaVuBot.TPS);
        Log.i(TAG, " drove forward =");
        //Set to top level if vision did not detect the team element
        telemetry.addData(TAG, " Vision detected current level = "+ currentLevel);
        //TODO : WE dont have level 0 bucket drop so set level to top
        if(currentLevel < DejaVuArm.MID_LEVEL ) {
            currentLevel = DejaVuArm.TOP_LEVEL;
            Log.i(TAG, " adjusted the level to top level "+ currentLevel);
        }
        telemetry.addData(TAG, " Updated current level set to  "+ currentLevel);
        robot.arm.moveArmToLevel(currentLevel);
        Log.i(TAG, " Moved the arm to level  "+ currentLevel);
        robot.arm.openBucketPos();
        Log.i(TAG, " Bucket open called ");
        robot.arm.closeBucketPos();
        Log.i(TAG, " Bucket closed ");
        robot.arm.moveArmToLevel(0);
        Log.i(TAG, " arm moved down ");
        telemetry.addData(TAG, " Dropped the freight ");
        telemetry.update();

        //Go to wall
        turnToPID(-45,robot);
        Log.i(TAG, " aligning with wall ");
        strafeDirection(robot, false, 200);
        Log.i(TAG, " strafing to wall complete ");
        driveForwardByInches(50, robot, DejaVuBot.TPS);
        Log.i(TAG, " In warehouse to pickup second block ");
        robot.intake();
        sleep(500);
        robot.stopIntake();

        //TODO - add strafe back to drop the second freight
        //strafeDirection(robot, false, 200);
        //strafe45Direction(robot,true,false,50);

        // Step 4:  Stop and close the claw.
        robot.stopRobot();

        telemetry.addData(TAG, "Parked in warehouse");
        telemetry.update();
        */
        // always stop the robot
        robot.stopRobot();
    }

    private Runnable levelFinderRunnable = new Runnable() {
        @Override
        public void run() {
            //driveForwardByInches(4, robot, DejaVuBot.TPS);
            //Find the level in 10 attempts. If not detected set level to 3.
            if (opModeIsActive() && tfod != null) {
                telemetry.addData(">", "Detecting level using vision");
                telemetry.update();
                int count = 0;
                int lastResult = -1;
                while (opModeIsActive() && count < 3) {
                    lastResult = currentLevel;
                    currentLevel = findLevel();
                    telemetry.addData("Last Level", lastResult);
                    telemetry.addData("Current Level", currentLevel);
                    Log.i(TAG, "Called find level for "
                            + count+ " detected current level = "
                            + currentLevel + " (last level=" + lastResult + ")");
                    if(currentLevel != -1){
                        Log.i(TAG, " Setting Final level ="+ currentLevel);
                    }
                    // go back to last result if current result is not good
                    if(lastResult != -1){
                        Log.i(TAG, " Setting Final level ="+ lastResult);
                        currentLevel = lastResult;
                    }
                    count++;
                    Log.i(TAG, " count ="+ count);
                    telemetry.addData("Retry count:", count);
                }

                if(currentLevel == DejaVuArm.BOTTOM_LEVEL) {
                    telemetry.addLine(" Current level discovered is Bottom level");
                } else if(currentLevel == DejaVuArm.TOP_LEVEL) {
                    telemetry.addLine(" Current level discovered is Top level");
                } else if(currentLevel == DejaVuArm.MID_LEVEL) {
                    telemetry.addLine(" Current level discovered is Mid level");
                } else {
                    telemetry.addLine(" Current level discovered is UNKNOWN - defaulting to TOP");
                    currentLevel = DejaVuArm.TOP_LEVEL;
                }
                telemetry.update();
            } else{
                telemetry.addData(">", "Could not init vision code - defaulting to TOP");
                telemetry.update();
                currentLevel = DejaVuArm.TOP_LEVEL;
            }
            Log.d(TAG, "Thread 1 finishing up");
        }
    };
}