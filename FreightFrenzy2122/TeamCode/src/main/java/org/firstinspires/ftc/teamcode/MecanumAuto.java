package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumAutoDrive;
import org.firstinspires.ftc.teamcode.MecanumChassis;


import java.util.Locale;


@Autonomous(name="MecanumDrive: Auto Drive By Encoder and IMU", group="MecanumDrive")
//@Disabled


public class MecanumAuto extends LinearOpMode {
    /* Declare OpMode members. */
    MecanumChassis robot   = new MecanumChassis();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    // State used for updating telemetry
    volatile Orientation angles;        //just for telemetry
    Acceleration gravity;


    //Mecanum auto drive class
    MecanumAutoDrive mecanumAutoDrive = null;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    @Override
    public void runOpMode()  throws InterruptedException{
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.addData("IMU Mode", "IMU calibrating....");
        telemetry.update();

        //make sure the IMU gyro is calibrated before continue
        while(!isStopRequested() && ! robot.imu.isGyroCalibrated() &&
                                    ! robot.imu.isAccelerometerCalibrated() &&
                                    ! robot.imu.isMagnetometerCalibrated() &&
                                    ! robot.imu.isSystemCalibrated())
        {
            idle();
        }

        composeTelemetry();
        telemetry.update();


        telemetry.addData("Working Mode", "waiting for start");
        telemetry.update();
        mecanumAutoDrive = new MecanumAutoDrive(robot, telemetry);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // only drive base on encoder
        /*
        //mecanumAutoDrive.encoderDrive(0.2,  1.2,  1.2, 10.0);  // S1: Forward 50cm with 5 Sec timeout
        //mecanumAutoDrive.encoderTurn(0.2, -180, MecanumAutoDrive.TURN_METHOD.LEFT_WHEEL, 10);
        //mecanumAutoDrive.encoderDrive(0.2,   0.5, 0.5, 10.0);  // S2: Turn Right 12cm with 4 Sec timeout
        //mecanumAutoDrive.encoderDrive(0.5, 1.0, 0, 10.0);  // S3: Reverse 24 cm with 4 Sec timeout
        */

        // drive based on encoder + gyro 1.2m = 2 tiles
        //mecanumAutoDrive.goStraightTask(0.05, 0, 0.05, 0,5);
        /*
        mecanumAutoDrive.goStraightTask(1.2, 0, 1.0, 0.02, 5);
        mecanumAutoDrive.turnRobotTask(-90,1.0,1.0, MecanumAutoDrive.TURN_METHOD.TWO_WHEEL, 5);
        mecanumAutoDrive.goStraightTask(0.5, -90, 1.0, 0.01,5);
        mecanumAutoDrive.goStraightTask(0.5, -90, -1.0, 0.01,5);
        mecanumAutoDrive.turnRobotTask(0,1.0,1.0, MecanumAutoDrive.TURN_METHOD.TWO_WHEEL, 5);
        mecanumAutoDrive.goStraightTask(1.2, 0, -1.0, 0.02, 5);
        */
        while(true){
            mecanumAutoDrive.getHeadingAngle();
            // Read the sensor
//            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
//                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
//                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
//                    hsvValues);
//
//            // send the info back to driver station using telemetry function.
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Alpha", robot.sensorColor.alpha());
//            telemetry.addData("Red  ", robot.sensorColor.red());
//            telemetry.addData("Green", robot.sensorColor.green());
//            telemetry.addData("Blue ", robot.sensorColor.blue());
//            telemetry.addData("Hue", hsvValues[0]);


            telemetry.addData("HeadingF=","%.1f",mecanumAutoDrive.getHeadingAngle());


            telemetry.update();
           idle();


        }

/*
        telemetry.addData("Path0",  "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.update();
*/
    }


        //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = robot.imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);


                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        /*
        telemetry.addLine()
                .addData("vel", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getVelocity().toString();
                    }
                })
                .addData("pos", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getPosition().toString();
                    }
                });
        */

    /*
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
                */

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
