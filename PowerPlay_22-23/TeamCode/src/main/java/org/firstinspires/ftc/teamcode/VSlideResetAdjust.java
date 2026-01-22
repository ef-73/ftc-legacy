package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;

@TeleOp(name = "Reset-Adjust VSlideServo")
public class VSlideResetAdjust extends LinearOpMode {

    private Servo vcClaw;
    private Servo vcPan1;
    private Servo vcPan2;
    private Servo vcRotate;
    private Servo vcTiltLeft;
    private Servo vcTiltRight;
    private double vPanZero = 0.492;
    private double vTiltZero = 0.5;
    private double vRotateZero = 0.5;
    private double vClawZero = 0.5;

    private boolean saveFile = false;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
       // saveZeroValue();
        readVZeroValue();
        vcClaw = hardwareMap.get(Servo.class, "vcClaw");
        vcPan1 = hardwareMap.get(Servo.class, "vcPan1");
        vcPan2 = hardwareMap.get(Servo.class, "vcPan2");
        vcRotate = hardwareMap.get(Servo.class, "vcRotate");
        vcTiltLeft = hardwareMap.get(Servo.class, "vcTiltLeft");
        vcTiltRight = hardwareMap.get(Servo.class, "vcTiltRight");

        // Put initialization blocks here.
        vcClaw.setDirection(Servo.Direction.FORWARD);
        vcClaw.setPosition(vClawZero);
        vcPan1.setDirection(Servo.Direction.FORWARD);
        vcPan2.setDirection(Servo.Direction.FORWARD);
        vcPan1.setPosition(vPanZero);
        vcPan2.setPosition(vPanZero);
        vcRotate.setDirection(Servo.Direction.FORWARD);
        vcRotate.setPosition(vRotateZero);
        vcTiltLeft.setDirection(Servo.Direction.REVERSE);

        vcTiltRight.setDirection(Servo.Direction.FORWARD);
        vcTiltLeft.setPosition(vTiltZero);
        vcTiltRight.setPosition(vTiltZero);
        updateDisPlay();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad2.x){
                    vPanZero -= 0.002;
                    vcPan1.setPosition(vPanZero);
                    vcPan2.setPosition(vPanZero);
                }
                else if(gamepad2.b){
                    vPanZero += 0.002;
                    vcPan1.setPosition(vPanZero);
                    vcPan2.setPosition(vPanZero);
                }

                if (gamepad2.y){
                    vTiltZero += 0.002;
                    vcTiltLeft.setPosition(vTiltZero);
                    vcTiltRight.setPosition(vTiltZero);
                }
                else if(gamepad2.a){
                    vTiltZero -= 0.002;
                    vcTiltLeft.setPosition(vTiltZero);
                    vcTiltRight.setPosition(vTiltZero);
                }

                if (gamepad2.dpad_up){
                    vRotateZero += 0.002;
                    vcRotate.setPosition(vRotateZero);
                }
                else if(gamepad2.dpad_down){
                    vRotateZero -= 0.002;
                    vcRotate.setPosition(vRotateZero);
                }

                if (gamepad2.dpad_left){
                    vClawZero -= 0.005;
                    vcClaw.setPosition(vClawZero);
                }
                else if (gamepad2.dpad_right){
                    vClawZero += 0.005;
                    vcClaw.setPosition(vClawZero);
                }

                if (gamepad2.right_stick_button){
                    saveZeroValue();
                }


                updateDisPlay();
                sleep(100);
            }
        }

    }

    private void updateDisPlay(){
        telemetry.addLine("Pan -- GamePad2 left(X) -- Right(B)");
        telemetry.addLine("Rotate -- GamePad2 dpad_up -- dpad_down");
        telemetry.addLine(" Tilt-- GamePad2 Y(Up) -- A(Down)");
        telemetry.addLine("Claw -- GamePad2 dpad_left(Open) -- dpad_right(Close");
        telemetry.addLine("Press Right Joystick to Save File");

        telemetry.addData("VPanZero", "%2.4f", vPanZero);

        telemetry.addData("VTilt", "%2.4f", vTiltZero);

        telemetry.addData("VRotate", "%2.4f", vRotateZero);

        telemetry.addData("Claw", "%2.4f", vClawZero);

        if (saveFile){
            telemetry.addLine("Zero value saved");
        }
        else{
            telemetry.addLine("Not save Zero value");
        }
        telemetry.update();
    }


    private void saveZeroValue()
    {
        saveFile = true;
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        dir.mkdirs();
        File file = new File(dir, "vSlideServo.txt");
        try {
            FileOutputStream f = new FileOutputStream(file, false);
            final PrintStream printStream = new PrintStream(f);

            printStream.println(Double.toString(vPanZero));
            printStream.println(Double.toString(vTiltZero));
            printStream.println(Double.toString(vRotateZero));
            printStream.println(Double.toString(vClawZero));
            printStream.close();
            f.close();
        }
        catch(Exception e){

        }

    }

    private boolean readVZeroValue()
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        File file = new File(dir,"vSlideServo.txt");

        StringBuilder text = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            int lineCnt = 0;
            while ((line = br.readLine()) != null) {
                if (lineCnt == 0){
                    vPanZero= Double.parseDouble(line);
                }else if(lineCnt == 1){
                    vTiltZero = Double.parseDouble(line);
                }else if(lineCnt == 2){
                    vRotateZero = Double.parseDouble(line);
                }else if(lineCnt == 3){
                    vClawZero = Double.parseDouble(line);
                }

                lineCnt ++;
            }
            telemetry.update();
            br.close();
            return true;
        }
        catch (IOException e) {
            //You'll need to add proper error handling here
            return false;
        }
    }
}