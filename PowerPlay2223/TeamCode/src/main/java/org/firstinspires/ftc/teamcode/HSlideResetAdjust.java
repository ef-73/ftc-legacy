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

@TeleOp(name = "Reset-Adjust HSlideServo")
public class HSlideResetAdjust extends LinearOpMode {

    private Servo hcBlock;
    private Servo hcClaw;
    private Servo hcPan;
    private Servo hcTilt1Left;
    private Servo hcTilt1Right;
    private Servo hcTilt2;
    private double hPanZero = 0.472;
    private double hClawZero = 0.5;
    private double hTilt1Zero = 0.5;
    private double hTilt2Zero = 0.5;
    private double degree180Pos = 1.0 / 162.0;
    private double hTilt1_H_Angle = -10.8;
    private double hTitl2_H_Angle = -44.55;
    private boolean saveFile = false;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

     //   saveZeroValue();
        readHZeroValue();


        hcBlock = hardwareMap.get(Servo.class, "hcBlock");
        hcClaw = hardwareMap.get(Servo.class, "hcClaw");
        hcPan = hardwareMap.get(Servo.class, "hcPan");
        hcTilt1Left = hardwareMap.get(Servo.class, "hcTilt1Left");
        hcTilt1Right = hardwareMap.get(Servo.class, "hcTilt1Right");
        hcTilt2 = hardwareMap.get(Servo.class, "hcTilt2");

        // Put initialization blocks here.
        hcBlock.setDirection(Servo.Direction.FORWARD);
        hcBlock.setPosition(0.5);
        hcClaw.setDirection(Servo.Direction.FORWARD);
        hcClaw.setPosition(hClawZero);
        hcPan.setDirection(Servo.Direction.FORWARD);
        hcPan.setPosition(hPanZero);
        hcTilt1Left.setDirection(Servo.Direction.REVERSE);
        hcTilt1Right.setDirection(Servo.Direction.FORWARD);
        hcTilt1Left.setPosition(hTilt1Zero + hTilt1_H_Angle * degree180Pos);
        hcTilt1Right.setPosition(hTilt1Zero + hTilt1_H_Angle * degree180Pos);
        hcTilt2.setDirection(Servo.Direction.REVERSE);
        hcTilt2.setPosition(hTilt2Zero + hTitl2_H_Angle * degree180Pos);

        updateDisPlay();

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad2.x){
                    hPanZero -= 0.002;
                    hcPan.setPosition(hPanZero);
                }
                else if(gamepad2.b){
                    hPanZero += 0.002;
                    hcPan.setPosition(hPanZero);
                }

                if (gamepad2.y){
                    hTilt2Zero += 0.002;
                    hcTilt2.setPosition(hTilt2Zero + hTitl2_H_Angle * degree180Pos);
                }
                else if(gamepad2.a){
                    hTilt2Zero -= 0.002;
                    hcTilt2.setPosition(hTilt2Zero + hTitl2_H_Angle * degree180Pos);
                }

                if (gamepad2.dpad_up){
                    hTilt1Zero += 0.002;
                    hcTilt1Left.setPosition(hTilt1Zero + hTilt1_H_Angle * degree180Pos);
                    hcTilt1Right.setPosition(hTilt1Zero + hTilt1_H_Angle * degree180Pos);
                }
                else if(gamepad2.dpad_down){
                    hTilt1Zero -= 0.002;
                    hcTilt1Left.setPosition(hTilt1Zero + hTilt1_H_Angle * degree180Pos);
                    hcTilt1Right.setPosition(hTilt1Zero + hTilt1_H_Angle * degree180Pos);
                }

                if (gamepad2.dpad_left){
                    hClawZero -= 0.005;
                    hcClaw.setPosition(hClawZero);
                }
                else if (gamepad2.dpad_right){
                    hClawZero += 0.005;
                    hcClaw.setPosition(hClawZero);
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
        telemetry.addLine("Tilt1 -- GamePad2 dpad_up -- dpad_down");
        telemetry.addLine("Tilt2 -- GamePad2 Y(Up) -- A(Down)");
        telemetry.addLine("Claw -- GamePad2 dpad_left(Open) -- dpad_right(Close");
        telemetry.addLine("Press Right Joystick to Save File");

        telemetry.addData("HPanZero", "%2.4f", hPanZero);

        telemetry.addData("HTilt1", "%2.4f", hTilt1Zero);

        telemetry.addData("HTilt2", "%2.4f", hTilt2Zero);

        telemetry.addData("Claw", "%2.4f", hClawZero);

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
        File file = new File(dir, "hSlideServo.txt");
        try {
            FileOutputStream f = new FileOutputStream(file, false);
            final PrintStream printStream = new PrintStream(f);

            printStream.println(Double.toString(hPanZero));
            printStream.println(Double.toString(hTilt1Zero));
            printStream.println(Double.toString(hTilt2Zero));
            printStream.println(Double.toString(hClawZero));
            printStream.close();
            f.close();
        }
        catch(Exception e){

        }

    }

    private boolean readHZeroValue()
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        File file = new File(dir,"hSlideServo.txt");

        StringBuilder text = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            int lineCnt = 0;
            while ((line = br.readLine()) != null) {
                if (lineCnt == 0){
                    hPanZero= Double.parseDouble(line);
                }else if(lineCnt == 1){
                    hTilt1Zero = Double.parseDouble(line);
                }else if(lineCnt == 2){
                    hTilt2Zero = Double.parseDouble(line);
                }else if(lineCnt == 3){
                    hClawZero = Double.parseDouble(line);
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