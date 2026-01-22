package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Controllers Test")
// @Disabled
public class ControllerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {

            // Gamepad 1
            String inputs1 = "";

            telemetry.addData("leftStick1.x", gamepad1.left_stick_x);
            telemetry.addData("leftStick1.y", gamepad1.left_stick_y);
            telemetry.addData("leftStick1.click", gamepad1.left_stick_button);
            telemetry.addData("rightStick1.x", gamepad1.right_stick_x);
            telemetry.addData("rightStick1.y", gamepad1.right_stick_y);
            telemetry.addData("rightStick1.click", gamepad1.right_stick_button);

            if (gamepad1.start) inputs1 += "start | ";
            if (gamepad1.back) inputs1 += "back | ";

            if (gamepad1.y) inputs1 += "y | ";
            if (gamepad1.b) inputs1 += "b | ";
            if (gamepad1.a) inputs1 += "a | ";
            if (gamepad1.x) inputs1 += "x | ";

            if (gamepad1.dpad_up) inputs1 += "up | ";
            if (gamepad1.dpad_right) inputs1 += "right | ";
            if (gamepad1.dpad_down) inputs1 += "down | ";
            if (gamepad1.dpad_left) inputs1 += "left | ";

            if (gamepad1.left_bumper) inputs1 += "lb | ";
            if (gamepad1.right_bumper) inputs1 += "rb | ";

            if (gamepad1.left_trigger != 0) telemetry.addData("LT 1" , gamepad1.left_trigger);
            if (gamepad1.right_trigger != 0) telemetry.addData("RT 1" , gamepad1.right_trigger);

            telemetry.addLine(inputs1);


            // Gamepad 2
            String inputs2 = "";

            telemetry.addData("leftStick1.x", gamepad2.left_stick_x);
            telemetry.addData("leftStick1.y", gamepad2.left_stick_y);
            telemetry.addData("leftStick1.click", gamepad2.left_stick_button);
            telemetry.addData("rightStick1.x", gamepad2.right_stick_x);
            telemetry.addData("rightStick1.y", gamepad2.right_stick_y);
            telemetry.addData("rightStick1.click", gamepad2.right_stick_button);

            if (gamepad2.start) inputs2 += "start | ";
            if (gamepad2.back) inputs2 += "back | ";

            if (gamepad2.y) inputs2 += "y | ";
            if (gamepad2.b) inputs2 += "b | ";
            if (gamepad2.a) inputs2 += "a | ";
            if (gamepad2.x) inputs2 += "x | ";

            if (gamepad2.dpad_up) inputs2 += "up | ";
            if (gamepad2.dpad_right) inputs2 += "right | ";
            if (gamepad2.dpad_down) inputs2 += "down | ";
            if (gamepad2.dpad_left) inputs2 += "left | ";

            if (gamepad2.left_bumper) inputs2 += "lb | ";
            if (gamepad2.right_bumper) inputs2 += "rb | ";

            if (gamepad2.left_trigger != 0) telemetry.addData("LT 2" , gamepad2.left_trigger);
            if (gamepad2.right_trigger != 0) telemetry.addData("RT 2" , gamepad2.right_trigger);

            telemetry.addLine(inputs2);

            telemetry.update();
        }
    }
}