package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ethan_oldbot")
public class Ethan_opmode extends LinearOpMode {

    @Override
    public void runOpMode() {
        Ethan_config robot = new Ethan_config();

        robot.init(hardwareMap);
        robot.arm_tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//hold the arm up
        telemetry.addLine("done calibration, ready to start");
        telemetry.update();
        waitForStart();
        float basketTiltPosition = 0.5f;
        float basketMovePosition = 0.5f;
        while (opModeIsActive()) {
            float leftStickY = -this.gamepad1.left_stick_y;
            float leftStickX = this.gamepad1.left_stick_x;
            float rightStickX = this.gamepad1.right_stick_x;

            boolean intakeBtn = this.gamepad1.left_bumper;
            boolean outakeBtn = this.gamepad1.right_bumper;
            boolean armUp = this.gamepad1.a;
            boolean armDown = this.gamepad1.b;
            boolean armExtend = this.gamepad1.dpad_up;
            boolean armRetract = this.gamepad1.dpad_down;
            boolean basketRight = this.gamepad1.dpad_right;
            boolean basketLeft = this.gamepad1.dpad_left;
            boolean basketUp = this.gamepad1.y;
            boolean basketDown = this.gamepad1.x;


            robot.leftFrontDrive.setPower(leftStickX + leftStickY + rightStickX);
            robot.rightFrontDrive.setPower(-leftStickX + leftStickY - rightStickX);
            robot.leftRearDrive.setPower(-leftStickX + leftStickY + leftStickX);
            robot.rightRearDrive.setPower(leftStickX + leftStickY - rightStickX);

            robot.claw_rotate.setPosition(basketTiltPosition);
            robot.claw_move.setPosition(basketMovePosition);
            if (intakeBtn) {
                robot.intake_right.setPower(1);
                robot.intake_left.setPower(-1);

            } else if (outakeBtn) {
                robot.intake_right.setPower(-1);
                robot.intake_left.setPower(1);
            } else {
                robot.intake_right.setPower(0);
                robot.intake_left.setPower(0);
            }
            if (armUp) {
                robot.arm_tilt.setPower(-0.25);
            } else if (armDown) {
                robot.arm_tilt.setPower(0.25);
            } else {
                robot.arm_tilt.setPower(0);
            }
            if (armExtend) {
                robot.arm_stretch.setPower(0.25);
            } else if (armRetract) {
                robot.arm_stretch.setPower(-0.25);
            } else {
                robot.arm_stretch.setPower(0);
            }
            if (basketUp) {
                basketTiltPosition += 0.05f;
            } else if (basketDown) {
                basketTiltPosition -= 0.05f;
            } else {
                basketTiltPosition += 0;
            }
            if (basketRight) {
                basketMovePosition += 0.05f;
            } else if (basketLeft) {
                basketMovePosition -= 0.05f;
            } else {
                basketMovePosition += 0;
            }


        }
    }
}
