package org.firstinspires.ftc.teamcode;
//this is a sample class that you can use to make teleop or auto programs
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="test")
public class sample_class extends LinearOpMode {
    robot_hardware robot = new robot_hardware();
    action_lib action = null;
    imu_lib imu = null;

    @Override
    public void runOpMode() throws RuntimeException{
        robot.init(hardwareMap, telemetry);
        action = new action_lib(robot);
        imu = new imu_lib(robot,action);
        waitForStart();
        robot.arm.setPower(0.1);
        double test1 = imu.getAngle();

        /*
        while(true){
            double forward = this.gamepad1.left_stick_x;
            double strafe = this.gamepad1.left_stick_y;
            double turn = -this.gamepad1.right_stick_x;
            action.run_drive(forward-strafe-turn,forward+strafe+turn,
                    -forward+strafe+turn,forward+strafe-turn);
            boolean intake = this.gamepad1.a;
            boolean outake = this.gamepad1.b;
            if (intake == true){
                robot.left_intake.setPower(1);
                robot.right_intake.setPower(1);
            }
            else if(outake){
                robot.left_intake.setPower(-1);
                robot.right_intake.setPower(-1);

            }
            else{
                robot.left_intake.setPower(0);
                robot.right_intake.setPower(0);

            }
        }*/
    }
}
