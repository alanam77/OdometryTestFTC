package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SimpleTeleOp extends LinearOpMode {
    private Drivetrain dr = null;
    @Override
    public void runOpMode(){
        dr = new Drivetrain(hardwareMap, this);
        while(opModeInInit()){

        }
        while(opModeIsActive()){
            double r = Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            dr.FL.setPower(v1);
            dr.FR.setPower(v2);
            dr.BL.setPower(v3);
            dr.BR.setPower(v4);
        }
    }
}
