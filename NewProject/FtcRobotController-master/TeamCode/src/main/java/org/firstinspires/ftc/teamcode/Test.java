package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="OdoTest", group="Linear Opmode")
//@Disabled
public class Test extends LinearOpMode {

    private Drivetrain dr = null;
    @Override
    public void runOpMode(){
        dr = new Drivetrain(hardwareMap, this);
        while(opModeInInit()){
            dr.Odometry();
            telemetry.addData("Left: ", dr.currLpos);
            telemetry.addData("Right: ", dr.currRpos);
            telemetry.addData("Aux: ", dr.currApos);
            telemetry.addData("X: ", dr.pos.x);
            telemetry.addData("Y: ", dr.pos.y);
            telemetry.addData("H: ", Math.toDegrees(dr.pos.heading));
            telemetry.update();
        }
        while (opModeIsActive()){
            telemetry.update();
            dr.goToPos(0,30,0,0.2,1);
            break;
        }
    }

}
