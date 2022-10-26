package org.firstinspires.ftc.teamcode;

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
        dr.run();
        while(opModeInInit()){
            telemetry.addData("Left, Right, Aux: ", "%6d %6d %6d",dr.currLpos, dr.currRpos, dr.currApos);
            telemetry.addData("XYH: ", "%6d %6d %6d", dr.pos.x, dr.pos.y, Math.toDegrees(dr.pos.heading));
            telemetry.addData("Thread State Alive: ", dr.isAlive()? "Yes": "No");
            telemetry.update();
        }
    }

}
