package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="HeadlessTest", group="Linear Opmode")
public class MecanumDriveSpeeds extends LinearOpMode {
    MotorEx FL = null;
    MotorEx FR = null;
    MotorEx BL = null;
    MotorEx BR = null;
    MecanumDrive mecanum = null;
    RevIMU imu = null;
    boolean isFieldCentric = true;
    @Override
    public void runOpMode(){
        FL = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_435);
        FR = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_435);
        BL = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_435);
        BR = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435);
        FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FL.setInverted(true);
        FR.setInverted(true);
        BL.setInverted(true);
        BR.setInverted(true);
        mecanum = new MecanumDrive(FL, FR, BL, BR);
        imu = new RevIMU(hardwareMap);
        imu.init();
        GamepadEx driverOp = new GamepadEx(gamepad1);
        while(opModeInInit()){

        }
        while(opModeIsActive()){
            if(isFieldCentric){
                mecanum.driveFieldCentric(driverOp.getLeftX(),driverOp.getLeftY(), driverOp.getRightX(), imu.getRotation2d().getDegrees(), false);
            }
            else{
                mecanum.driveRobotCentric(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(),false);
            }
        }
    }
}
