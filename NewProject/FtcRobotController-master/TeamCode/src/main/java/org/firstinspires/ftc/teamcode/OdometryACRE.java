package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "OdometryTest", group = "LinearOpMode")
public class OdometryACRE extends LinearOpMode {
    static final double TRACKWIDTH = 37;
    static final double WHEEL_DIAMETER = 6.0;
    static double TICKS_TO_CM ;
    static final double CENTER_WHEEL_OFFSET = 15;


    private HolonomicOdometry Odo = null;
    private OdometrySubsystem OdoSub = null;
    private PurePursuitCommand ppCom = null;
    private MecanumDrive mecanum = null;
    private MotorEx FL, FR, BL, BR = null;
    private MotorEx LE, RE, CE = null;

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
        BR.setInverted(false);
        RE = FL;
        LE = BL;
        CE = BR;
        mecanum = new MecanumDrive(FL, FR, BL, BR);
        TICKS_TO_CM = WHEEL_DIAMETER * Math.PI /8192;
        Odo = new HolonomicOdometry(()-> -LE.getCurrentPosition() * TICKS_TO_CM, ()->RE.getCurrentPosition() * TICKS_TO_CM, ()->CE.getCurrentPosition() * TICKS_TO_CM,TRACKWIDTH,CENTER_WHEEL_OFFSET);
        OdoSub = new OdometrySubsystem(Odo);
        while(opModeInInit()){


        }
        while(opModeIsActive()){
            Waypoint p1 = new StartWaypoint(0,0);
            Waypoint p2 = new GeneralWaypoint(20,0,0.5,0.5,0);
            Waypoint p3 = new PointTurnWaypoint(40,0,0.5,0.5,90,1,1);
            Path m_path = new Path(p1,p2,p3);
            m_path.init();

        }
    }

}
