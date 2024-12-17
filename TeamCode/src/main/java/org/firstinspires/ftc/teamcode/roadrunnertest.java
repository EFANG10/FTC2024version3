package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="roadrunnertest")
//@Disabled
public class roadrunnertest extends LinearOpMode {

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx leftExtend = null;
    private DcMotorEx leftTilt = null;
    private DcMotorEx rightExtend = null;
    private DcMotorEx rightTilt = null;



    // Set motor directions if necessary (adjust according to your robot configuration)

    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "front left");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "back left");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "front right");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "back right");
        rightTilt = hardwareMap.get(DcMotorEx.class, "righttilt");
        leftTilt = hardwareMap.get(DcMotorEx.class, "lefttilt");
        leftExtend = hardwareMap.get(DcMotorEx.class, "leftextend");
        rightExtend = hardwareMap.get(DcMotorEx.class, "rightextend");


        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);



        Pose2d initialPose = new Pose2d(0.0, 0.0, Math.toRadians(0));
        //
        Pose2d nextPose = new Pose2d(12.0, 12.0, Math.toRadians(45));
        // Initialize Roadrunner drive system (assuming a mecanum drive)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();



    }






}
