package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="autoright")
//@Disabled
public class roadrunnertest extends LinearOpMode {

    private DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "front left");
    private DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "back left");
    private DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "front right");
    private DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "back right");
    private DcMotor leftextend = hardwareMap.get(DcMotor.class, "leftextend");
    private DcMotor lefttilt = hardwareMap.get(DcMotor.class, "lefttilt");
    private DcMotor rightextend = hardwareMap.get(DcMotor.class, "rightextend");
    private DcMotor righttilt = hardwareMap.get(DcMotor.class, "righttilt");



    // Set motor directions if necessary (adjust according to your robot configuration)

    public void runOpMode() {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);



        Pose2d initialPose = new Pose2d(0.0, 0.0, Math.toRadians(0));
        //
        Pose2d nextPose = new Pose2d(12.0, 12.0, Math.toRadians(45))
        // Initialize Roadrunner drive system (assuming a mecanum drive)
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();
        new TrajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(12.0, 12.0), Math.toRadians(45))
                .build();



    }






}
