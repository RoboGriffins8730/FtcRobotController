/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")

public class RobotAutoDriveByTime_Linear extends LinearOpMode {


    /* Declare OpMode members. */

    DcMotor launcherLeft;
    DcMotor launcherRight;
    DcMotor conveyor;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intake;
    Limelight3A camera;

    double xpos;
    double zpos;
    double heading;

    enum DriveState {
        SHOOTING, INTAKE1, INTAKE2, INTAKE3, END;
    }
    DriveState driveState = DriveState.SHOOTING;

    IMU imu;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.

        launcherLeft = hardwareMap.get(DcMotor.class, "launcher_left");
        launcherRight = hardwareMap.get(DcMotor.class, "launcher_right");
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        camera = hardwareMap.get(Limelight3A.class, "camera");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));

        camera.pipelineSwitch(0);
        camera.start();
        /*
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
*/
        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = camera.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D robotPose = result.getBotpose_MT2();

                xpos = robotPose.getPosition().x;
                zpos = robotPose.getPosition().z;
                heading = robotPose.getOrientation().getYaw();

                telemetry.addData("x position (m)", xpos);
                telemetry.addData("z position (m)", zpos);
                telemetry.addData("heading (deg)", Math.toDegrees(heading));
            }
            telemetry.update();

            //all calls to drivingRobot are placeholder
            switch (driveState) {
                case SHOOTING:
                    if (drivingRobot(4, 4, 60)) {
                        conveyorShooting();
                        driveState = DriveState.INTAKE1;
                        break;
                    }
                    break;
                case INTAKE1:
                    if (drivingRobot(3, 3, 0)) {
                        intakeConveyor();
                        if (drivingRobot(2, 3, 0)) {
                            intakeConveyor();
                            driveState = DriveState.SHOOTING;
                            break;
                        }
                    }
                    break;
                case INTAKE2:
                    if (drivingRobot(7, 3, 0)) {
                        intakeConveyor();
                        if (drivingRobot(2, 3, 0)) {
                            intakeConveyor();
                            driveState = DriveState.SHOOTING;
                            break;
                        }
                    }
                    break;
                case INTAKE3:
                    if (drivingRobot(3, 3, 0)) {
                        intakeConveyor();
                        if (drivingRobot(4, 3, 0)) {
                            intakeConveyor();
                            driveState = DriveState.SHOOTING;
                            break;
                        }
                    }
                    break;
                case END:
                    if (drivingRobot(5, 5, 90)) {
                        break;
                    }
            }
        }

    }

    public void intakeConveyor() {
        intake.setPower(80);
        conveyor.setPower(80);
        sleep(2000);
        intake.setPower(0);
        conveyor.setPower(0);
    }

    public void conveyorShooting() {
        launcherLeft.setPower(100);
        launcherRight.setPower(100);
        sleep(500);
        conveyor.setPower(80);
        sleep(2000);
        launcherRight.setPower(0);
        launcherLeft.setPower(0);
        conveyor.setPower(0);

    }
    public boolean drivingRobot(double targetX, double targetZ, double targetHeading) {
        //this gets the error relative to the field
        double errorX = targetX - xpos;
        double errorZ = targetZ - zpos;
        double errorHeading = targetHeading - heading;

        //this converts the error so it is relative to robot
        double forwardError = errorX * Math.sin(heading) + errorZ * Math.cos(heading);
        double strafeError = errorX * Math.cos(heading) - errorZ * Math.sin(heading);

        //this is proportional control, so the robot slows down as the error decreases
        double forwardPower = 0.5 * forwardError;
        double strafePower = 0.5 * strafeError;
        double turnPower = 0.02 * errorHeading;

        double frontLeftPower = forwardPower - strafePower - turnPower;
        double frontRightPower = forwardPower + strafePower + turnPower;
        double backLeftPower = forwardPower - strafePower + turnPower;
        double backRightPower = forwardPower + strafePower - turnPower;



        if ((errorX + errorZ < 0.05) && errorHeading < 5) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            return true;
        }
        //do all the power stuff here before returning false

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
        return false;
    }
}
