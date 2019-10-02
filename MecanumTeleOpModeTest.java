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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.sql.Array;


@TeleOp(name="MecanumTeleOpModeTest", group="Linear Opmode")
//@Disabled
public class MecanumTeleOpModeTest extends LinearOpMode {

    //control ctrl = new control();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    /*
    DECLARING OTHER VARIABLES
    */

    // Voltage multipliers for each wheel
    double V[];

    // Variables to store the input of the gamepad
    double left_x, left_y, right_x, angleStick;

    // Variables used for the imu (gyro)
    double robotHeading, deltaHeading, correction, lastAngle = 0;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        control.initialize(hardwareMap);


        /**
        INITIALIZING IMU (GYRO)
         */

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
            CODE FOR CONTROLLING THE DRIVE MOTORS
             */

            // Getting input from gamepad joysticks
            left_x = gamepad1.left_stick_x;
            left_y = gamepad1.left_stick_y;
            right_x = gamepad1.right_stick_x;

            // DESIRED ROBOT SPEED, RANGE: [-1,1]
            double VDes = Math.hypot(left_x, left_y);           // Calculate the distance between the center and the current position of the stick

            // DESIRED ANGLE TO MOVE IN, RANGE: [-π,π]
            // Robot must move relative to the controller, so input from the gyro sensor and start direction of the robot is needed
            // Clockwise rotation make the gyro output decrease
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);  //refreshing the imu
            deltaHeading = control.startDeltaHeading + getHeading();    // Calculate the difference between the heading of the robot and the controller in degrees
            deltaHeading = Math.toRadians(deltaHeading);        // Converting the heading to radians, because that's what the formula needs
            angleStick = -Math.atan2(-left_x, -left_y);         // Calculate the angle in the plane of the left stick in radians
            double ThetaD = angleStick + deltaHeading;          // With those two things calculate the direction in which the robot needs to move at

            // DESIRED SPEED FOR CHANGING DIRECTION: [-1,1]
            double VTheta = right_x;                            // The right stick is used to rotate, so the right stick input can be directly linked to the VTheta

            //if the robot isn't supposed to turn, correct change in heading
            if (VTheta != 0){
                robotHeading = getHeading();
            }
            else {
                correction = robotHeading - getHeading();
            }
            //VTheta += correction;

            V[0] = VDes * Math.sin(ThetaD + Math.PI / 4) + VTheta;
            V[1] = VDes * Math.cos(ThetaD + Math.PI / 4) - VTheta;
            V[2] = VDes * Math.cos(ThetaD + Math.PI / 4) + VTheta;
            V[3] = VDes * Math.sin(ThetaD + Math.PI / 4) - VTheta;


            // Assigning the calculated speed to each wheel
            // LF:1, RF:2, LB:3, RB:4
            control.setPower(V);

            // Show information on the screen of the DS phone
            telemetry.addData("Power motor 1: " , V[0]);
            telemetry.addData("Power motor 2: " , V[1]);
            telemetry.addData("Power motor 3: " , V[2]);
            telemetry.addData("Power motor 4: " , V[3]);
            telemetry.addData("gyro: ", getHeading());
            telemetry.update();
        }

    }

    /**
    GYRO FUNCTIONS
     */

    public double getHeading(){
        // and return the rotation in degrees on the Z-axis of the imu
        return angles.firstAngle;
    }

    public double getDeltaAngle(){
        //get the difference between the last and the current Z-axis angle
        double deltaAngle = angles.firstAngle - lastAngle;
        if (deltaAngle > 180){
            deltaAngle -= 360;
        }
        else if (deltaAngle < 180){
            deltaAngle += 360;
        }
        lastAngle = angles.firstAngle;
        return Math.toRadians(deltaAngle);
    }

}
