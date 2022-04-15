/*
Copyright 2021 FIRST Tech Challenge Team FTC_13406

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import AutoTools.Drive;
import AutoTools.Action;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class DiscoDrive extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private DcMotor armMotor;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotor carousel;
    private Servo clawServoLeft;
    private Servo clawServoRight;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private BNO055IMU imu;
    
    private Drive drive;
    
    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        clawServoLeft = hardwareMap.get(Servo.class, "clawServoLeft");
        clawServoRight = hardwareMap.get(Servo.class, "clawServoRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        
        this.setupIMU();
        
        drive = new Drive(this, imu, frontLeft, frontRight, backLeft, backRight);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // move arm out of way
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(-400);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        /*Action move = new Action(1.0, 0, 0);
        move.lockAxes();
            
        drive.stop();
        drive.performAction(move);
        
        while(opModeIsActive());
        */
        
        double startingOrientation = drive.getYRotation();
        // testing
        //startingOrientation = 0;
        
        armMotor.setTargetPosition(-400);
        armMotor.setPower(0.75);
        
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            
            double movement_x = Range.clip(gamepad1.left_stick_x, -1, 1);
            double movement_y = Range.clip(-gamepad1.left_stick_y, -1, 1);
            double movement_turn = -Range.clip(gamepad1.right_stick_x, -1, 1);
            
            double orientation = drive.getYRotation() - startingOrientation;
            double toRadians = Math.PI / 180;
            
            orientation *= toRadians;
            double temp = movement_x*Math.cos(orientation) + movement_y*Math.sin(orientation);
            movement_y = -movement_x*Math.sin(orientation) + movement_y*Math.cos(orientation);
            movement_x = temp;
            
            
            double fl = movement_y + movement_turn + movement_x;
            double fr = movement_y - movement_turn - movement_x;
            double bl = movement_y + movement_turn - movement_x;
            double br = movement_y - movement_turn + movement_x;
            
            /*
            fl *= Math.cos( (orientation-45) * toRadians );
            fr *= Math.cos( (orientation+45) * toRadians );
            bl *= Math.cos( (orientation+45) * toRadians );
            br *= Math.cos( (orientation-45) * toRadians );
            
            fl -= movement_turn;
            fr += movement_turn;
            bl -= movement_turn;
            br += movement_turn;
            */
            
            drive.setPowers(fl, fr, bl, br);
        }
    }
    
    public void setupIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
}
