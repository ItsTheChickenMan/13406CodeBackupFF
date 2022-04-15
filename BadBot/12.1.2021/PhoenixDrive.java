/*
Copyright 2021 FIRST Tech Challenge Team FTC

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class PhoenixDrive extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Gyroscope imu;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        
        frontLeft.setDirection(DcMotor.Direction.FORWARD); // reverse
        backLeft.setDirection(DcMotor.Direction.REVERSE); // reverse
        frontRight.setDirection(DcMotor.Direction.REVERSE); // reverse
        backRight.setDirection(DcMotor.Direction.FORWARD); // forward
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            // testing color sensor
            telemetry.addData("Light", colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue());
            telemetry.update();
            
            // personal note for Phoenix: get input with gamepad1 .left_stick .right_stick
                // power motors through motor.setPower(/*-1 to 1*/);
            
            double movement_x = Range.clip(gamepad1.left_stick_x, -1.0, 1.0);
            double movement_y = Range.clip(gamepad1.left_stick_y, -1.0, 1.0);
            double movement_turn = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);
            
            // when left trigger is full pressed power is reduced to 0.3
            double relativePower = 1.3-gamepad1.left_trigger;
            
            double frontLeftPower = movement_y - movement_turn - movement_x;
            double frontRightPower = movement_y + movement_turn + movement_x;
            double backLeftPower = -movement_y + movement_turn - movement_x;
            double backRightPower = -movement_y - movement_turn + movement_x;
            
            this.setMotorPowers(frontLeftPower*relativePower, frontRightPower*relativePower, backLeftPower*relativePower, backRightPower*relativePower);
        }
    }
    
    // set all motor powers individually
    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        this.frontLeft.setPower(frontLeftPower);
        this.frontRight.setPower(frontRightPower);
        this.backLeft.setPower(backLeftPower);
        this.backRight.setPower(backRightPower);
    }
    
    // set all motor powers to one value
    public void setMotorPowers(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }
    
    // set all motor modes to one mode
    public void setMotorMode(DcMotor.RunMode mode){
        this.frontLeft.setMode(mode);
        this.frontRight.setMode(mode);
        this.backLeft.setMode(mode);
        this.backRight.setMode(mode);
    }
}
