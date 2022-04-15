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
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public class BasicDriveTest extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Gyroscope imu;
    
    private ElapsedTime globalTimer;    

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // global ElapsedTime object used for wait()
        globalTimer = new ElapsedTime();
        
        // TEST SETTINGS //
        double testDuration = 1.0; // amount of seconds to perform each test
        double testInterval = 1.0; // amount of seconds to wait between tests
        
        // all tests, defined in order that they are performed
        boolean individualMotorTest = false;
        boolean testAllMotorsSlowly = false;
        boolean testAllMotorsFast = false;
        boolean testStrafeLeft = false;
        boolean testStrafeRight = false;
        boolean testTurnLeft = false;
        boolean testTurnRight = false;
        boolean userDrive = true;
        
        // INIT + MAIN //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // todo: abstract similar test functions into single function
        if(individualMotorTest){
            DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};
            String[] motorNames = {"frontLeft", "frontRight", "backLeft", "backRight"};
            
            for(int i = 0; i < motors.length; i++){
                telemetry.addLine("Testing each individual motor");
                telemetry.addData("Testing", motorNames[i]);
                telemetry.update();
                
                motors[i].setPower(1.0);
                
                wait(testDuration);
                
                motors[i].setPower(0.0);
                
                wait(testInterval);
            }
        }
        
        if(testAllMotorsSlowly){
            telemetry.addLine("Powering all motors slowly (0.1)...");
            telemetry.update();
            
            this.setMotorPowers(0.1);
        
            wait(testDuration);
        
            this.setMotorPowers(0);
        
            wait(testInterval);
        }
        
        if(testAllMotorsFast){
            telemetry.addLine("Powering all motors fast (1.0)...");
            telemetry.update();
            
            this.setMotorPowers(1.0);
            
            wait(testDuration);
            
            this.setMotorPowers(0);
        
            wait(testInterval);
        }
        
        if(testStrafeLeft){
            telemetry.addLine("Strafing left...");
            telemetry.update();
            
            this.strafe(-0.5);
            
            wait(testDuration);
            
            this.setMotorPowers(0);
        
            wait(testInterval);
        }
        
        if(testStrafeRight){
            telemetry.addLine("Strafing right...");
            telemetry.update();
            
            this.strafe(0.5);
            
            wait(testDuration);
            
            this.setMotorPowers(0);
        
            wait(testInterval);
        }
        
        if(testTurnLeft){
            telemetry.addLine("Turning left...");
            telemetry.update();
            
            this.turn(-0.5);
            
            wait(testDuration);
            
            this.setMotorPowers(0);
        
            wait(testInterval);
        }
        
        if(testTurnRight){
            telemetry.addLine("Turning right...");
            telemetry.update();
            
            this.turn(0.5);
            
            wait(testDuration);
            
            this.setMotorPowers(0);
        }
        
        telemetry.addLine("Tests complete.");
        
        if(userDrive){
            telemetry.addLine("Giving user control in 1 second...");
        }
        
        telemetry.update();
        wait(1.0);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            
            if(!userDrive) break;
            
            double movement_x = Range.clip(gamepad1.left_stick_x, -1.0, 1.0);
            movement_x = 0;
            double movement_y = Range.clip(-gamepad1.left_stick_y, -1.0, 1.0);
            double movement_turn = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);
            
            double frontLeftPower = movement_y + movement_x + movement_turn;
            double frontRightPower = movement_y - movement_x - movement_turn;
            double backLeftPower = movement_y - movement_x + movement_turn;
            double backRightPower = movement_y + movement_x - movement_turn;
            
            this.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
    }
    
    public void setMotorPowers(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }
    
    public void setMotorPowers(double fl, double fr, double bl, double br){
        this.frontLeft.setPower(fl);
        this.frontRight.setPower(fr);
        this.backLeft.setPower(bl);
        this.backRight.setPower(br);
    }
    
    public void wait(double seconds){
        double startTime = globalTimer.seconds();
        
        double endTime = startTime + seconds;
        
        while(globalTimer.seconds() < endTime && opModeIsActive());
    }
    
    // move bot forward (direction defined by power: + = forward - = backward)
    public void forward(double power){
        this.setMotorPowers(power);
    }
    
    // strafe bot (direction defined by power: + = right - = left)
    public void strafe(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(-power);
        this.backLeft.setPower(-power);
        this.backRight.setPower(power);
    }
    
    // turn bot (direction defined by power: + = right - = left)
    public void turn(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(-power);
        this.backLeft.setPower(power);
        this.backRight.setPower(-power);
    }
}
