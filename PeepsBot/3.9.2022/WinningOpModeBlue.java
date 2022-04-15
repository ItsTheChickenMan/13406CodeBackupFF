/*
Copyright 2022 FIRST Tech Challenge Team 13406

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import AutoTools.EzrasLaw;

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

/**
 * OpMode logic is meant to be simple and standard.  The process is as follows:
 * 
 * 1. Update states (indicators of the robot's state, such as swivelPosition) to their proper values
 * 2. Run through the main loop, which uses the robot state to change settings
 * 3. React according to the current settings
*/
public class WinningOpModeBlue extends EzrasLaw {
    public int mode = 0;
    public double carouselDirection = -1;
    
    public double GALastPressed = 0;
    public double GBLastPressed = 0;
    public double GPWaitTime = 0.3;
    
    public void initCarousel(){
        this.carouselDirection = -1;    
    }
    
    public void checkCycle(){
        this.checkCycleState();    
    }
    
    // initialize opmode
    public void initOpMode(){
        this.initVals();
        
        this.setupIMU();
        
        this.initArm();
        
        this.initializeSwivel();
        
        this.globalTime = new ElapsedTime();
        
        // setup motor directions
        this.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void runOpMode() {
        this.initOpMode();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            this.logStates();
            telemetry.addData("distance reading", this.freightSensor.getDistance(DistanceUnit.MM));
            telemetry.addLine(this.mode == 0 ? "Automatic mode" : "Manual Mode");
            telemetry.update();
            
            // update states
            this.updateStates();
            
            // drive + gamepad1 logic
            double movement_x = Range.clip(gamepad1.left_stick_x, -1, 1);
            double movement_y = Range.clip(gamepad1.left_stick_y, -1, 1);
            double movement_turn = Range.clip(gamepad1.right_stick_x, -1, 1);
            
            double orientation = -this.getYRotation();
            
            orientation = this.toRadians(orientation);
            orientation = 0;
          
            double temp = movement_x * Math.cos(orientation) + movement_y * Math.sin(orientation);
            movement_y = -movement_x * Math.sin(orientation) + movement_y * Math.cos(orientation);
            movement_x = temp;
          
            double fl = movement_y - movement_turn - movement_x;
            double fr = movement_y + movement_turn + movement_x;
            double bl = movement_y - movement_turn + movement_x;
            double br = movement_y + movement_turn - movement_x;
          
            double relativePower = (1.3) - gamepad1.left_trigger;
          
            this.setPowers(fl*relativePower, fr*relativePower, bl*relativePower, br*relativePower);
            //this.setPower(0);
            
            // check intake 
            this.intakePower = gamepad2.right_trigger;
            
            if(gamepad1.right_bumper){
                this.carousel.setPower(0.65 * this.carouselDirection);    
            } else {
                this.carousel.setPower(0);
            }
            
            // run logic
            if(this.mode == 0){
                if(gamepad2.y && this.globalTime.seconds() >= this.GBLastPressed+this.GPWaitTime){
                    this.mode = 1;
                    this.GBLastPressed = this.globalTime.seconds();
                } else if(gamepad2.x && this.globalTime.seconds() >= this.GBLastPressed+this.GPWaitTime){
                    this.cycleState = 6;
                    this.GBLastPressed = this.globalTime.seconds();
                }
                
                this.checkCycle();
            } else {
                if(gamepad2.y && this.globalTime.seconds() >= this.GBLastPressed+this.GPWaitTime){
                    this.mode = 0;
                    this.GBLastPressed = this.globalTime.seconds();
                }
                
                if(gamepad2.a && this.globalTime.seconds() >= this.GBLastPressed+this.GPWaitTime){
                    if(!this.clamped){
                        this.clamped = true;
                    } else {
                        this.clamped = false;
                    }
                    
                    this.GBLastPressed = this.globalTime.seconds();
                }
                
                // check for arm input
                if(this.swivelRotation > this.swivelOutPosition || this.swivelRotation < -this.swivelOutPosition){
                    if(gamepad2.x){
                        this.desiredArmPosition = this.armCapPosition;
                    } else if(gamepad2.left_stick_y != 0 && !this.isArmBusy() ){
                      // if at arm wait, go to armUp
                      if(-gamepad2.left_stick_y > 0){
                        this.desiredArmPosition = this.armUpPosition;
                      // if arm at up, go to armWait
                      } else if(-gamepad2.left_stick_y < 0){
                        this.desiredArmPosition = this.armDownPosition;
                      }
                    }
                    
                    if(gamepad2.right_stick_y != 0){
                      this.desiredArmPosition += -gamepad2.right_stick_y;  
                    }
                }
                
                // check for swivel input
                if(gamepad2.left_stick_x != 0){
                  this.desiredSwivelRotation += -gamepad2.left_stick_x*2;
                }
            }
            
            // check settings
            this.checkSettings();
        }
    }
}
