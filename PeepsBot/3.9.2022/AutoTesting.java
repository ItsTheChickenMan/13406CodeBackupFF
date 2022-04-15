/*
Copyright 2022 FIRST Tech Challenge Team FTC

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

public class AutoTesting extends EzrasLaw {
    public void initAuto(){
        initVals();
        
        initArm();
        
        setupIMU();
        
        this.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        double inchTolerance = 1.0/16.0;
        double tickTolerance = (inchTolerance / this.wheelCircumference) * this.tpr;
        
        this.frontLeft.setTargetPositionTolerance((int)tickTolerance);
        this.frontRight.setTargetPositionTolerance((int)tickTolerance);
        this.backLeft.setTargetPositionTolerance((int)tickTolerance);
        this.backRight.setTargetPositionTolerance((int)tickTolerance);
        
        this.globalTime = new ElapsedTime();
    }
    
    @Override
    public void runOpMode() {
        initAuto();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        while(opModeIsActive()){
            this.updateStates();
            
            if(gamepad2.a){
                this.centerIntake();
                this.intakePower = 0;
            } else {
                this.intakePower = gamepad2.right_trigger;
            }
            
            this.checkSettings();
        }
        
        /*this.swivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        this.swivelMotor.setTargetPosition(1000);
        
        this.swivelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        this.swivelMotor.setPower(0.5);
    
        while(opModeIsActive()){
            telemetry.addData("currentPosition", this.swivelMotor.getCurrentPosition());
            telemetry.update();
            
            if(gamepad1.a){
                this.swivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.swivelMotor.setPower(0);
            }    
        }*/
        
        /*this.compoundMoveNoVelocity(24.0, 0.0);
        
        double currentTime = globalTime.seconds();
        double previousTime = currentTime;
        double delta = 0;
        
        double[] avg = {Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE};
        
        while( opModeIsActive() ){
            currentTime = globalTime.seconds();
            delta = currentTime - previousTime;
            previousTime = currentTime;
            
            if(!this.isDone()){
                this.trapezoidVelocityCycle(24.0, 128.0, delta);
                
                for(int i = 0; i < avg.length; i++){
                    if(avg[i] == Integer.MAX_VALUE){
                        avg[i] = this.driveMotors[i].getVelocity();
                    } else {
                        avg[i] += this.driveMotors[i].getVelocity();
                        
                        avg[i] /= 2;
                    }
                }
            }
            
            for(int i = 0; i < avg.length; i++){
                telemetry.addData(i + "", avg[i]);
            }
            
            telemetry.update();
        };*/
        
        // initially move back into warehouse
        /*this.compoundMove(24.0, 0.0, 24.0, 128.0);
        
        while(opModeIsActive()){
            // move back slowly waiting for freight
            double startTime = globalTime.seconds();
            
            double moveBackVelocity = this.inchesToTicks(6.0);
            
            this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            this.setVelocity(moveBackVelocity);
            
            this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            // run cycle state in state 0 (looking for freight)
            while(this.cycleState == 0 && opModeIsActive()){
                this.intakePower = 1; // run intake
                
                this.updateStates();
                
                this.checkCycleStateBlue(); // run for blue bc delivering to red alliance
            
                this.checkSettings();
            }
            
            // freight acquired, move out
            double distanceTravelled = this.ticksToInches( (globalTime.seconds() - startTime) * moveBackVelocity );
            double distanceToTravel = -(distanceTravelled + 44);
            
            // travel to hub, check cycle state along the way up until deposit
            this.compoundMoveNoVelocity(distanceToTravel, 8.0);
            
            // delta stuff
            double delta = 0;
            double currentTime = globalTime.seconds();
            startTime = currentTime;
            
            while(!this.isDone() && opModeIsActive()){
                currentTime = globalTime.seconds();
                delta = currentTime - startTime;
                startTime = currentTime;
                
                this.updateStates();
                
                this.checkCycleStateBlue(); // move arm into deposit position
                
                // if cycle state is 3, instruct arm to go to top level
                if(this.cycleState == 2 && Math.abs(this.swivelRotation) > this.swivelReturningPosition){
                    this.desiredArmPosition = this.armUpPosition;
                }
                
                this.checkSettings();
                
                // run trapezoid velocity
                this.trapezoidVelocityCycle(48.0, 128.0, delta);
            }
            
            // make sure arm isn't still moving
            while(this.armMotor.isBusy() && opModeIsActive());
            
            // deposit
            this.cycleState++;
            this.clamped = false;
            
            this.updateStates();
            
            this.checkCycleStateBlue();
            
            this.checkSettings();
            
            // go back to warehouse + move arm back to position
            // TODO: strafe + forward as setting?
            this.compoundMoveNoVelocity(20.0, -8.0);
            
            while(!isDone() && opModeIsActive()){
                currentTime = globalTime.seconds();
                delta = currentTime - startTime;
                startTime = currentTime;
                
                // check cycle state (move arm back to position)
                if(this.cycleState != 0){
                    this.updateStates();
                
                    this.checkCycleStateBlue();
                
                    this.checkSettings();
                }
                
                // trapezoid velocity
                this.trapezoidVelocityCycle(48.0, 128.0, delta);
            }
            
            // continue moving back into warehouse + updating cycle state
            this.compoundMoveNoVelocity(24.0, 0.0);
            
            while(!isDone() && opModeIsActive()){
                currentTime = globalTime.seconds();
                delta = currentTime - startTime;
                startTime = currentTime;
                
                // check cycle state (move arm back to position)
                // TODO: wrap these three into cycle function, for auto?
                if(this.cycleState != 0){
                    this.updateStates();
                
                    this.checkCycleStateBlue();
                
                    this.checkSettings();
                }
                
                // trapezoid velocity
                this.trapezoidVelocityCycle(48.0, 128.0, delta);
            }
            
            // make sure cycle is complete and then restart
            while(this.cycleState != 0 && opModeIsActive()){
                // check cycle state (move arm back to position)
                // TODO: wrap these three into cycle function, for auto?
                this.updateStates();
                
                this.checkCycleStateBlue();
                
                this.checkSettings();
            }
        }*/
        
        // run wheels for a couple frames
        /*this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        this.setVelocity(this.inchesToTicks(24.0));
        
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        int frameCount = 0;
        int frames = 90;
        
        DcMotorEx[] driveMotors = {this.frontLeft, this.frontRight, this.backLeft, this.backRight};
        
        double avg[] = new double[4];
        
        for(int i = 0; i < driveMotors.length; i++){
            avg[i] = driveMotors[i].getVelocity();
        }
        
        while(frameCount < frames && opModeIsActive()){
            for(int i = 0; i < driveMotors.length; i++){
                avg[i] += driveMotors[i].getVelocity();
                
                avg[i] /= 2.0;
            }
            
            frameCount++;
        }
        
        this.setVelocity(0);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            for(int i = 0; i < avg.length; i++){
                telemetry.addData(i + "", this.ticksToInches(avg[i]));    
            }
            
            telemetry.update();
        };
    }*/
    }
}
