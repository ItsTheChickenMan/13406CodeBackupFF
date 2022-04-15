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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous

public class PhoenixAutoRedDuck extends EzrasLaw {
    // SETTINGS
    
    // how long to wait for a pipe recognition
    private double pipeWaitTime = 1.0;
    
    // how long to spin carousel for duck
    private double duckWait = 2.2;
    
    // zoom for tfod
    private double tfodZoom = 1.1;
    
    // each barcode's corresponding hub level
    // indices: 0 = left 1 = mid 2 = right
    // values: 0 = bottom 1 = mid 2 = top
    private int[] hubOrder = {0, 1, 2};
    
    // MISC GLOBALS
    
    // what hub level to place pre-loaded block onto
    // 0 = bottom 1 = mid 2 = top
    private int hubLevel;
    
    public void initAuto(){
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
        
        initVuforia();
        initTfod();
        
        if (tfod != null) {
            tfod.activate();
            
            tfod.setZoom(1.1, 16.0/9.0);
        }
    }

    @Override
    public void runOpMode() {
        this.initAuto();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        // immediately clamp
        this.clamp();
        
        // lift arm
        this.setArmRotation(this.armWaitPosition);
    
        // get position of pipe in frame
        float pipeDetection = waitForPipeRecognition(0.75);
        
        int hubLevel = 0; // 0 = bottom, 1 = middle, 2 = top
        
        // middle
        if(pipeDetection < 200 && pipeDetection > 0){
            hubLevel = 1;
        } else if(pipeDetection > 200){ // right
            hubLevel = 2;
        } else { // right
            // this should be default anyways, but set it just for fun
            hubLevel = 0;
        }
        
        double armPosition = this.armPositions[hubLevel+1];
        
        telemetry.addData("hubLevel", hubLevel + "");
        telemetry.update();
        
        while(!this.isDone() && opModeIsActive());
        
        this.compoundMove(24.0, -26.0, 26.0, 0.0);
        
        // wait until arm is at position, then move swivel
        while(!this.isDone() && opModeIsActive()){
            if(!this.armMotor.isBusy() && !this.swivelMotor.isBusy()){
                this.setSwivelAngle(60.0);
            }
            
            if(this.getSwivelRotation() > this.swivelWaitPosition){
                this.setArmRotation(armPosition);
            }
        }
        
        while(this.swivelMotor.isBusy() && opModeIsActive());
        
        this.unclamp();
        
        this.compoundMove(-24.0, 16.0, 26.0, 0.0);
        
        this.waitForDrive();
        
        this.pivot(-90.0, 26.0);
        
        this.waitForDrive();
        
        this.setArmRotation(this.armWaitPosition);
        
        this.compoundMove(0.0, 18.6, 26.0, 0.0);
        
        this.waitForDrive();
        
        // set carousel power
        double startTime = this.globalTime.seconds();
        this.carousel.setPower(0.65);
        
        while( (this.armMotor.isBusy() ||  this.globalTime.seconds() <= startTime+2.75) && opModeIsActive());
        
        this.carousel.setPower(0.0);
        
        this.setSwivelAngle(this.swivelRestingPosition);
        
        this.compoundMove(-19.0, 6.0, 26.0, 0.0);
        
        while(this.swivelMotor.isBusy() && opModeIsActive());
        
        while(opModeIsActive());
    }
    
    public void waitForDrive(){
        while(!this.isDone() && opModeIsActive());
    }
}
