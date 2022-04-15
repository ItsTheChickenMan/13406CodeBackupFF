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

public class PhoenixAutoBlueDuck extends EzrasLaw {
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
    
    // what hubl evel to place pre-loaded block onto
    // 0 = bottom 1 = mid 2 = top
    private int hubLevel;
    
    public void initAuto(){
        initVals();
        
        this.unclamp();
        
        initVuforia();
        initTfod();
        
        if (tfod != null) {
            tfod.activate();
            
            tfod.setZoom(tfodZoom, 16.0/9.0);
        }
        
        initArm();
        
        setupIMU();
        
        this.clamp();
        
        this.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        // bottom should be out of frame, default to bottom
        this.hubLevel = 0;
    }

    @Override
    public void runOpMode() {
        initAuto();
        
        // settings (the boolean variables are only for default initialization, use the settings array to access them in the program)
        boolean parkInWarehouse = false;
        
        // settings index
        int PARK_IN_WAREHOUSE = 1;
        
        String[] settingsStrings = {"Dummy Setting (Nothing)", "Park In Warehouse?"};
        boolean[] settings = {false, parkInWarehouse};
        int settingIndex = 0;
        
        // bools to make buttons work properly
        /*boolean toggledLastFrame = false;
        boolean switchedLastFrame = false;
        
        while(!isStarted() && !gamepad1.b){
            // inform user of settings
            telemetry.addLine("Press A to toggle selected setting");
            telemetry.addLine("Press left or right bumpers to switch selected setting");
            telemetry.addLine("Press B to stop settings (cannot be modified afterward)");
            telemetry.addLine("");
            telemetry.addData("Selected setting", settingsStrings[settingIndex]);
            telemetry.addLine("");
            telemetry.addLine("Current settings:");
            
            for(int i = 0; i < settingsStrings.length; i++){
                telemetry.addData(settingsStrings[i], settings[i]);
            }
            
            telemetry.update();
            
            if(gamepad1.a && !toggledLastFrame){
                settings[settingIndex] = !settings[settingIndex];
                toggledLastFrame = true;
            } else if(!gamepad1.a){
                toggledLastFrame = false;
            }
            
            if(gamepad1.left_bumper && !switchedLastFrame){
                settingIndex = settingIndex-1 < 0 ? settings.length-1 : settingIndex-1;
                switchedLastFrame = true;
            } else if(gamepad1.right_bumper && !switchedLastFrame){
                settingIndex = settingIndex+1 >= settings.length ? 0 : settingIndex+1;
                switchedLastFrame = true;
            } else if(!gamepad1.left_bumper && !gamepad1.right_bumper){
                switchedLastFrame = false;
            }
        }*/
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        globalTime = new ElapsedTime();
        
        // get position of pipe in frame
        float pipeDetection = waitForPipeRecognition(pipeWaitTime);
        
        // middle
        if(pipeDetection < 200 && pipeDetection > 0){
            this.hubLevel = 1;
        } else if(pipeDetection > 200){ // right
            this.hubLevel = 2;
        } else { // right
            // this should be default anyways, but set it just for fun
            this.hubLevel = 0;
        }
        
        // update step
        telemetry.addData("Got detection, this.hubLevel", this.hubLevel == 0 ? "bottom" : this.hubLevel == 1 ? "middle" : "top");
        telemetry.update();
        
        // move away from wall
        this.strafeDistance(4, 0.5, true);
        
        // move backward into carousel
        double distance = this.moveBackUntilCarousel(duckWait);
        
        // correct for distance travelled for duck
        // +1 inch is to correct for the extra bit of time against the hub
        double duckCorrection = -12 + (distance-1);
        
        telemetry.addData("distance", distance);
        telemetry.addData("correction", duckCorrection);
        telemetry.addData("hub level", this.hubLevel);
        telemetry.update();

        // correct
        this.goForwardDistance(duckCorrection, 0.5, true);
        
        // log
        telemetry.addLine("going to hub");
        telemetry.addData("hub level", this.hubLevel);
        telemetry.update();
        
        // if hubLevel is top, avoid TSE
        if(this.hubLevel == 2){
            this.strafeDistance(14, 0.5, true);
            
            this.goForwardDistance(-4, 0.5, true);
            
            this.strafeDistance(22, 0.5, true);
            
            this.goForwardDistance(4, 0.5, true);
        }
        
        // strafe to hub
        double distanceToStrafe = 60.0;
        
        if(this.hubLevel == 0){
            distanceToStrafe -= 1.0;
        } else if(this.hubLevel == 2){
            distanceToStrafe -= 38;
        }
        
        this.strafeDistance(distanceToStrafe, 0.5, true);
        
        // log
        telemetry.addLine("going to hub");
        telemetry.addLine("moving arm to position");
        telemetry.update();
        
        // move arm to level
        // top 22
        // middle 25
        // bottom 28
        int offset = 25;
        
        if(this.hubLevel == 1){
            offset = 28;    
        } else if(this.hubLevel == 0){
            offset = 29;
        } else if(this.hubLevel == 2){
            offset = 28;
        }
        
        setArmLevel(this.hubLevel+1, offset, 0.2);
        
        // go to hub
        // d = 43
        // arm = 20
        // rest = 23
        this.goForwardDistance(17, 0.5, true);
        
        // wait for arm
        while(this.armMotor.isBusy() && opModeIsActive());
        
        // go rest of distance
        this.goForwardDistance(24, 0.5, true);
        
        // unclamp
        this.unclamp();
        
        // wait a sec
        wait(0.6);
        
        // parking time
        this.goForwardDistance(-28, 0.5, true);
        
        // move arm back
        this.setArmPosition(-200, 0.4); 
        
        // at this point, check if we're set to park in warehouse.
        // if we are, take the warehouse route (strafe all the way into wall and gun it forward)
        // otherwise park in the storage area or whatever it's called
        
        if(settings[PARK_IN_WAREHOUSE]){
            // strafe pretty close
            this.strafeDistance(-48, 0.75, true);
            
            // turn off arm
            this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            // go forward a foot to avoid TSE
            this.goForwardDistance(16, 0.5, true);
            
            // strafe into wall
            this.strafeDistance(-20, 0.75, true);
            
            // gun it
            telemetry.addLine("gunnin it");
            telemetry.update();
            
            this.goForwardDistance(66, 1/this.autoSpeed, true);
        } else {
            double distanceToPark = -37.5;
            
            if(this.hubLevel == 0){
                distanceToPark += 1;    
            } else if(this.hubLevel == 2){
                distanceToPark += 2;
            }
            
            this.strafeDistance(distanceToPark, 0.5, true);
        
            // turn off arm
            this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            // move back a little bit
            this.goForwardDistance(-24, 0.5, true);   
        }
        
        // run until the end of the match (driver presses STOP)
        String message = getRandomFinalMessage();
        while (opModeIsActive()) {
            telemetry.addLine(message);
            telemetry.update();
        }
    }
}
