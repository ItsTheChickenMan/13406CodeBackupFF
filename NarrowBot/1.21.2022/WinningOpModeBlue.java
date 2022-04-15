/*
Copyright 2022 FIRST Tech Challenge Team FTC_13406

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

import AutoTools.EzrasLaw;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.WinningOpModeBlue;

@TeleOp
public class WinningOpModeBlue extends EzrasLaw {
    /* SETTINGS */
    public boolean blueTape = true; // if true, using blue tape else using red tape
    private boolean usingFOD = true;
    
    private double keyDelay = 0.3;
    private double carouselPower = 0.7;
    private double tuneSpeed = 0.1;
  
    /* OTHER */
    private CRServo tape;
    
    private boolean clamped;

    private double GAkeyLastPressed;  
    private double GBkeyLastPressed;
  
    private int hubLevel;
    private int tune;
  
    public void initVals() {
        this.control_Hub = (Blinker)this.hardwareMap.get(Blinker.class, "Control Hub");
        this.expansion_Hub_2 = (Blinker)this.hardwareMap.get(Blinker.class, "Expansion Hub 2");
        this.armMotor = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "armMotor");
        this.backLeft = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "backLeft");
        this.backRight = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "backRight");
        this.carousel = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "carousel");
        this.frontLeft = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.frontRight = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, "frontRight");
        this.tapeBlue = (CRServo)this.hardwareMap.get(CRServo.class, "tapeBlue");
        this.tapeRed = (CRServo)this.hardwareMap.get(CRServo.class, "tapeRed");
        this.imu = (BNO055IMU)this.hardwareMap.get(BNO055IMU.class, "imu");
        
        this.GAkeyLastPressed = 0;
        this.GBkeyLastPressed = 0;
        this.clamped = false;
        this.hubLevel = 2;
        this.tune = 0;
        
        if(this.blueTape){
            tape = tapeBlue;
        } else {
            tape = tapeRed;
        }
    }
  
    public void runOpMode() {
        initVals();
        
        this.telemetry.addData("Status", "Initialized");
        this.telemetry.update();
        
        waitForStart();
        
        this.globalTime = new ElapsedTime();
        
        while (opModeIsActive()) {
            this.telemetry.addData("Status", "Running");
            this.telemetry.update();
            
            double movement_x = Range.clip(this.gamepad1.left_stick_x, -1.0, 1.0);
            double movement_y = Range.clip(this.gamepad1.left_stick_y, -1.0, 1.0);
            double movement_turn = Range.clip(this.gamepad1.right_stick_x, -1.0, 1.0);
            
            double orientation = getYRotation();
            
            orientation = toRadians(orientation);
        
            if (this.usingFOD){
                double temp = movement_x * Math.cos(orientation) + movement_y * Math.sin(orientation);
                movement_y = -movement_x * Math.sin(orientation) + movement_y * Math.cos(orientation);
                movement_x = temp;
            }
            
            if(this.globalTime.seconds() > this.GAkeyLastPressed + this.keyDelay){
                if(this.gamepad1.a){
                    this.usingFOD = !this.usingFOD;
                    this.GAkeyLastPressed = this.globalTime.seconds();
                } 
        
                if (this.gamepad1.x){
                    this.GAkeyLastPressed = this.globalTime.seconds(); 
                }
                
                if (this.gamepad1.y){
                    this.GAkeyLastPressed = this.globalTime.seconds(); 
                }
                
                if (this.gamepad1.b){
                    this.GAkeyLastPressed = this.globalTime.seconds(); 
                }
            } 
      
            this.carousel.setPower(this.gamepad1.right_trigger * this.carouselPower);
            
            // gamepad2 key controls
            if (this.globalTime.seconds() > this.GBkeyLastPressed + this.keyDelay){
                if(this.gamepad2.a){
                    if(this.clamped){
                        unclamp();
                    } else {
                        clamp();
                    } 
                    
                    this.clamped = !this.clamped;
                    this.GBkeyLastPressed = this.globalTime.seconds();
                }  
            }
            
            // gamepad2 continuous controls
            double movement_arm = Range.clip(this.gamepad2.left_stick_y, -1.0, 1.0);
            double movement_tune = Range.clip(this.gamepad2.right_stick_y, -1.0, 1.0);
          
            if(movement_arm != 0.0 && !this.armMotor.isBusy()){
                this.hubLevel = (int)(this.hubLevel + Math.signum(movement_arm)); 
            }
      
            if(movement_tune != 0.0){
                this.tune = (int)(this.tune + movement_arm * this.tuneSpeed); 
            }
      
            this.hubLevel = Range.clip(this.hubLevel, -1, 3);
            setArmLevel(this.hubLevel + 1, this.tune);
        } 
  }
}