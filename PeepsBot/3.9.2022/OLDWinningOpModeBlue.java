package org.firstinspires.ftc.teamcode;

import AutoTools.EzrasLaw;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class OLDWinningOpModeBlue extends EzrasLaw {
  // SETTINGS //
  private boolean usingFOD = true;
  private double keyDelay = 0.3;
  private double carouselPower = 0.65;
  private double tuneSpeed = 1;
  private double slowModeSpeed = 0.3;
  
  // MISC //
  public int blueTape;
  public CRServo tape;
  
  private boolean clamped;
  
  private double GAkeyLastPressed;
  private double GBkeyLastPressed;
  private double orientationOffset;  

  private int hubLevel;
  private int tune;
  
  // overriden by winningopmodered
  public void setupTape(){
    this.blueTape = 1;
    
    this.tape = this.tapeBlue;
  }
  
  public void initAuto() {
    /*this.control_Hub = (Blinker)this.hardwareMap.get(Blinker.class, "Control Hub");
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
    */
    
    this.initVals();
    
    this.setupIMU();
    
    this.GAkeyLastPressed = 0.0D;
    this.GBkeyLastPressed = 0.0D;
    this.orientationOffset = 0;
    
    this.clamped = false;
    this.hubLevel = 3;
    this.tune = 0;
    /*if (this.blueTape == 1) {
      this.tape = this.tapeBlue;
    } else {
      this.tape = this.tapeRed;
    }*/
    this.setupTape();
    
    // pidf
    this.initArm();
  }
  
  public void runOpMode() {
    initAuto();
    
    this.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    this.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    
    this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    this.telemetry.addData("Status", "Initialized");
    this.telemetry.update();
    
    waitForStart();
    
    this.globalTime = new ElapsedTime();
    
    while (opModeIsActive()) {
      double movement_x = Range.clip(this.gamepad1.left_stick_x, -1.0, 1.0);
      double movement_y = Range.clip(this.gamepad1.left_stick_y, -1.0, 1.0);
      double movement_turn = Range.clip(this.gamepad1.right_stick_x, -1.0, 1.0);
      
      double orientation = -this.getYRotation() - orientationOffset;
      
      orientation = toRadians(orientation);
      
      if (this.usingFOD) {
        double temp = movement_x * Math.cos(orientation) + movement_y * Math.sin(orientation);
        movement_y = -movement_x * Math.sin(orientation) + movement_y * Math.cos(orientation);
        movement_x = temp;
      } 
      
      double fl = movement_y - movement_turn - movement_x;
      double fr = movement_y + movement_turn + movement_x;
      double bl = movement_y - movement_turn + movement_x;
      double br = movement_y + movement_turn - movement_x;
      
      double relativePower = (1+this.slowModeSpeed) - gamepad1.left_trigger;
      
      setPowers(fl*relativePower, fr*relativePower, bl*relativePower, br*relativePower);
      
      if (this.globalTime.seconds() > this.GAkeyLastPressed + this.keyDelay) {
        
        if (this.gamepad1.a) {
          this.usingFOD = !this.usingFOD;
          this.GAkeyLastPressed = this.globalTime.seconds();
        }
        
        if (this.gamepad1.x) {
          this.tape.setPower(0.0);
          this.GAkeyLastPressed = this.globalTime.seconds();
        }
        
        if (this.gamepad1.y) {
          this.tape.setPower( -(this.blueTape * 2 - 1));
          this.GAkeyLastPressed = this.globalTime.seconds();
        }
        
        if (this.gamepad1.b) {
          this.tape.setPower( (this.blueTape * 2 - 1));
          this.GAkeyLastPressed = this.globalTime.seconds();
        }
      } 
      
      this.carousel.setPower(this.gamepad1.right_trigger * this.carouselPower * (this.blueTape * 2 - 1));
      
      if(gamepad1.right_bumper){
        orientationOffset = -this.getYRotation();
      }
      
      if (this.globalTime.seconds() > this.GBkeyLastPressed + this.keyDelay) {
        if(gamepad2.a){
            if (this.clamped) {
              unclamp();
            } else {
              clamp();
            } 
            
            this.clamped = !this.clamped;
            this.GBkeyLastPressed = this.globalTime.seconds();
        } 
        
        if(gamepad2.b && this.clamped){
            quickDeposit();
            
            this.GBkeyLastPressed = this.globalTime.seconds();
        }
        
        if(gamepad2.x){
          this.hubLevel = 4;
        }
      }
      
      double movement_arm = Range.clip(-this.gamepad2.left_stick_y, -1.0, 1.0);
      double movement_tune = Range.clip(-this.gamepad2.right_stick_y, -1.0, 1.0);
      
      if (movement_arm != 0.0 && !this.armMotor.isBusy()){
        this.hubLevel = (int)(this.hubLevel + Math.signum(movement_arm)); 
      }
      
      if (movement_tune != 0.0){
        this.tune = (int)(this.tune + movement_tune * this.tuneSpeed); 
      }
      
      this.hubLevel = Range.clip(this.hubLevel, 0, 4);
      
      if(this.GBkeyLastPressed != 0){
        setArmLevel(this.hubLevel, this.tune, 0.4);
      }
      
      if(this.hubLevel == 0 && movement_arm == 0){
        this.hubLevel = 1;
      }
      
      this.telemetry.addData("Status", "Running");
      this.telemetry.addData("usingFOD", usingFOD);
      this.telemetry.addData("orientation", toDegrees(orientation));
      this.telemetry.addData("arm at target?", !armMotor.isBusy());
      this.telemetry.addData("hub level", this.hubLevel);
      this.telemetry.addData("tune", this.tune);
      this.telemetry.update();
    } 
  }
}
