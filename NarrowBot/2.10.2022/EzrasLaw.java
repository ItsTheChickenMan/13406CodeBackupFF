package AutoTools;

import android.graphics.Bitmap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Frame;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class EzrasLaw extends LinearOpMode {
  public Blinker control_Hub;
  
  public Blinker expansion_Hub_2;
  
  public HardwareDevice webcam_1;
  
  public DcMotorEx armMotor;
  
  public DcMotorEx backLeft;
  
  public DcMotorEx backRight;
  
  public DcMotorEx carousel;
  
  public Servo clawServoLeft;
  
  public Servo clawServoRight;
  
  public DcMotorEx frontLeft;
  
  public DcMotorEx frontRight;
  
  private BNO055IMU imu;
  
  public CRServo tapeBlue;
  
  public CRServo tapeRed;
  
  public static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/FreightFrenzy_PIPE.tflite";
  
  public static final String[] LABELS = new String[] { "Pipe" };
  
  public static final String VUFORIA_KEY = "AcnDHVv/////AAABmRnrh4YWCUq9oXJiDJfQ4tpGcyqNmoiVhQEAVeBCc1pxc59UM914g/AfIlzXvB+DEj61Op1AD7CdC/shuEqyF7E9jn4nYrzRhWXBB2ZZOISZ7dsulJ3ZxGsbSF6xpAuvnGoVTZL2wZGVzFvHmz7re2Ek7IH7Rc59kzFI2mkvtUxHPU4hqR14fXBNvHG5bdVoidyZVm50h5nyDdbnOij/2sJXYh01NzH02y2O1voPzA2XT6sEKAgqi9j8UQ6q4XP/wSa1BLPwwcInLnEJohQSOED4GGWW+M9WHZ6t2zUkSge+YPzBINQwczICkTG6qfDwlRKb7ghKYjvScxr2quuUnIGZntALNrHSlP5OQlBW7QHf";
  
  public VuforiaLocalizer vuforia;
  
  public TFObjectDetector tfod;
  
  public ElapsedTime globalTime;
  
  public double gearRatio = 30.0/14.0;
  
  public double tpr = 384.5 * this.gearRatio;
  
  public double wheelDiameter = 4.0;
  
  public double wheelCircumference = this.wheelDiameter * Math.PI;
  
  public double strafeMultiplier = 1.045;
  
  public double autoSpeed = 40.0D;
  
  public double blockingPauseTime = 0.05;
  
  public double startingOrientation = 0.0D;
  
  public double intendedRotation = 0.0D;
  
  public double x;
  
  public double y;
  
  public String[] finalMessages = new String[] { "mission accomplished", "payload delivered", "job well done", "pheonix is a genius", "tyler is better than saeid", "the cake is a lie", "the Baked Ziti King reigns supreme"};
  
  public int[] armPositions = new int[] {-975, -918, -815, -714, -550};
  
  public int armOffset = 0;
  
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
    this.clawServoLeft = (Servo)this.hardwareMap.get(Servo.class, "clawServoLeft");
    this.clawServoRight = (Servo)this.hardwareMap.get(Servo.class, "clawServoRight");
    this.imu = (BNO055IMU)this.hardwareMap.get(BNO055IMU.class, "imu");
  }
  
  public void initArm() {
    this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    PIDFCoefficients pidf = this.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    this.telemetry.addLine(pidf.toString());
    this.telemetry.update();
    PIDFCoefficients newPidf = new PIDFCoefficients(5.0, pidf.i, pidf.d, pidf.f, MotorControlAlgorithm.LegacyPID);
    try {
      this.armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPidf);
    } catch (UnsupportedOperationException e) {
      this.telemetry.addLine(e.toString());
      this.telemetry.update();
      
      while(opModeIsActive());
    } 
    for (int i = 0; i < this.armPositions.length; i++)
      this.armPositions[i] = this.armPositions[i] + this.armOffset; 
  }
  
  public void initArm(int offset) {
    for (int i = 0; i < this.armPositions.length; i++)
      this.armPositions[i] = this.armPositions[i] + offset; 
  }
  
  public void initArm(int offset, boolean addToOffset) {
    if (addToOffset) {
      for (int i = 0; i < this.armPositions.length; i++)
        this.armPositions[i] = this.armPositions[i] + this.armOffset + offset; 
    } else {
      for (int i = 0; i < this.armPositions.length; i++)
        this.armPositions[i] = this.armPositions[i] + offset; 
    } 
  }
  
  public void setPower(double power) {
    this.frontLeft.setPower(power);
    this.frontRight.setPower(power);
    this.backLeft.setPower(power);
    this.backRight.setPower(power);
  }
  
  public void setPowers(double fl, double fr, double bl, double br) {
    this.frontLeft.setPower(fl);
    this.frontRight.setPower(fr);
    this.backLeft.setPower(bl);
    this.backRight.setPower(br);
  }
  
  public void forward(double power) {
    this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    setPower(power);
  }
  
  public void strafe(double power) {
    this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    setPowers(power, -power, -power, power);
  }
  
  public void turn(double power) {
    this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    setPowers(power, -power, power, -power);
  }
  
  public void stopMotors() {
    setPower(0.0D);
  }
  
  public void initVuforia() {
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    parameters.vuforiaLicenseKey = "AcnDHVv/////AAABmRnrh4YWCUq9oXJiDJfQ4tpGcyqNmoiVhQEAVeBCc1pxc59UM914g/AfIlzXvB+DEj61Op1AD7CdC/shuEqyF7E9jn4nYrzRhWXBB2ZZOISZ7dsulJ3ZxGsbSF6xpAuvnGoVTZL2wZGVzFvHmz7re2Ek7IH7Rc59kzFI2mkvtUxHPU4hqR14fXBNvHG5bdVoidyZVm50h5nyDdbnOij/2sJXYh01NzH02y2O1voPzA2XT6sEKAgqi9j8UQ6q4XP/wSa1BLPwwcInLnEJohQSOED4GGWW+M9WHZ6t2zUkSge+YPzBINQwczICkTG6qfDwlRKb7ghKYjvScxr2quuUnIGZntALNrHSlP5OQlBW7QHf";
    parameters.cameraName = (CameraName)this.hardwareMap.get(WebcamName.class, "Webcam 1");
    this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
  }
  
  public void initTfod() {
    int tfodMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", this.hardwareMap.appContext
        .getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfodParameters.minResultConfidence = 0.8F;
    tfodParameters.isModelTensorFlow2 = true;
    tfodParameters.inputSize = 320;
    this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.vuforia);
    this.tfod.loadModelFromFile("/sdcard/FIRST/tflitemodels/FreightFrenzy_PIPE.tflite", LABELS);
  }
  
  public void quickDeposit(){
    this.clawServoLeft.setPosition(0.26D);  
    this.clawServoRight.setPosition(0.56D);
    
    wait(0.2);
    
    this.clamp();
  }
  
  public void unclamp() {
    this.clawServoLeft.setPosition(0.0D);
    this.clawServoRight.setPosition(0.83D);
  }
  
  public void clamp() {
    this.clawServoLeft.setPosition(0.435D);
    this.clawServoRight.setPosition(0.365D);
  }
  
  public void goForwardSpeed(double velocity) {
    this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    double revolutions = velocity / this.wheelCircumference;
    double ticks = revolutions * this.tpr;
    this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    this.backLeft.setVelocity(ticks);
    this.backRight.setVelocity(ticks);
    this.frontLeft.setVelocity(ticks);
    this.frontRight.setVelocity(ticks);
  }
  
  public void strafeSpeed(double velocity){
    this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    double ticks = velocityToTicks(velocity);
    
    this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    this.backLeft.setVelocity(-ticks);
    this.backRight.setVelocity(ticks);
    this.frontLeft.setVelocity(ticks);
    this.frontRight.setVelocity(-ticks);
  }
  
  public double ticksToVelocity(double ticks){
    double revolutions = ticks / this.tpr;
    double velocity = revolutions * this.wheelCircumference;
    
    return velocity;
  }
  
  public double velocityToTicks(double velocity){
    double revolutions = velocity / this.wheelCircumference;
    double ticks = revolutions * this.tpr;
    
    return ticks;
  }
  
  // set each motor's velocity in inches per second
  public void setMotorsVelocity(double ipr){
    // get ticks 
    double tickSpeed = velocityToTicks(ipr);
    
    // set each motor's velocity to tickspeed
    this.backLeft.setVelocity(tickSpeed);
    this.backRight.setVelocity(tickSpeed);
    this.frontLeft.setVelocity(tickSpeed);
    this.frontRight.setVelocity(tickSpeed);
  }
  
  public void goForwardDistance(double distance, double speed) {
    speed *= this.autoSpeed;
    double revolutions = distance / this.wheelCircumference;
    double ticks = revolutions * this.tpr;
    this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backLeft.setTargetPosition((int)ticks);
    this.backRight.setTargetPosition((int)ticks);
    this.frontLeft.setTargetPosition((int)ticks);
    this.frontRight.setTargetPosition((int)ticks);
    this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.setMotorsVelocity(speed);
    /*this.backRight.setPower(speed);
    this.frontLeft.setPower(speed);
    this.frontRight.setPower(speed);*/
    this.x += Math.sin(toRadians(-this.intendedRotation)) * distance;
    this.y += Math.cos(toRadians(-this.intendedRotation)) * distance;
    //this.telemetry.addData("x", Double.valueOf(this.x));
    //this.telemetry.addData("y", Double.valueOf(this.y));
    //this.telemetry.addData("orientation", Double.valueOf(-this.intendedRotation));
    //this.telemetry.update();
  }
  
  public void goForwardDistance(double distance, double speed, boolean blocking) {
    speed *= this.autoSpeed;
    double startingOrientation = getYRotation();
    double revolutions = distance / this.wheelCircumference;
    double ticks = revolutions * this.tpr;
    this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backLeft.setTargetPosition((int)ticks);
    this.backRight.setTargetPosition((int)ticks);
    this.frontLeft.setTargetPosition((int)ticks);
    this.frontRight.setTargetPosition((int)ticks);
    this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    /*this.backLeft.setPower(speed);
    this.backRight.setPower(speed);
    this.frontLeft.setPower(speed);
    this.frontRight.setPower(speed);*/
    this.setMotorsVelocity(speed);
    if (blocking)
      while (opModeIsActive() && this.backLeft.isBusy() && this.backRight.isBusy() && this.frontLeft.isBusy() && this.frontRight.isBusy()); 
      wait(blockingPauseTime);
      
    double rotation = -this.intendedRotation;
    if (Math.abs(getYRotation() - this.intendedRotation) > 3.5D)
      rotation = -getYRotation(); 
    this.x += Math.sin(toRadians(rotation)) * distance;
    this.y += Math.cos(toRadians(rotation)) * distance;
    //this.telemetry.addData("x", Double.valueOf(this.x));
    //this.telemetry.addData("y", Double.valueOf(this.y));
    //this.telemetry.addData("orientation used", Double.valueOf(rotation));
    //this.telemetry.update();
  }
  
  public void strafeDistance(double distance, double speed) {
    //distance = -distance;
    speed *= this.autoSpeed;
    double revolutions = distance / this.wheelCircumference;
    double ticks = revolutions * this.tpr;
    ticks *= this.strafeMultiplier;
    this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backLeft.setTargetPosition(-((int)ticks));
    this.backRight.setTargetPosition((int)ticks);
    this.frontLeft.setTargetPosition((int)ticks);
    this.frontRight.setTargetPosition(-((int)ticks));
    this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    /*this.backLeft.setPower(speed);
    this.backRight.setPower(speed);
    this.frontLeft.setPower(speed);
    this.frontRight.setPower(speed);*/
    this.setMotorsVelocity(speed);
    this.x += Math.cos(toRadians(-this.intendedRotation)) * distance;
    this.y += Math.sin(toRadians(-this.intendedRotation)) * distance;
    this.telemetry.addData("x", Double.valueOf(this.x));
    this.telemetry.addData("y", Double.valueOf(this.y));
    this.telemetry.addData("orientation", Double.valueOf(-this.intendedRotation));
    this.telemetry.update();
  }
  
  public void strafeDistance(double distance, double speed, boolean blocking) {
    speed *= this.autoSpeed;
    double startingOrientation = getYRotation();
    double revolutions = distance / this.wheelCircumference;
    double ticks = revolutions * this.tpr;
    ticks *= this.strafeMultiplier;
    this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backLeft.setTargetPosition(-((int)ticks));
    this.backRight.setTargetPosition((int)ticks);
    this.frontLeft.setTargetPosition((int)ticks);
    this.frontRight.setTargetPosition(-((int)ticks));
    this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    /*this.backLeft.setPower(speed);
    this.backRight.setPower(speed);
    this.frontLeft.setPower(speed);
    this.frontRight.setPower(speed);*/
    this.setMotorsVelocity(speed);
    if (blocking)
      while (opModeIsActive() && this.backLeft.isBusy() && this.backRight.isBusy() && this.frontLeft.isBusy() && this.frontRight.isBusy()); 
      wait(blockingPauseTime);
    double rotation = -this.intendedRotation;
    if (Math.abs(getYRotation() - this.intendedRotation) > 3.5D)
      rotation = -getYRotation(); 
    this.x += Math.cos(toRadians(rotation)) * distance;
    this.y += Math.sin(toRadians(rotation)) * distance;
    //this.telemetry.addData("x", Double.valueOf(this.x));
    //this.telemetry.addData("y", Double.valueOf(this.y));
    //this.telemetry.addData("orientation used", Double.valueOf(rotation));
    //this.telemetry.update();
  }
  
  public void pivotAngle(double angle, double speed) {
    setupIMU();
    double startingOrientation = getYRotation();
    double finalOrientation = startingOrientation + angle;
    if (finalOrientation > 180.0D || finalOrientation < -180.0D)
      finalOrientation = 360.0D - finalOrientation; 
    this.intendedRotation = finalOrientation + this.startingOrientation;
    double extraSpeed = 0.15D;
    this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    boolean flipped = false;
    double lastOrientation = getYRotation();
    if (finalOrientation < startingOrientation) {
      /*this.backLeft.setPower(speed);
      this.backRight.setPower(-speed);
      this.frontLeft.setPower(speed);
      this.frontRight.setPower(-speed);*/
      while (getYRotation() > finalOrientation && !flipped && opModeIsActive()) {
        double speedFactor = 1.0D - Math.abs((getYRotation() - startingOrientation) / finalOrientation);
        double rot = getYRotation();
        if (rot - lastOrientation > 300.0D || rot - lastOrientation < -300.0D) {
          flipped = true;
          break;
        } 
        lastOrientation = rot;
        speedFactor += extraSpeed;
        this.backLeft.setPower(-speed * speedFactor);
        this.backRight.setPower(speed * speedFactor);
        this.frontLeft.setPower(-speed * speedFactor);
        this.frontRight.setPower(speed * speedFactor);
        //this.telemetry.addData("this.getYRotation() - lastOrientation", Double.valueOf(getYRotation() - lastOrientation));
        //this.telemetry.update();
      } 
    } else {
      /*this.backLeft.setPower(-speed);
      this.backRight.setPower(speed);
      this.frontLeft.setPower(-speed);
      this.frontRight.setPower(speed);*/
      while (getYRotation() < finalOrientation && !flipped && opModeIsActive()) {
        double speedFactor = 1.0D - Math.abs((getYRotation() - startingOrientation) / finalOrientation);
        double rot = getYRotation();
        if (rot - lastOrientation > 300.0D || rot - lastOrientation < -300.0D) {
          flipped = true;
          break;
        } 
        lastOrientation = rot;
        speedFactor += extraSpeed;
        this.backLeft.setPower(speed * speedFactor);
        this.backRight.setPower(-speed * speedFactor);
        this.frontLeft.setPower(speed * speedFactor);
        this.frontRight.setPower(-speed * speedFactor);
        telemetry.addData("this.getYRotation()", this.getYRotation());
        telemetry.addData("finalOrientation", finalOrientation);
        telemetry.addData("getYRotation() >= finalOrientation ? ", getYRotation() >= finalOrientation);
        this.telemetry.addData("this.getYRotation() - lastOrientation", Double.valueOf(getYRotation() - lastOrientation));
        this.telemetry.update();
      } 
    } 
    this.backLeft.setPower(0.0D);
    this.backRight.setPower(0.0D);
    this.frontLeft.setPower(0.0D);
    this.frontRight.setPower(0.0D);
  }
  
  public void pivotToAngle(double angle, double speed) {
    double startingOrientation = getYRotation();
    double finalOrientation = angle;
    if (finalOrientation > 180.0D || finalOrientation < -180.0D)
      finalOrientation = 360.0D - finalOrientation; 
    this.intendedRotation = finalOrientation + this.startingOrientation;
    double extraSpeed = 0.15D;
    this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    boolean flipped = false;
    double lastOrientation = getYRotation();
    if (finalOrientation < startingOrientation) {
      /*this.backLeft.setPower(speed);
      this.backRight.setPower(-speed);
      this.frontLeft.setPower(speed);
      this.frontRight.setPower(-speed);*/
      while (getYRotation() > finalOrientation && !flipped && opModeIsActive()) {
        double speedFactor = 1.0D - Math.abs((getYRotation() - startingOrientation) / finalOrientation);
        double rot = getYRotation();
        if (rot - lastOrientation > 300.0D || rot - lastOrientation < -300.0D) {
          flipped = true;
          break;
        } 
        lastOrientation = rot;
        speedFactor += extraSpeed;
        this.backLeft.setPower(-speed * speedFactor);
        this.backRight.setPower(speed * speedFactor);
        this.frontLeft.setPower(-speed * speedFactor);
        this.frontRight.setPower(speed * speedFactor);
        //this.telemetry.addData("this.getYRotation() - lastOrientation", Double.valueOf(getYRotation() - lastOrientation));
        //this.telemetry.update();
      } 
    } else {
      /*this.backLeft.setPower(-speed);
      this.backRight.setPower(speed);
      this.frontLeft.setPower(-speed);
      this.frontRight.setPower(speed);*/
      while (getYRotation() < finalOrientation && !flipped && opModeIsActive()) {
        double speedFactor = 1.0D - Math.abs((getYRotation() - startingOrientation) / finalOrientation);
        double rot = getYRotation();
        if (rot - lastOrientation > 300.0D || rot - lastOrientation < -300.0D) {
          flipped = true;
          break;
        } 
        lastOrientation = rot;
        speedFactor += extraSpeed;
        this.backLeft.setPower(speed * speedFactor);
        this.backRight.setPower(-speed * speedFactor);
        this.frontLeft.setPower(speed * speedFactor);
        this.frontRight.setPower(-speed * speedFactor);
        telemetry.addData("this.getYRotation()", this.getYRotation());
        telemetry.addData("finalOrientation", finalOrientation);
        telemetry.addData("getYRotation() >= finalOrientation ? ", getYRotation() >= finalOrientation);
        this.telemetry.addData("this.getYRotation() - lastOrientation", Double.valueOf(getYRotation() - lastOrientation));
        this.telemetry.update();
      } 
    } 
    this.backLeft.setPower(0.0D);
    this.backRight.setPower(0.0D);
    this.frontLeft.setPower(0.0D);
    this.frontRight.setPower(0.0D);
  }
  
  /*public void goToCoordinates(double dx, double dy, double speed) {
    double rotation = this.intendedRotation;
    while (rotation > 360.0D || (rotation < -360.0D && opModeIsActive()))
      rotation += -Math.signum(rotation) * 360.0D; 
    if (rotation > 180.0D) {
      rotation = -360.0D + rotation;
    } else if (rotation < -180.0D) {
      rotation = 360.0D + rotation;
    } 
    if (rotation != 0.0D)
      pivotAngle(-rotation, 0.5D); 
    double forwardDistance = dy - this.y;
    double strafeDistance = dx - this.x;
    goForwardDistance(forwardDistance, speed, true);
    strafeDistance(strafeDistance, speed, true);
  }
  
  public void goToCoordinates(double dx, double dy, double speed, boolean strafeFirst) {
    double rotation = this.intendedRotation;
    while (rotation > 360.0D || (rotation < -360.0D && opModeIsActive()))
      rotation += -Math.signum(rotation) * 360.0D; 
    if (rotation > 180.0D) {
      rotation = -360.0D + rotation;
    } else if (rotation < -180.0D) {
      rotation = 360.0D + rotation;
    } 
    if (rotation != 0.0D)
      pivotAngle(-rotation, 0.5D); 
    double forwardDistance = dy - this.y;
    double strafeDistance = dx - this.x;
    if (!strafeFirst) {
      goForwardDistance(forwardDistance, speed, true);
      strafeDistance(strafeDistance, speed, true);
    } else {
      strafeDistance(strafeDistance, speed, true);
      goForwardDistance(forwardDistance, speed, true);
    } 
  }*/
  
  public void setupIMU() {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    this.imu.initialize(parameters);
  }
  
  public double getYRotation() {
    return (this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle;
  }
  
  public float getPipeRecognition() {
    if (this.tfod != null) {
      List<Recognition> updatedRecognitions = this.tfod.getUpdatedRecognitions();
      
      if (updatedRecognitions != null && updatedRecognitions.size() != 0) {
        Recognition recognition;
        float width;
        int i = 0;
        do {
          recognition = updatedRecognitions.get(i);
          width = recognition.getRight() - recognition.getLeft();
          ++i;
        } while (i < updatedRecognitions.size() && width > 250.0F && opModeIsActive());
        if (width > 250.0F)
          return -1.0F;
        return recognition.getLeft();
      } 
    } 
    return -1.0F;
  }
  
  public float waitForPipeRecognition(double time) {
    double startTime = this.globalTime.seconds();
    boolean detected = false;
    float x = -1.0F;
    while (x < 0.0F && this.globalTime.seconds() < startTime + time && opModeIsActive())
      x = getPipeRecognition(); 
    return x;
  }
  
  /*public boolean waitForDuck(double r, double g, double b, int minimumFrequency, double colorThreshold) throws InterruptedException {
    double startTime = this.globalTime.seconds();
    boolean detected = false;
    double red = 0.0D;
    double green = 0.0D;
    double blue = 0.0D;
    double rtog = (g == 0.0D) ? 0.0D : (r / g);
    double rtob = (b == 0.0D) ? 0.0D : (r / b);
    double gtor = (r == 0.0D) ? 0.0D : (g / r);
    double gtob = (b == 0.0D) ? 0.0D : (g / b);
    double btor = (r == 0.0D) ? 0.0D : (b / r);
    double btog = (g == 0.0D) ? 0.0D : (b / g);
    this.telemetry.addData("rtog", Double.valueOf(rtog));
    this.telemetry.addData("rtob", Double.valueOf(rtob));
    this.telemetry.addData("gtob", Double.valueOf(gtob));
    this.telemetry.update();
    int frequency = 0;
    Frame frame = this.vuforia.getFrameQueue().take();
    Bitmap bmp = this.vuforia.convertFrameToBitmap(frame);
    int[] pixels = new int[bmp.getWidth() * bmp.getHeight()];
    try {
      bmp.getPixels(pixels, 0, bmp.getWidth(), 0, 0, bmp.getWidth(), bmp.getHeight());
    } catch (Exception e) {
      this.telemetry.addLine("Problem fetching bmp pixels");
      this.telemetry.update();
      return false;
    } 
    for (int i = 0; i < pixels.length; i++) {
      red = Color.red(pixels[i]);
      green = Color.green(pixels[i]);
      blue = Color.blue(pixels[i]);
      if (approximatelyEqual((green == 0.0D) ? 0.0D : (red / green), rtog, colorThreshold) && 
        approximatelyEqual((blue == 0.0D) ? 0.0D : (red / blue), rtob, colorThreshold) && 
        approximatelyEqual((red == 0.0D) ? 0.0D : (green / red), gtor, colorThreshold) && 
        approximatelyEqual((blue == 0.0D) ? 0.0D : (green / blue), gtob, colorThreshold) && 
        approximatelyEqual((red == 0.0D) ? 0.0D : (blue / red), btor, colorThreshold) && 
        approximatelyEqual((green == 0.0D) ? 0.0D : (blue / green), btog, colorThreshold)) {
        if (i % 5000 == 0) {
          this.telemetry.addData("i", Integer.valueOf(i));
          this.telemetry.addData("r", Double.valueOf(red));
          this.telemetry.addData("g", Double.valueOf(green));
          this.telemetry.addData("b", Double.valueOf(blue));
        } 
        frequency++;
      } 
    } 
    this.telemetry.addData("frequency", Integer.valueOf(frequency));
    this.telemetry.addData("Detection?: ", Boolean.valueOf((frequency >= minimumFrequency)));
    this.telemetry.update();
    while (opModeIsActive());
    return (frequency >= minimumFrequency);
  }*/
  
  public boolean approximatelyEqual(double val1, double val2, double threshold) {
    return (val1 > val2 - threshold && val1 < val2 + threshold && val2 > val1 - threshold && val2 < val1 + threshold);
  }
  
  /*public boolean waitForDuck(double maxTime) {
    double startTime = this.globalTime.seconds();
    boolean detected = false;
    while (this.globalTime.seconds() < startTime + maxTime && !detected && opModeIsActive()) {
      if (this.tfod != null) {
        List<Recognition> updatedRecognitions = this.tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null && updatedRecognitions.size() != 0) {
          int i = 0;
          boolean found = false;
          while (i < updatedRecognitions.size() && opModeIsActive()) {
            Recognition recognition = updatedRecognitions.get(i);
            this.telemetry.addData("Recognition label", recognition.getLabel());
            if (recognition.getLabel().equals("Duck")) {
              detected = true;
              break;
            } 
            i++;
          } 
        } 
        this.telemetry.update();
        if (detected)
          break; 
      } 
    } 
    return detected;
  }*/
  
  public double moveBackUntilCarousel(double waitTime) {
    double startTime = this.globalTime.seconds();
    double expectedVelocity = 175.0D;
    double velocityThreshold = 12.0D;
    this.carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    this.carousel.setVelocity(expectedVelocity, AngleUnit.DEGREES);
    double moveSpeed = -5.5D;
    double moveStart = this.globalTime.seconds();
    goForwardSpeed(moveSpeed);
    double carouselVelocity = expectedVelocity;
    startTime = this.globalTime.seconds();
    
    double distanceTravelled = 0;
    double lastTime = startTime;
    
    while (this.globalTime.seconds() < startTime + 1.0D&& opModeIsActive());
    do {
      carouselVelocity = this.carousel.getVelocity(AngleUnit.DEGREES);
      
      double vel = ticksToVelocity(this.backLeft.getVelocity());
      
      double elapsed =this.globalTime.seconds();
      
      double dif = elapsed - lastTime;
      lastTime = elapsed;
      
      distanceTravelled += vel * dif;
    } while (approximatelyEqual(expectedVelocity, carouselVelocity, velocityThreshold) && opModeIsActive());
    double finished = this.globalTime.seconds();
    
    this.backLeft.setVelocity(0.0D);
    this.backRight.setVelocity(0.0D);
    this.frontLeft.setVelocity(0.0D);
    this.frontRight.setVelocity(0.0D);
    
    startTime = finished;
    double runTime = waitTime;
    while (this.globalTime.seconds() < startTime + runTime && opModeIsActive());
    return -distanceTravelled;
  }
  
  public double strafeBackUntilCarousel(double waitTime){
    double startTime = this.globalTime.seconds();
    
    double expectedVelocity = -165.0D;
    double velocityThreshold = 12.0D;
    
    this.carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    this.carousel.setVelocity(expectedVelocity, AngleUnit.DEGREES);
    
    double moveSpeed = -5.5D;
    double moveStart = this.globalTime.seconds();
    
    strafeSpeed(moveSpeed);
    
    double carouselVelocity = expectedVelocity;
    startTime = this.globalTime.seconds();
    
    double distanceTravelled = 0;
    double lastTime = startTime;
    
    while (this.globalTime.seconds() < startTime + 1.0D && opModeIsActive());
    
    startTime = this.globalTime.seconds();
    
    do {
      carouselVelocity = this.carousel.getVelocity(AngleUnit.DEGREES);
      
      //double vel = ticksToVelocity(this.backLeft.getVelocity());
      
      //double elapsed =this.globalTime.seconds();
      
      //double dif = elapsed - lastTime;
      //lastTime = elapsed;
      
      //distanceTravelled += vel * dif;
    } while (approximatelyEqual(expectedVelocity, carouselVelocity, velocityThreshold) && opModeIsActive());
    
    telemetry.addData("caoursel velcoity", carouselVelocity);
    telemetry.update();
    
    double finished = this.globalTime.seconds();
    
    distanceTravelled = moveSpeed * (finished - startTime) * strafeMultiplier;
    
    this.backLeft.setVelocity(0.0D);
    this.backRight.setVelocity(0.0D);
    this.frontLeft.setVelocity(0.0D);
    this.frontRight.setVelocity(0.0D);
    
    startTime = finished;
    
    double runTime = waitTime;
    
    while (this.globalTime.seconds() < startTime + runTime && opModeIsActive());
    
    return distanceTravelled;
  }
  
  public double toRadians(double degrees) {
    return degrees * Math.PI / 180.0D;
  }
  
  public double toDegrees(double radians) {
    return radians * 180.0D / Math.PI;
  }
  
  public void setArmLevel(int level, int offset, double power) {
    this.armMotor.setTargetPosition(this.armPositions[level] + offset);
    this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.armMotor.setPower(power);
  }
  
  public void setArmPosition(int position, double power){
    this.armMotor.setTargetPosition(position);
    this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.armMotor.setPower(power);
  }
  
  public String getRandomFinalMessage(){
    int index = (int)(Math.random() * finalMessages.length);
    
    return finalMessages[index];
  }
  
  public void wait(double time){
    double start = this.globalTime.seconds();
    
    while(this.globalTime.seconds() <= start+time && opModeIsActive());
  }
}
