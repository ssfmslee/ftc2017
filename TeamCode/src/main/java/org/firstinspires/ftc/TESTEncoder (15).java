package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.Timer;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
/**
 * {@link SensorMRRangeSensor} illustrates how to use the Modern Robotics
 * Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@Autonomous(name = "TESTEncoder", group = "Sensor")

public class TESTEncoder extends LinearOpMode {
    public DcMotor Motor5 = null;
    private Servo JewelServo = null;
    private ElapsedTime Timer = new ElapsedTime();
    
    private String blank = "";
    
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    
    private ElapsedTime runtime = new ElapsedTime();
   
    
     private Servo Servo1      = null;
    private Servo Servo2     = null;
    private Servo Servo4      = null;
    private Servo Servo5     = null;
    
     private double pivotR;
      private double verticalDrvF;
     private double verticalDrvR;
     private double verticalDrv;
     private double horizontalDrv;
     
     private ColorSensor colorSensor;
    
    private Servo Servo3 = null;
    private DcMotor Extend      = null;
    private DcMotor MotorLift       = null;
    private DcMotor Motor1      = null;
    private DcMotor Motor2      = null;
    private DcMotor Motor3      = null;
    private DcMotor Motor4      = null;
  
    private double left_posTop    = 0;
    private double right_posTop   = 1.0;
    private double left_posBot    = 0;
    private double right_posBot   = 1.0;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    private double ExtendPower;
    private double LiftPower;
    private boolean isScanDone = false;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private int color = 0;
    
    
    private boolean isPosition = true;
    
    private String VuMark = "-";
    
    IntegratingGyroscope gyro;
  ModernRoboticsI2cGyro modernRoboticsI2cGyro; 

  public void zeroAllMotor(){
   Motor1.setPower(0);
   Motor2.setPower(0);
   Motor3.setPower(0);
   Motor4.setPower(0);
  }

    @Override public void runOpMode() {
        // get a reference to our compass
        Motor5 = hardwareMap.get(DcMotor.class, "Motor5");
        JewelServo = hardwareMap.get(Servo.class, "JewelServo");
        JewelServo.setDirection(Servo.Direction.FORWARD);
        
       Motor5.setDirection(DcMotor.Direction.FORWARD);
       Motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       Motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
         
   boolean lastResetState = false;
    boolean curResetState  = false;
    
     modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
   telemetry.log().add("Gyro Calibrating. Do Not Move!");
    modernRoboticsI2cGyro.calibrate();
    
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //South City Robotics 2018 VuforiaLicenseKey 
        //https://developer.vuforia.com/license-manager
        //email darrendotng@gmail.com to obtain username and password to manage license keys
        parameters.vuforiaLicenseKey = "ATbZfEb/////AAAAmQZejrUuP0+4ilV5yBR2nxhz5jMi9FeNdT1dA+wRG/tlwKxFyLVR+WzYN+H7YngjFFE60ojETbbaieEC5Jqa7yJu7qaDsCj6a1biqAafZL6iGrSCE5218seIoIINnYAnWXIGCZD8lSjRPQ0oK6m5cpDdjDF9+ghVmDcBwkDRZk4kRWnYsQIBYlfHDvu6Ct9q6Yso+zQNn2PSrXA2rZg01eDMZz2MS5Fb+Oi9LHKmdG22oXejb6xd4MwxW60m4uxlcDdqcve7wOv9D3oC6y4qY1pjitP1YpQ4tjK0qG285tdMj+1WPIMHJ8/96XOa29CvOjbGXZft71L7rwxy2r+iE3hq/V+0sHCtYB1KOu2pquIx";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

      
        

     //Motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       
        // wait for the start button to be pressed
        
        waitForStart();
        
  

        
           relicTrackables.activate(); 
          JewelServo.setPosition(1); 
           boolean bLedOn = true;
        boolean isOpModeActive = true;
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");
        
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        
    Servo1  = hardwareMap.get(Servo.class, "Servo1");
    Servo2 = hardwareMap.get(Servo.class, "Servo2");
    Servo4  = hardwareMap.get(Servo.class, "Servo4");
    Servo5 = hardwareMap.get(Servo.class, "Servo5");
    
     MotorLift     = hardwareMap.get(DcMotor.class, "MotorLift");
     
    colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
     
     Servo1.scaleRange(MIN_POS,MAX_POS);
    Servo2.scaleRange(MIN_POS,MAX_POS);
    
    Servo4.scaleRange(MIN_POS,MAX_POS);
    Servo5.scaleRange(MIN_POS,MAX_POS);
    
    Servo1.setPosition(left_posTop);
    Servo2.setPosition(right_posTop);
    
    Servo4.setPosition(right_posBot);
    Servo5.setPosition(left_posBot);
    
     MotorLift.setDirection(DcMotor.Direction.REVERSE);
    Motor1.setDirection(DcMotor.Direction.FORWARD);
    Motor2.setDirection(DcMotor.Direction.FORWARD);
    Motor3.setDirection(DcMotor.Direction.FORWARD);
    Motor4.setDirection(DcMotor.Direction.FORWARD);
    
    
          
        Timer.reset();

     Servo4.setPosition(0);
     Servo5.setPosition(1);
     Servo1.setPosition(.90);
 

Timer.reset();
 while(Timer.seconds()<1&&opModeIsActive()){
     MotorLift.setPower(0.3);
 }
MotorLift.setPower(0);

       
       
       
         
         
            while(Motor5.getCurrentPosition()>-300&&opModeIsActive()){
                Motor5.setPower(-0.1);
              
            telemetry.addData("Pos", JewelServo.getPosition());
         
            telemetry.update();
             }
             
             
             Timer.reset();
             boolean first = false;
             while(Timer.seconds()<0.1&&opModeIsActive()){
                
                    Motor5.setPower(0.1);
                    telemetry.addData("Pos", JewelServo.getPosition());
         
            telemetry.update();
                
             }
             Motor5.setPower(0);
                     
                    
                    
                   JewelServo.setPosition(.5);
                   //  JewelServo.setPosition(14);
                    
                    
             
             
        //     Motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             Motor5.setPower(0);
        //     Motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             
        
        Timer.reset();
 while(Timer.seconds()<1&&opModeIsActive()){
  
 RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);
               
                VuMark = vuMark.toString();
                if(vuMark.toString() == "RIGHT"){
                telemetry.addData("VUUUIEMARK", VuMark);
                }
                 telemetry.update();
                 

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
              //  telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
 }
 int heading = modernRoboticsI2cGyro.getHeading();

Timer.reset();

int colorSense = 0; //note that 0 = no color dectected 
//1 = blue detected
//2 = red detected

 while(Timer.seconds()<1&&opModeIsActive()){
     if((colorSensor.red()>0)&&(colorSensor.blue()==0)){ 
         //if red
         telemetry.addData("Red","Red");
        colorSense = 2;
     }
  else   if((colorSensor.blue()>0)&&(colorSensor.red()==0)){
         //if blue
         telemetry.addData("blue","Blue");
        colorSense = 1;
}

         telemetry.update();
}
Timer.reset();

//=================================================
 if(colorSense == 2){//if red
 while(Timer.seconds()<.5&&opModeIsActive()){
  Motor3.setPower(-0.1);
         Motor4.setPower(-0.1);
         Motor2.setPower(-0.1);
         Motor1.setPower(-0.1);
         Motor5.setPower(0.1);
 }
 Motor3.setPower(0);
         Motor4.setPower(0);
         Motor2.setPower(0);
         Motor1.setPower(0);
         Timer.reset();
         
           while(Motor5.getCurrentPosition()<-30&&opModeIsActive()){
                Motor5.setPower(0.1);
             }
             Timer.reset();
            
             Motor5.setPower(0);
                   JewelServo.setPosition(1);
             Motor5.setPower(0);
             
   int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
while(integratedZ<-7&&opModeIsActive()){
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
    Motor4.setPower(0.1);
    Motor3.setPower(0.1);
    Motor1.setPower(0.1);
    Motor2.setPower(0.1);
      telemetry.addData("Turning",integratedZ);
    telemetry.update();
        }
while(integratedZ<-5&&opModeIsActive()){
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
    Motor4.setPower(0.05);
    Motor3.setPower(0.05);
    Motor1.setPower(0.05);
    Motor2.setPower(0.05);
    telemetry.addData("Turning",integratedZ);
    telemetry.update();
        }
         Motor3.setPower(0);
         Motor4.setPower(0);
         Motor2.setPower(0);
         Motor1.setPower(0);
 
  Timer.reset();
 while(Timer.seconds()<0.5&&opModeIsActive()){ // lower from 1 second
  //chill out
 }
 Timer.reset();
 while(Timer.seconds()<1.5&&opModeIsActive()){
  Motor3.setPower(0.3);
  Motor4.setPower(-0.3);
 }
  while(Timer.seconds()<0.5&&opModeIsActive()){  //lowered from 1 second
 Motor3.setPower(0);
 Motor4.setPower(0);
 Motor2.setPower(0);
 Motor1.setPower(0);
 }   
}

//================================================
else if(colorSense == 1){//if blue
 while(Timer.seconds()<.5&&opModeIsActive()){
   Motor3.setPower(0.1);
         Motor4.setPower(0.1);
         Motor2.setPower(0.1);
         Motor1.setPower(0.1);
          Motor5.setPower(0.1);
          
 }
 Motor3.setPower(0);
         Motor4.setPower(0);
         Motor2.setPower(0);
         Motor1.setPower(0);
         Timer.reset();
         
     
         
           while(Motor5.getCurrentPosition()<-30&&opModeIsActive()){
                Motor5.setPower(0.1);
             }
             Timer.reset();
            
             Motor5.setPower(0);
                   JewelServo.setPosition(1);
             Motor5.setPower(0);
             
 int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
while(integratedZ>17&&opModeIsActive()){
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
    Motor4.setPower(-0.1);
    Motor3.setPower(-0.1);
    Motor1.setPower(-0.1);
    Motor2.setPower(-0.1);
    telemetry.addData("Turning",integratedZ);
    telemetry.update();
        }
while(integratedZ>10&&opModeIsActive()){
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
    Motor4.setPower(-0.05);
    Motor3.setPower(-0.05);
    Motor1.setPower(-0.05);
    Motor2.setPower(-0.05);
    telemetry.addData("Turning",integratedZ);
    telemetry.update();
        }
       
 Motor3.setPower(0);
         Motor4.setPower(0);
         Motor2.setPower(0);
         Motor1.setPower(0);

 Timer.reset();
 while(Timer.seconds()<0.5&&opModeIsActive()){ 
  //chill out
 }
 Timer.reset();
 while(Timer.seconds()<1.7&&opModeIsActive()){
  Motor3.setPower(0.3);
  Motor4.setPower(-0.3);
 }
  while(Timer.seconds()<0.5&&opModeIsActive()){  
 Motor3.setPower(0);
 Motor4.setPower(0);
 Motor2.setPower(0);
 Motor1.setPower(0);
 }
}
//==========================================
//if no detection
else if(colorSense == 0){
 //retrach arm
  while(Motor5.getCurrentPosition()<-35&&opModeIsActive()){
                Motor5.setPower(0.1);
             }
             Timer.reset();
            
             Motor5.setPower(0);
                   JewelServo.setPosition(1);
             Motor5.setPower(0);
             
}
  Motor3.setPower(0);
         Motor4.setPower(0);
         Motor2.setPower(0);
         Motor1.setPower(0);
 

 
 //=====================================
 int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
 if(integratedZ>0){
while(integratedZ>17&&opModeIsActive()){
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
    Motor4.setPower(-0.1);
    Motor3.setPower(-0.1);
    Motor1.setPower(-0.1);
    Motor2.setPower(-0.1);
    telemetry.addData("Turning",integratedZ);
    telemetry.update();
        }
while(integratedZ>10&&opModeIsActive()){
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
    Motor4.setPower(-0.05);
    Motor3.setPower(-0.05);
    Motor1.setPower(-0.05);
    Motor2.setPower(-0.05);
    telemetry.addData("Turning",integratedZ);
    telemetry.update();
        }
        Timer.reset();
 
 Motor3.setPower(0);
 Motor4.setPower(0);
 Motor2.setPower(0);
 Motor1.setPower(0);
 
 
             
        //    }
 }
 
 //======================================
 if(integratedZ<0){
while(integratedZ<-7&&opModeIsActive()){
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
    Motor4.setPower(0.1);
    Motor3.setPower(0.1);
    Motor1.setPower(0.1);
    Motor2.setPower(0.1);
      telemetry.addData("Turning",integratedZ);
    telemetry.update();
        }
while(integratedZ>-5&&opModeIsActive()){
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
    Motor4.setPower(0.05);
    Motor3.setPower(0.05);
    Motor1.setPower(0.05);
    Motor2.setPower(0.05);
    telemetry.addData("Turning",integratedZ);
    telemetry.update();
        }
         Motor3.setPower(0);
         Motor4.setPower(0);
         Motor2.setPower(0);
         Motor1.setPower(0);
    
 }
  while(Timer.seconds()<1){ 
 Motor3.setPower(0);
 Motor4.setPower(0);
 Motor2.setPower(0);
 Motor1.setPower(0);
 }
 

 
 
 


  //=============================================
 //spin 180
 while(integratedZ>-164&&opModeIsActive()){ //was 163
  integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
 Motor3.setPower(-0.1);
 Motor4.setPower(-0.1);
 Motor2.setPower(-0.1);
 Motor1.setPower(-0.1);
 telemetry.addData("TurningRed",integratedZ);
 telemetry.update();
 }
  zeroAllMotor();
 
  

 //boolean isBackboard = true;
 //boolean foundWedge = false;
 /*
 double time;
 Timer.reset();
 while(rangeSensor.rawUltrasonic()>40&&opModeIsActive()){
  Motor1.setPower(0.1);
  Motor2.setPower(-0.1);
  telemetry.addData("Distance",rangeSensor.rawUltrasonic());
 telemetry.update();
 }
 time = Timer.seconds();
 zeroAllMotor();
 */
 Timer.reset();
 //move towards alliance board, past crypto box
 while(Timer.seconds()<2.4&&opModeIsActive()){
  Motor4.setPower(0.1);
 Motor3.setPower(-0.1);
 telemetry.addData("Seconds",Timer.seconds());
 telemetry.addData("VuMark",VuMark);
 telemetry.update();
 
 }
 Timer.reset();
 while(Timer.seconds()<0.5&&opModeIsActive()){
   zeroAllMotor();
 }

 //drive towards wall
 int initalWallDistance;
 while(rangeSensor.rawUltrasonic()>50){ //was 44
  Motor1.setPower(0.15);
  Motor2.setPower(-0.15);
  telemetry.addData("Distance",rangeSensor.rawUltrasonic());
   telemetry.addData("VuMark",VuMark);
 telemetry.update();
 }
 Timer.reset();
 while(Timer.seconds()<0.5&&opModeIsActive()){
 zeroAllMotor();
 }
 initalWallDistance = rangeSensor.rawUltrasonic();
 //drives to wedge
 while(initalWallDistance-4<=rangeSensor.rawUltrasonic()&&opModeIsActive()){
  Motor4.setPower(-0.1);
  Motor3.setPower(0.1);
  telemetry.addData("Distance",rangeSensor.rawUltrasonic());
   telemetry.addData("VuMark",VuMark);
 telemetry.update();
  
 }
 //after wedge is seen
 //if vuMArk is not seen or is LEFT
 //===========================================================
 if(VuMark=="-"||VuMark=="LEFT"){
  
 
 Timer.reset();
 //stops after certain time after wedge
 while(Timer.seconds()<0.1&&opModeIsActive()){ //from 0.17 seconds
  
 }
 Timer.reset();
 while(Timer.seconds()<0.5&&opModeIsActive()){
  zeroAllMotor();
 }
 Timer.reset();
 //banks it in
 while(Timer.seconds()<.5&&opModeIsActive()){
  Motor1.setPower(0.15);
  Motor2.setPower(-0.15);
  
 }
 while(Timer.seconds()<.7&&opModeIsActive()){
  Motor4.setPower(0.1);
  Motor3.setPower(-0.1);
 }
 while(Timer.seconds()<.9&&opModeIsActive()){
  Motor4.setPower(-0.1);
  Motor3.setPower(0.1);
 }
 while(Timer.seconds()<1.3&&opModeIsActive()){
  Motor4.setPower(0.1);
  Motor3.setPower(-0.1);
 }
   while(Timer.seconds()<1.5&&opModeIsActive()){
  Motor4.setPower(-0.1);
  Motor3.setPower(0.1);
   }
   while(Timer.seconds()<2&&opModeIsActive()){
    Motor4.setPower(0);
    Motor3.setPower(0);
   }
 }
 //if vuMArk is equal to CENTER
 //==================================================
 if(VuMark=="CENTER"){
 
 Timer.reset();
 //stops after certain time after wedge
 while(Timer.seconds()<1.37&&opModeIsActive()){ //from 0.17 seconds
   telemetry.addData("Center", VuMark);
  telemetry.update();
 }
 Timer.reset();
 while(Timer.seconds()<0.5&&opModeIsActive()){
  zeroAllMotor();
   telemetry.addData("Center", VuMark);
  telemetry.update();
 }
 Timer.reset();
 //banks it in
 while(Timer.seconds()<.5&&opModeIsActive()){
  Motor1.setPower(0.15);
  Motor2.setPower(-0.15);
   telemetry.addData("Center", VuMark);
  telemetry.update();
 }
 while(Timer.seconds()<.7&&opModeIsActive()){
  Motor4.setPower(0.1);
  Motor3.setPower(-0.1);
   telemetry.addData("Center", VuMark);
  telemetry.update();
 }
 while(Timer.seconds()<.9&&opModeIsActive()){
  Motor4.setPower(-0.1);
  Motor3.setPower(0.1);
   telemetry.addData("Center", VuMark);
  telemetry.update();
 }
 while(Timer.seconds()<1.3&&opModeIsActive()){
  Motor4.setPower(0.1);
  Motor3.setPower(-0.1);
   telemetry.addData("Center", VuMark);
  telemetry.update();
 }
   while(Timer.seconds()<1.5&&opModeIsActive()){
  Motor4.setPower(-0.1);
  Motor3.setPower(0.1);
   telemetry.addData("Center", VuMark);
  telemetry.update();
   }
   while(Timer.seconds()<2&&opModeIsActive()){
    Motor4.setPower(0);
    Motor3.setPower(0);
     telemetry.addData("Center", VuMark);
  telemetry.update();
   }
 }
 //if vuMark is equal to RIGHT
 //========================================================
 if(VuMark=="RIGHT"){
  
 Timer.reset();
 //stops after certain time after wedge
 while(Timer.seconds()<2.64&&opModeIsActive()){ //from 0.17 seconds
  
 }
 Timer.reset();
 while(Timer.seconds()<0.5&&opModeIsActive()){
  zeroAllMotor();
 }
 Timer.reset();
 //banks it in
 while(Timer.seconds()<.5&&opModeIsActive()){
  Motor1.setPower(0.15);
  Motor2.setPower(-0.15);
  
 }
 while(Timer.seconds()<.7&&opModeIsActive()){
  Motor4.setPower(0.1);
  Motor3.setPower(-0.1);
 }
 while(Timer.seconds()<.9&&opModeIsActive()){
  Motor4.setPower(-0.1);
  Motor3.setPower(0.1);
 }
 while(Timer.seconds()<1.3&&opModeIsActive()){
  Motor4.setPower(0.1);
  Motor3.setPower(-0.1);
 }
   while(Timer.seconds()<1.5&&opModeIsActive()){
  Motor4.setPower(-0.1);
  Motor3.setPower(0.1);
   }
   while(Timer.seconds()<2&&opModeIsActive()){
    Motor4.setPower(0);
    Motor3.setPower(0);
   }
 }
 zeroAllMotor();
 
 
 Servo4.setPosition(1);
     Servo5.setPosition(0);
     Timer.reset();
     while(Timer.seconds()<0.5){
     }
     
     Timer.reset();
 while(Timer.seconds()<0.6&&opModeIsActive()){
  Motor1.setPower(-0.15);
  Motor2.setPower(0.15);
 }
 zeroAllMotor();
 
 
 while(integratedZ<0&&opModeIsActive()){
  integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
 Motor3.setPower(0.1);
 Motor4.setPower(0.1);
 Motor2.setPower(0.1);
 Motor1.setPower(0.1);
 telemetry.addData("...Preparing for Battle...",integratedZ);
 telemetry.update();
 }
  zeroAllMotor();
  
  Timer.reset();
 while(Timer.seconds()<1&&opModeIsActive()){
     MotorLift.setPower(-0.3);
 }
MotorLift.setPower(0);
 
 
 while(opModeIsActive()){
  telemetry.addData("Distance",rangeSensor.rawUltrasonic());
 telemetry.update();
 }
 
    }
    
    
}
