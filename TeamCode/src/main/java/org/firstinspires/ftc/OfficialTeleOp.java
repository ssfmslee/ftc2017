package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="OfficialTeleOp", group="Linear Opmode")

public class OfficialTeleOp extends LinearOpMode {
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
    
    private Servo Servo3 = null;
 //   private DcMotor Extend      = null;
    private DcMotor Chain       = null;
    private DcMotor Motor1      = null;
    private DcMotor Motor2      = null;
    private DcMotor Motor3      = null;
    private DcMotor Motor4      = null;
    private double left_pos     = 0;
    private double right_pos    = 1.0;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    private double ExtendPower;
    private double ChainPower;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
    //Hardware Map    
    Servo1  = hardwareMap.get(Servo.class, "Servo1");
    Servo2 = hardwareMap.get(Servo.class, "Servo2");
    Servo4  = hardwareMap.get(Servo.class, "Servo4");
    Servo5 = hardwareMap.get(Servo.class, "Servo5");
    
    Servo3 = hardwareMap.get(Servo.class, "Servo3");
 //   Extend    = hardwareMap.get(DcMotor.class, "MotorExtend");
    Chain     = hardwareMap.get(DcMotor.class, "MotorLift");
    Motor1    = hardwareMap.get(DcMotor.class, "Motor1");
    Motor2    = hardwareMap.get(DcMotor.class, "Motor2");
    Motor3    = hardwareMap.get(DcMotor.class, "Motor3");
    Motor4    = hardwareMap.get(DcMotor.class, "Motor4");
    
    //Setup
    Servo1.scaleRange(MIN_POS,MAX_POS);
    Servo2.scaleRange(MIN_POS,MAX_POS);
    
    Servo4.scaleRange(MIN_POS,MAX_POS);
    Servo5.scaleRange(MIN_POS,MAX_POS);
    
    Servo1.setPosition(left_pos);
    Servo2.setPosition(right_pos);
    
    Servo4.setPosition(right_pos);
    Servo5.setPosition(left_pos);
    
    
    Servo3.scaleRange(MIN_POS,MAX_POS);
    Servo3.setPosition(1.0);
//    Extend.setDirection(DcMotor.Direction.FORWARD);
    Chain.setDirection(DcMotor.Direction.FORWARD);
    Motor1.setDirection(DcMotor.Direction.FORWARD);
    Motor2.setDirection(DcMotor.Direction.FORWARD);
    Motor3.setDirection(DcMotor.Direction.FORWARD);
    Motor4.setDirection(DcMotor.Direction.FORWARD);
        
        // Wait for start
        waitForStart();
        runtime.reset();
        
        //Main
        while (opModeIsActive()) {
        
        //Grabber
        if (gamepad2.dpad_down){
            left_pos +=0.10;
            right_pos -=0.10;
            }
        if (gamepad2.dpad_up){
            left_pos -=0.10;
            right_pos +=0.10;
            }
        if (right_pos >0.99){
            right_pos =0.99;
            }   
        if (left_pos <0){
            left_pos =0;
            }
        if (left_pos >0.99){
            left_pos =0.99;
            }   
        if (right_pos <0){
            right_pos =0;
            }
       Servo1.setPosition(left_pos);
       Servo2.setPosition(right_pos);
       Servo4.setPosition(right_pos);
       Servo5.setPosition(left_pos);
            
        //Chain and Extend
        ExtendPower  = gamepad2.left_stick_y ;
        if(gamepad2.right_trigger == 0)
        Chain.setPower(-gamepad2.left_trigger);
        
        if(gamepad2.left_trigger == 0)
        Chain.setPower(gamepad2.right_trigger);
    
  //      Extend.setPower(ExtendPower/3);
    //    Chain.setPower(ChainPower/3);
        
        //Drive/Pivot
        
    
        pivotR  = -gamepad1.left_stick_x ;
        verticalDrvF = gamepad1.right_trigger;
        verticalDrvR = gamepad1.left_trigger;
       
        //x' = xcos(angle)-ysin(angle)
        horizontalDrv = (gamepad1.right_stick_x)*Math.cos(Math.toRadians(-45))-(-gamepad1.right_stick_y)*(Math.sin(Math.toRadians(-45)));
        //y' = ycos(angle)+xsin(angle)
        verticalDrv = (-gamepad1.right_stick_y)*Math.cos(Math.toRadians(-45))+(gamepad1.right_stick_x)*(Math.sin(Math.toRadians(-45)));
       
        if((gamepad1.right_trigger == 0.0) && (gamepad1.left_trigger == 0.0) && (gamepad1.right_stick_y == 0.0) && (gamepad1.right_stick_x == 0.0)){
        Motor1.setPower(pivotR/2);
        Motor2.setPower(pivotR/2);
        Motor3.setPower(pivotR/2);
        Motor4.setPower(pivotR/2);
        }
        
        if((gamepad1.left_stick_x == 0.0)&&(gamepad1.left_trigger == 0.0)&&(gamepad1.right_trigger == 0.0)){
        Motor2.setPower(verticalDrv);
        Motor4.setPower(-verticalDrv);
        Motor1.setPower(horizontalDrv);
        Motor3.setPower(-horizontalDrv); 
        }
        //End of Main
        
        }
    }
}
