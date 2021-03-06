package frc.robot;

import frc.robot.Vision.RedHalfPipeline;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class VisionTest extends TimedRobot {

    private static final int IMG_WIDTH = 320;
    private static final int IMG_HEIGHT = 240;
    private VisionThread visionThread;
    private double centerX = 0.0;
    private final Object imgLock = new Object();
    private final MotorController m_frontLeft = new WPI_TalonSRX(1);
    private final MotorController m_rearLeft = new WPI_TalonSRX(3);
    private final MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

    private final MotorController m_frontRight = new WPI_TalonSRX(2);
    private final MotorController m_rearRight = new WPI_TalonSRX(4);
    private final MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
    

@Override
public void robotInit() {
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    visionThread = new VisionThread(camera, new RedHalfPipeline(), pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
            synchronized (imgLock) {
                centerX = r.x + (r.width / 2);
            }
        }
        else{
            synchronized (imgLock) {
                centerX = 0;
            }
        }
        
    });
    visionThread.start();
}

@Override
public void autonomousPeriodic() {
    double centerX;
    synchronized (imgLock) {
        centerX = this.centerX;
    }

    double turn = centerX - (IMG_WIDTH / 2);
    String s=String.valueOf(turn);  
    String x=String.valueOf(centerX); 
    System.out.print("Turn value is" + s);
    System.out.print("X value is" + x);
    if (centerX == 0){
        m_drive.arcadeDrive(0, 0);
    }
    else if(centerX <= 170 && centerX >= 150){
        m_drive.arcadeDrive(0.6, 0);
    }
    else{
        m_drive.arcadeDrive(0.6, turn * 0.005);
    }
    
    try {
        Thread.sleep(100);
    } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
    }
    }
}