package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.limelight_pipeline;
import frc.robot.modules.LimelightModule;
import frc.robot.modules.LimelightModule.LightMode;
import frc.robot.modules.LimelightModule.camMode;

public class Limelight extends SubsystemBase{

    LimelightModule module = new LimelightModule();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    @Override
    public void periodic()
    {
        double[] data = {
            LimelightModule.getBasicData("tv"),
            LimelightModule.getBasicData("tx"),
            LimelightModule.getBasicData("ty"),
            LimelightModule.getBasicData("ta"),
            LimelightModule.getBasicData("tid"),
            module.get_AprilTag_3D_Data("botpose")[0],
            module.get_AprilTag_3D_Data("botpose")[1],
            module.get_AprilTag_3D_Data("botpose")[2],
            (module.get_AprilTag_3D_Data("botpose")[3]+360)%360,
            (module.get_AprilTag_3D_Data("botpose")[4]+360)%360,
            (module.get_AprilTag_3D_Data("botpose")[5]+360)%360
        };
        SmartDashboard.putNumber("Limelight/tv", data[0]);
        SmartDashboard.putNumber("Limelight/tx", data[1]);
        SmartDashboard.putNumber("Limelight/ty", data[2]);
        SmartDashboard.putNumber("Limelight/ta", data[3]);
        SmartDashboard.putNumber("Limelight/tid", data[4]);

        SmartDashboard.putNumber("Limelight/field_x", data[5]);
        SmartDashboard.putNumber("Limelight/field_y", data[6]);
        SmartDashboard.putNumber("Limelight/field_z", data[7]);
        SmartDashboard.putNumber("Limelight/field_Roll", data[8]);
        SmartDashboard.putNumber("Limelight/field_Pitch", data[9]);
        SmartDashboard.putNumber("Limelight/field_Yaw", data[10]);

    }

    /**
     * @return v, x, y, a
     */
    public double[] get_tag_data(limelight_pipeline pipeline){
        module.set_Cam_Control("pipeline", pipeline.value);

        double[] data = {
            LimelightModule.getBasicData("tv"),
            LimelightModule.getBasicData("tx"),
            LimelightModule.getBasicData("ty"),
            LimelightModule.getBasicData("ta"),
            LimelightModule.getBasicData("tid"),
            module.get_AprilTag_3D_Data("robotpose")[0],
            module.get_AprilTag_3D_Data("robotpose")[1],
            module.get_AprilTag_3D_Data("robotpose")[2],
            (module.get_AprilTag_3D_Data("robotpose")[3]+360)%360,
            (module.get_AprilTag_3D_Data("robotpose")[4]+360)%360,
            (module.get_AprilTag_3D_Data("robotpose")[5]+360)%360
        };

        return data;
    }

     
    public void setLightCode(LightMode mode){
        module.setLightMode(mode);
    }
    
    public double[] getAprilTagID(){
        return module.get_AprilTag_3D_Data("tid");
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(module.get_AprilTag_3D_Data("botpose")[5]);
    }

    public void setDriverMode(){
        module.set_Cam_Control("pipeline", limelight_pipeline.driver.value);
    }

    public int get_pipeline(){
        return (int)LimelightModule.getBasicData("getpipe");
    }

    public void limeretrorefliective(){
        module.set_Cam_Control("retrorefliective", limelight_pipeline.reflective.value);
    }

    public void limeapriltag(){
        module.set_Cam_Control("apriltag", limelight_pipeline.aprilTag.value);
    }

    public double limelighttx(){
        return table.getEntry("tx").getDouble(0.0);
    }
    public double limelightty(){
        return table.getEntry("ty").getDouble(0.0);
    }

    public boolean limelighttv(){
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }
 
}