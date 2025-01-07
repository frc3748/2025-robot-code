package frc.robot.Utilities.Config;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class DynaPerfConf {
    public String SubPref;
    public DynaPerfConf(String ID){
        this.SubPref = ID + "/";
    }
    public void setDouble(String key, double val){
        Preferences.initDouble(SubPref + key, val);
    }
    public void setInt(String key, int val){
        Preferences.initInt(SubPref + key, val);
    }
    public void setString(String key, String val){
        Preferences.initString(SubPref + key, val);
    }
    public void setBool(String key, boolean val){
        Preferences.initBoolean(SubPref + key, val);
    }
    public double getDouble(String key){
        if(Preferences.containsKey(SubPref + key)){
         return Preferences.getDouble(SubPref + key, 0.0);   
        }
        DriverStation.reportError("Double key " + key + " not found!", false);
        return 0;

    }
    public int getInt(String key){
        if(Preferences.containsKey(SubPref + key)){
            return Preferences.getInt(SubPref + key, 0);
        }
        DriverStation.reportError("Int key " + key + " not found!", false);
        return 0;

    }
    public String getString(String key){
        if(Preferences.containsKey(SubPref + key)){
            return Preferences.getString(SubPref + key, "");
        }
        DriverStation.reportError("String key " + key + " not found!", false);
        return null;
    }
    public boolean getBool(String key){
        if(Preferences.containsKey(SubPref + key)){
            return Preferences.getBoolean(SubPref + key, false);
        }
        DriverStation.reportError("Boolean key " + key + " not found!", false);
        return false;
    }
}
