package frc.team3128.common.utility;

import java.util.HashMap;
import java.util.function.Supplier;

import javax.lang.model.element.Name;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import static frc.team3128.Constants.Shuffleboard_Constants.*;

public class NAR_Shuffleboard{

    private static class tabInfo {
        public NetworkTableEntry m_data;
        private Supplier<Object> m_supply;
        public tabInfo(NetworkTableEntry data, Supplier<Object> supply){
            m_supply = supply;
            m_data = data;
        }

        public void update() {
            if(m_supply == null) return;
            m_data.setValue(m_supply.get());
        }
    }
    private static HashMap<String, HashMap<String, tabInfo>> tabs;

    static {
        tabs = new HashMap<String, HashMap<String,tabInfo>>();
        for(int i = 0; i < TabNames.length; i++) {
            create_tab(TabNames[i]);
        }

    }
    private static void create_tab(String tabName) {
        tabs.put(tabName, new HashMap<String,tabInfo>());
    }

    public static void addData(String tabName, String name, Supplier<Object> supply){
        addData(tabName,name,supply.get(),supply);
    }

    public static void addData(String tabName, String name, Object data){
        addData(tabName,name,data,null);
    }

    public static void addData(String tabName, String name, Object data, Supplier<Object> supply){
        NetworkTableEntry entry = Shuffleboard.getTab(tabName).add(name,data).getEntry();
        tabs.get(tabName).put(name,new tabInfo(entry,supply));
    }

    public Object getData(String tabName, String dataName) {
        return tabs.get(tabName).get(dataName).m_data.getValue();
    }

    public static void update() {
        for(String i : tabs.keySet()){
            for(String j : tabs.get(i).keySet()){
                tabs.get(i).get(j).update();
            }
        }
    }

}
