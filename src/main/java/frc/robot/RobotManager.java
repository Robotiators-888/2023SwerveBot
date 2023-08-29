package frc.robot;

public class RobotManager {
    public enum TargetScoring{
        HIGH("High"),
        MED("Medium"),
        LOW("Low");

        private final String height;
        TargetScoring(final String height){
            this.height = height;
        }

        public String toString(){
            return height;
        }
    }

    public RobotManager instance = null;
    private TargetScoring targetScoring;

    public synchronized RobotManager getInstance() {
        if(instance == null){
            instance = new RobotManager();
        }
        return instance;
    }

    private RobotManager(){
        targetScoring = TargetScoring.HIGH;
    }

    public String getScoringHeight(){

    }

    public Command build
    
    
}
