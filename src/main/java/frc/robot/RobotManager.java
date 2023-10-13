package frc.robot;

public class RobotManager {
    public enum TargetScoring{
        HIGH(Constants.Manuiplator.kScoreHigh),
        MED(Constants.Manuiplator.kScoreMid),
        LOW(Constants.Manuiplator.kScoreLow);

        private final double height;
        TargetScoring(final double height){
            this.height = height;
        }

        public double getValue(){
            return height;
        }
    }

    public static RobotManager instance = null;
    private TargetScoring targetScoring;

    public synchronized static RobotManager getInstance() {
        if(instance == null){
            instance = new RobotManager();
        }
        return instance;
    }

    private RobotManager(){
        targetScoring = TargetScoring.HIGH;
    }
    public void setScoringHeightLow(){
        targetScoring = TargetScoring.LOW;
    }
    public void setScoringHeightMid(){
        targetScoring = TargetScoring.MED;
    }
    public void setScoringHeightHigh(){
        targetScoring = TargetScoring.HIGH;
    }

    public double getScoringHeight(){
        return targetScoring.getValue();
    }
   
    
}
