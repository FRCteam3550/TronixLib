package TronixLib;

public class TuningPilotAxis {

    private int m_mode; // 0 : tel que
                        // 1 : Au carre
                        // 2 : exponentiel? etc 

    public TuningPilotAxis() {
        m_mode = 0;
    }

    public TuningPilotAxis(int tuningmode) {
        m_mode = tuningmode;
    }

    public void setTuningMode(int newtuningmode) {
        m_mode = newtuningmode;
    }

    public int getTuningMode() {
        return m_mode;
    }

    public double getEnhancedAxis(double axisvalue) {
        double finalvalue = 0;

        if (m_mode == 0) {
            finalvalue = axisvalue;
        }
        else if (m_mode == 1){
            finalvalue =  Math.copySign(axisvalue * axisvalue, axisvalue);
        }

        return finalvalue;
    }
}