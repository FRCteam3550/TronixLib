package TronixLib;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickTronix {

    private Joystick m_CurrentJoystick;

    //Ces variables peuvent soit être modifier manuellement ici ou avec les fonctions plus bas.
    //Par contre, je conseille de les modifier ici à chaque début de saison et seulement utiliser les fonctions en cas d'urgence
    private double m_deadbandXMinimumPositif = 0.4;
    private double m_deadbandXMinimumNegatif = -0.3;

    private double m_deadbandYMinimumPositif = 0.1;
    private double m_deadbandYMinimumNegatif = -0.10;

    private double m_deadbandZMinimumPositif = 0.4;
    private double m_deadbandZMinimumNegatif = -0.3;

    public JoystickTronix(int port) {
        m_CurrentJoystick = new Joystick(port);
    }

    //Only use these for emergencies (ex: on est à la compétition et on trouve un problème) or for testing values more easely
    public void setDeadbandX (double Positif, double Negatif) {
        m_deadbandXMinimumPositif = Positif;
        m_deadbandXMinimumNegatif = Negatif;
    }

    public void setDeadbandY (double Positif, double Negatif) {
        m_deadbandYMinimumPositif = Positif;
        m_deadbandYMinimumNegatif = Negatif;
    }

    public void setDeadbandZ (double Positif, double Negatif) {
        m_deadbandZMinimumPositif = Positif;
        m_deadbandZMinimumNegatif = Negatif;
    }

    public double getX() {
        double returnvalue = m_CurrentJoystick.getX();
        if ((returnvalue < m_deadbandXMinimumNegatif)||(returnvalue > m_deadbandXMinimumPositif)){ //Filtrer les valeurs trop petites
            return returnvalue;
        }
        else {
            return 0.0;
        }    
    }
    public double getY() {
        double returnvalue = m_CurrentJoystick.getY();
        if ((returnvalue < m_deadbandYMinimumNegatif)||(returnvalue > m_deadbandYMinimumPositif)){ //Filtrer les valeurs trop petites
            return returnvalue;
        }
        else {
            return 0.0;
        }
    }
    public double getZ() {
        double returnvalue = m_CurrentJoystick.getZ();
        if ((returnvalue < m_deadbandZMinimumNegatif)||(returnvalue > m_deadbandZMinimumPositif)){ //Filtrer les valeurs trop petites
            return returnvalue;
        }
        else {
            return 0.0;
        }
    }

}