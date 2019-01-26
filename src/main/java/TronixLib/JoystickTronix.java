package TronixLib;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickTronix {

    private Joystick m_CurrentJoystick;

    //Ces variables peuvent soit être modifier manuellement ici ou avec les fonctions plus bas.
    //Par contre, je conseille de les modifier ici à chaque début de saison et seulement utiliser les fonctions en cas d'urgence
    private double DeadbandXMinimumPositif = 0.4;
    private double DeadbandXMinimumNegatif = -0.3;

    private double DeadbandYMinimumPositif = 0.1;
    private double DeadbandYMinimumNegatif = -0.10;

    private double DeadbandZMinimumPositif = 0.4;
    private double DeadbandZMinimumNegatif = -0.3;

    public JoystickTronix(int port) {
        m_CurrentJoystick = new Joystick(port);
    }

    //Only use these for emergencies (ex: on est à la compétition et on trouve un problème) or for testing values more easely
    public void SetDeadbandX (double Positif, double Negatif) {
        DeadbandXMinimumPositif = Positif;
        DeadbandXMinimumNegatif = Negatif;
    }

    public void SetDeadbandY (double Positif, double Negatif) {
        DeadbandYMinimumPositif = Positif;
        DeadbandYMinimumNegatif = Negatif;
    }

    public void SetDeadbandZ (double Positif, double Negatif) {
        DeadbandZMinimumPositif = Positif;
        DeadbandZMinimumNegatif = Negatif;
    }

    public double getX() {
        double returnvalue = m_CurrentJoystick.getX();
        if ((returnvalue < DeadbandXMinimumNegatif)||(returnvalue > DeadbandXMinimumPositif)){ //Filtrer les valeurs trop petites
            //if ((Math.abs(returnvalue) < 0.2)){
            return returnvalue;
        }
        else {
            return 0.0;
        }    
    }
    public double getY() {
        double returnvalue = m_CurrentJoystick.getY();
        if ((returnvalue < DeadbandYMinimumNegatif)||(returnvalue > DeadbandYMinimumPositif)){ //Filtrer les valeurs trop petites
            //if ((Math.abs(returnvalue) < 0.1)){
            return returnvalue;
        }
        else {
            return 0.0;
        }
    }
    public double getZ() {
        double returnvalue = m_CurrentJoystick.getZ();
        if ((returnvalue < DeadbandZMinimumNegatif)||(returnvalue > DeadbandZMinimumPositif)){ //Filtrer les valeurs trop petites
            //if ((Math.abs(returnvalue) < 0.2)){
            return returnvalue;
        }
        else {
            return 0.0;
        }
    }

}