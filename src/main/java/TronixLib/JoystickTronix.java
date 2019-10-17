package TronixLib;

import edu.wpi.first.wpilibj.Joystick;

/**
 * TODO : This
 */
/**
 * Exemple :
 * A state machine representing a complete action to be performed by the robot.  Commands are
 * run by the {@link CommandScheduler}, and can be composed into CommandGroups to allow users to
 * build complicated multi-step actions without the need to roll the state machine logic themselves.
 *
 * <p>Commands are run synchronously from the main robot loop; no multithreading is used, unless
 * specified explicitly from the command implementation.
 */
public class JoystickTronix {
    
    private Joystick m_CurrentJoystick;

    /**
     * [FR] Ces variables ont des valeurs par défauts, et sont le minimum qu'on doit bouger le
     *      joystick avant qu'on le prend en compte
     * [EN] These variables have defaults values, and are the minimum amount that we need to move
     *      the joystick to make the input count
     */
    private double m_deadbandXMinimumPositif = 0.4;
    private double m_deadbandXMinimumNegatif = -0.3;

    private double m_deadbandYMinimumPositif = 0.1;
    private double m_deadbandYMinimumNegatif = -0.10;

    private double m_deadbandZMinimumPositif = 0.4;
    private double m_deadbandZMinimumNegatif = -0.3;

    /**
     * [FR] Crée un object joystick avec le calcul du deadband automatique
     * [EN] Create a custom joystick object with automatic deadband control
     * @param port [FR] Le port du joystick / [EN] The port that the joystick is plugged in
     */
    public JoystickTronix(int port) { // TODO: Add support for an xbox gamepad and a normal gamepad, either with a different class or inside the same class
        m_CurrentJoystick = new Joystick(port);
    }
    
    /**
     * [FR] Change le deadband du joystick dans l'axe des X
     * [EN] Changes the deadband of the joystick in the X axis
     * @param Positif [FR] Le minimum dans les positifs / [EN] The minimum in the positives
     * @param Negatif [FR] Le minimum dans les négatifs / [EN] The minimum in the negatives
     */
    public void setDeadbandX (double Positif, double Negatif) {
        m_deadbandXMinimumPositif = Positif;
        m_deadbandXMinimumNegatif = Negatif;
    }

    /**
     * [FR] Change le deadband du joystick dans l'axe des Y
     * [EN] Changes the deadband of the joystick in the Y axis
     * @param Positif [FR] Le minimum dans les positifs / [EN] The minimum in the positives
     * @param Negatif [FR] Le minimum dans les négatifs / [EN] The minimum in the negatives
     */
    public void setDeadbandY (double Positif, double Negatif) {
        m_deadbandYMinimumPositif = Positif;
        m_deadbandYMinimumNegatif = Negatif;
    }

    /**
     * [FR] Change le deadband du joystick dans l'axe des Z
     * [EN] Changes the deadband of the joystick in the Z axis
     * @param Positif [FR] Le minimum dans les positifs / [EN] The minimum in the positives
     * @param Negatif [FR] Le minimum dans les négatifs / [EN] The minimum in the negatives
     */
    public void setDeadbandZ (double Positif, double Negatif) {
        m_deadbandZMinimumPositif = Positif;
        m_deadbandZMinimumNegatif = Negatif;
    }

    /**
     * [FR] Retourne la valeur dans l'axis X du joystick, en comptant le deadband
     * [EN] Returns the X axis value of the joystick, after the deadband calculations
     * @return {@link double} [FR] L'axis X du joystick / [EN] The X axis of the joystick
     */
    public double getX() {
        double returnvalue = m_CurrentJoystick.getX();
        if ((returnvalue < m_deadbandXMinimumNegatif)||(returnvalue > m_deadbandXMinimumPositif)){ //Filtrer les valeurs trop petites
            return returnvalue;
        }
        else {
            return 0.0;
        }    
    }

    /**
     * [FR] Retourne la valeur dans l'axis Y du joystick, en comptant le deadband
     * [EN] Returns the Y axis value of the joystick, after the deadband calculations
     * @return {@link double} [FR] L'axis Y du joystick / [EN] The Y axis of the joystick
     */
    public double getY() {
        double returnvalue = m_CurrentJoystick.getY();
        if ((returnvalue < m_deadbandYMinimumNegatif)||(returnvalue > m_deadbandYMinimumPositif)){ //Filtrer les valeurs trop petites
            return returnvalue;
        }
        else {
            return 0.0;
        }
    }

    /**
     * [FR] Retourne la valeur dans l'axis Z du joystick, en comptant le deadband
     * [EN] Returns the X axis value of the joystick, after the deadband calculations
     * @return {@link double} [FR] L'axis X du joystick / [EN] The Z axis of the joystick
     */
    public double getZ() {
        double returnvalue = m_CurrentJoystick.getZ();
        if ((returnvalue < m_deadbandZMinimumNegatif)||(returnvalue > m_deadbandZMinimumPositif)){ //Filtrer les valeurs trop petites
            return returnvalue;
        }
        else {
            return 0.0;
        }
    }

    /**
     * [FR] Retourne l'object du Joystick afin de pouvoir l'utiliser directement
     * [EN] Returns the Joystick object in the goal to access it direcly
     * @return {@link Joystick} [FR] Retourne l'object WPILib Joystick / [EN] Returns the WPILib Joystick object
     */
    public Joystick getJoystick() {
        return m_CurrentJoystick;
    }

}