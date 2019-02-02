package TronixLib;

import java.util.List;
import java.io.FilterOutputStream;
import java.io.BufferedOutputStream;
import java.io.PrintStream;
import java.io.FileOutputStream;
import java.io.IOException;


public class RecordDataTronix {

	//
	//  Exemple d'utilisation:
	//
	//  Declarer un array de data    private ArrayList<Double> somedatalist;
	//  ET un pour les timestamps    private ArrayList<Float>  timelist;
	//
	//  Les initialiser              somedatalist =  new ArrayList<>();
	//                               timelist     =  new ArrayList<>();
	//                       
	//  Declarer et instancier un    RecordDataTronix log = new RecordDataTronix
	//
	//  Le remplir de data dans AutonomousPeriodic ou TeleopPeriodic
	//                               timelist.add(((float)(System.nanoTime()))-startCounter);
	//             ET                somedatalist.add(newdataitem);
	//
	//  Quand les tableaux sont remplie les ecrire au fichier local (dans le robotRIO) dans le répertoire /home/lvuser/somedatafile.txt
	//                               log.recordDataToFile(timelist, somedatalist, "/home/lvuser/somedataTime.txt", "/home/lvuser/somedataList.txt");
	//

    public static void recordDataToFile(List<Float> dataTime,List<Double> data, String filename1,String filename2) {
    	try { 
    		//DataOutputStream dos= new DataOutputStream(new BufferedOutputStream(new FileOutputStream(filename1))); 
    		FilterOutputStream fileOutputStream1 = new FilterOutputStream(new BufferedOutputStream(new FileOutputStream(filename1)));
    		FilterOutputStream fileOutputStream = new FilterOutputStream(new BufferedOutputStream(new FileOutputStream(filename2)));
    		
    		PrintStream printStream1 = new PrintStream(fileOutputStream1);
    		PrintStream printStream = new PrintStream(fileOutputStream);
    		
    		for(int i=0;i<data.size();i++){ 
    		//System.out.println("before"+data.get(i)); 
    		printStream1.println(dataTime.get(i));
    		printStream1.println(" ");
    		printStream.println(data.get(i));
    		printStream.println(" ");
    		//dos.writeDouble(data.get(i));
    		//System.out.println("After"+data.get(i)); 
    		} 
    		printStream1.close();
    		printStream.close();
    		//dos.close(); 
    		
    		} catch (IOException e) { 
    			System.out.println("cant create newDatafile");
    		e.printStackTrace(); 
			}
		
        System.out.println("Data saved.");
    }
}