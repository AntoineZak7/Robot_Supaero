
import java.util.ArrayList;

import lejos.*;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.motor.*;
import lejos.hardware.motor.*;
import java.lang.Math;

public class Robot {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		
		// Construction Chassis
		Wheel Roue_D = WheeledChassis.modelWheel(Motor.D, 5.6).offset(7.85);
		Wheel Roue_G = WheeledChassis.modelWheel(Motor.A, 5.6).offset(-7.85);
		Chassis chassis_robot = new WheeledChassis(new Wheel[] {Roue_D, Roue_G}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot robot = new MovePilot(chassis_robot);
		
		// Sensors (US + Gyro) (Obsolète quand tests finis)
		EV3UltrasonicSensor US = new EV3UltrasonicSensor(SensorPort.S2);
		US.enable();
		SampleProvider usProvider = US.getDistanceMode();
		EV3GyroSensor Gyro = new EV3GyroSensor(SensorPort.S1);
		Gyro.reset(); // Calibration gyro POUR TESTS UNIQUEMENT
		Delay.msDelay(1000);
		SampleProvider gyroProvider = Gyro.getAngleMode();
		
		//Moteur Fourche
		EV3MediumRegulatedMotor moteur_fourche = new EV3MediumRegulatedMotor(MotorPort.B);
		moteur_fourche.setSpeed(5);
		moteur_fourche.setAcceleration(2);
		
		
		float[] distances = new float[20];
		float[] angles = new float[2];
		int s = 0;
		
		// Réglages distances
		float long_bras;
		float dist_min_angles;
		float long_approach;
		int angle_fourche_bas;
		int angle_fouche_stockage;
		
		long_bras = (float) 0.08;
		dist_min_angles = long_bras + (float) 0.06; 
		long_approach = dist_min_angles - long_bras;
		
		angle_fourche_bas = -30;
		angle_fouche_stockage = +30;
		
		
		
		// Réglages accélération robot
		robot.setLinearAcceleration(3);
		robot.setLinearSpeed(3);
		
		robot.setAngularSpeed(3);
		robot.setAngularAcceleration(3);
		robot.setLinearSpeed(2);
		
		//Approche jusqu'a distance pour mesure angles (rajouter cas ou robot perd balle de visu)
		usProvider.fetchSample(distances, s);
		s +=1;
		float travel_dist = (distances[s] - dist_min_angles)*100;
		robot.travel(travel_dist); // Approche jusqu'à 10 cm
		

		// CALCUL MEILLEUR ANGLE
		
		int i = 0;

		
		while (i < 2) {
			
			if (i == 0) {
			Delay.msDelay(1200);
			usProvider.fetchSample(distances, s);
			s +=1;
			
				if (distances[s] > dist_min_angles + 0.1) {
					i += 1;
					gyroProvider.fetchSample(angles, 0);
				} 
				
				else { robot.rotate(3);}
				
			if ( i == 1) {
			Delay.msDelay(1200);
			usProvider.fetchSample(distances, s);
			s += 1;
			
				if (distances[s] > dist_min_angles + 0.1) {
				i += 1;
				gyroProvider.fetchSample(angles, 1);
				
			} 
			else { robot.rotate(-3);}		
				
		}
			
			
		// Alignement avec balle
		robot.setAngularSpeed(2);
		robot.setAngularAcceleration(2);
		float angle_diff = (angles[0] - angles[1])/2;
		robot.rotate(Math.abs(angle_diff));
		
		
		
		// APPROCHE et BAISSER FOURCHE (PAramètres)
		robot.setLinearAcceleration(2);
		robot.setLinearSpeed(2);
		usProvider.fetchSample(distances, s);
		s += 1;
		travel_dist = (distances[s] - long_bras)*100;
		
		 
		// Approche et baisser fourche (mouvement)
		moteur_fourche.setSpeed(10);
		moteur_fourche.setAcceleration(5);
		moteur_fourche.rotate(angle_fourche_bas, true);
		robot.travel(travel_dist);
		Delay.msDelay(500);
		
		//Remonter Fourche
		moteur_fourche.setSpeed(5);
		moteur_fourche.setAcceleration(2);
		moteur_fourche.rotate(angle_fouche_stockage);
		
		// Marche arrière
		robot.setLinearAcceleration(3);
		robot.setLinearSpeed(5);
		robot.travel(-15);

		
		
		
		// Check return value of sensor for infinity
		//Réglages disatnces + positions haut et bas fourche
		// Tests accélération
		
		
		

		
		
		
			
			
			
			
			
			
		
	  }
	}
		
		
		
		
		
		
		
		
		
		
		
	}

}
