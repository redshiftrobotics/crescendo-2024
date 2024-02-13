public class Deadzone {
	public static double joyDeadzone(double joyDeadzone, double joyValue){
		return (1/(1 - joyDeadzone)) * (joyValue - joyDeadzone);
	}

	public static double rangeChange(double joyDeadzoneInput, double joyDeadzoneLimit, int lowerJoyLimitValue, int upperJoyLimitValue){
		double joyDeadzoneOutput;
		if (joyDeadzoneInput <= joyDeadzoneLimit) {
			joyDeadzoneOutput = 0;
		}
		return (1/upperJoyLimitValue - joyDeadzoneLimit * (joyDeadzoneInput - upperJoyLimitValue));
	}
	
	public static void main(String args[]){
		double abc = Deadzone.rangeChange(0.5, 0.3, 0, 1);
		System.out.println(abc);

	}
}


//Unused functions. Use if necessary. Still need work.