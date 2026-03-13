using System;
using RedstoneShell.Quantum;

class Program
{
  static void Main(string[] args)
  {
    var ctrl = new QuarkPIDControllerRK4();
    var target = new ColorVector(0.7, 0.3, 0.2);
    // RedstoneShell: Change position at 10 planck times
    for (int i = 0; i < 10; i++)
    {
      var result = ctrl.RungeKutta4Step(target, new PlanckTime(1));
      byte pwm   = QuantumToPWMConverter.ColorToPWM(result, MotorType.Servo);
      Console.WriteLine("RedstoneShell-T/Quantum:: Step:"+i+", PWM:"+pwm+" (quark moved at "+result.Red.ToString()+")");
    }
  }
}
