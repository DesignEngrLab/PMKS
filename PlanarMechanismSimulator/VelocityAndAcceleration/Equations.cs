using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PlanarMechanismSimulator.VelocityAndAcceleration
{
  public abstract  class EquationBase
    {
    }
  public class RelativeVelocityEquation : EquationBase
  {
      private joint joint1;
      private joint joint2;
      private link link;

      public RelativeVelocityEquation(joint joint1, joint joint2, link link)
      {
          // TODO: Complete member initialization
          this.joint1 = joint1;
          this.joint2 = joint2;
          this.link = link;
      }
  }
  public class EqualLinkVelocityEquation : EquationBase
  {
  }
}
