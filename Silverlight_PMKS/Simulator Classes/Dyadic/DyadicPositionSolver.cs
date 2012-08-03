using System;
using System.Collections.Generic;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;

namespace PlanarMechanismSimulator
{
    internal abstract class DyadicPositionSolver
    {

        #region set & find link position and angles
        protected void setLinkPositionFromRotate(joint knownJoint, int knownIndex, int linkIndex, 
            double[,] currentJointParams, double[,] oldJointParams, double[,] currentLinkParams, double[,] oldLinkParams, double delta = 0.0)
        {
            var thisLink = links[linkIndex];

            foreach (var j in thisLink.joints.Where(j => j != knownJoint && (unknownPositions.Contains(j) || j.FixedWithRespectToLink(thisLink))))
            {
                unknownPositions.Remove(j);
                var jIndex = joints.IndexOf(j);
                var length = thisLink.lengthBetween(j, knownJoint);
                var angle = Constants.angle(oldJointParams[knownIndex, 0], oldJointParams[knownIndex, 1],
                                             oldJointParams[jIndex, 0], oldJointParams[jIndex, 1]);
                angle += delta;
                currentJointParams[jIndex, 0] = currentJointParams[knownIndex, 0] + length * Math.Cos(angle);
                currentJointParams[jIndex, 1] = currentJointParams[knownIndex, 1] + length * Math.Sin(angle);
            }
            setLinkAngles(delta, thisLink, linkIndex, unknownLinkAngles, currentLinkParams, oldLinkParams, knownJoint);
        }
        private void setLinkAngles(double angleChange, link thisLink, int linkIndex, List<link> unknownLinkAngles,
            double[,] currentLinkParams, double[,] oldLinkParams, joint from = null)
        {
            var oldAngle = oldLinkParams[linkIndex, 0];
            currentLinkParams[linkIndex, 0] = oldAngle + angleChange;
            unknownLinkAngles.Remove(thisLink);
            foreach (var j in thisLink.joints)
            {
                if (j.jointType != JointTypes.P || j == from) continue;
                var newLink = j.OtherLink(thisLink);
                setLinkAngles(angleChange, newLink, links.IndexOf(newLink), unknownLinkAngles, currentLinkParams, oldLinkParams, j);
            }
        }
        private void setLinkPositionFromFixedTranslation(int linkIndex, ICollection<joint> unknownPositions,
           double[,] currentJointParams, double[,] oldJointParams, double deltaX, double deltaY)
        {
            var thisLink = links[linkIndex];

            foreach (var j in thisLink.joints.Where(j => unknownPositions.Contains(j) || j.FixedWithRespectToLink(thisLink)))
            {
                unknownPositions.Remove(j);
                var jIndex = joints.IndexOf(j);
                currentJointParams[jIndex, 0] = oldJointParams[jIndex, 0] + deltaX;
                currentJointParams[jIndex, 1] = oldJointParams[jIndex, 1] + deltaY;
            }
        }

        private void setLinkPositionFromSlideTranslation(int linkIndex, ICollection<joint> unknownPositions,
           double[,] currentJointParams, double[,] oldJointParams, double deltaX, double deltaY, double angle)
        {
            var thisLink = links[linkIndex];

            foreach (var j in thisLink.joints.Where(j => unknownPositions.Contains(j) || j.FixedWithRespectToLink(thisLink)))
            {
                unknownPositions.Remove(j);
                var jIndex = joints.IndexOf(j);
                currentJointParams[jIndex, 0] = oldJointParams[jIndex, 0] + deltaX * Math.Cos(angle);
                currentJointParams[jIndex, 1] = oldJointParams[jIndex, 1] + deltaY * Math.Sin(angle);
            }
        }
        #endregion
    }
}
