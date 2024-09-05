using UnityEditor;
using UnityEngine;

namespace Player.Core.Editor
{
    [CustomEditor(typeof(KinematicCharacterMotor))]
    public sealed class KinematicCharacterMotorEditor : UnityEditor.Editor
    {
        private void OnSceneGUI()
        {            
            var motor = (target as KinematicCharacterMotor);
            if (motor)
            {
                var characterBottom = motor.transform.position + (motor.capsule.center + (-Vector3.up * (motor.capsule.height * 0.5f)));

                Handles.color = Color.yellow;
                Handles.CircleHandleCap(
                    0, 
                    characterBottom + (motor.transform.up * motor.maxStepHeight), 
                    Quaternion.LookRotation(motor.transform.up, motor.transform.forward), 
                    motor.capsule.radius + 0.1f, 
                    EventType.Repaint);
            }
        }
    }
}