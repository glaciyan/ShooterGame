using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Player
{
    public class MyPlayer : MonoBehaviour
    {
        [Header("Camera")]
        public FirstPersonCamera playerCamera;
        public Transform cameraFollowPoint;
        public float lookSensitivity = 1f;
        public Vector2 VirtualViewAngle { get; private set; }
        private const float LookSensitivityReduction = 0.1f;

        private void Awake()
        {
            playerCamera.Follow = cameraFollowPoint;
        }

        private void OnLook(InputValue value)
        {
            // x -> left(negative)/right(positive)
            // y -> up/down
            var input = value.Get<Vector2>();
        
            var newAngle = VirtualViewAngle;

            newAngle.x += input.x * lookSensitivity * LookSensitivityReduction;
            // normalize yaw angle in range [0, 360]
            newAngle.x %= 360f;
            if (newAngle.x < 0) newAngle.x += 360f;

            // clamp y for looking down and up all the way
            newAngle.y += input.y * lookSensitivity * LookSensitivityReduction;
            newAngle.y = Mathf.Clamp(newAngle.y, -90f, 90f);

            VirtualViewAngle = newAngle;
            playerCamera.UpdateRotation(VirtualViewAngle);
        }
    }
}
