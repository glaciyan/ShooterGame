using UnityEngine;
using UnityEngine.InputSystem;

namespace Player
{
    public class PlayerInputHandler : MonoBehaviour
    {
        public Camera playerCamera;

        private Vector2 _movementInput;

        private void OnMove(InputValue value)
        {
            // x -> left/right
            // y -> forward/back
            _movementInput = value.Get<Vector2>();
        }

        public Vector3 RequestedMovementDirection
        {
            get
            {
                var yaw = Mathf.Deg2Rad * -VirtualViewAngle.x;

                // rotate input by yaw with rotation matrix
                return new Vector3(_movementInput.x * Mathf.Cos(yaw) - _movementInput.y * Mathf.Sin(yaw), 0f,
                    _movementInput.x * Mathf.Sin(yaw) + _movementInput.y * Mathf.Cos(yaw));
            }
        }

        /// <summary>
        /// In Degrees, the view the player has input using their mouse.
        /// The X Component represents yaw.
        /// The Y Component represents pitch.
        /// </summary>
        public Vector2 VirtualViewAngle { get; private set; } = Vector2.zero;

        public Vector3 LineOfSight
        {
            get
            {
                var yawRadians = Mathf.Deg2Rad * VirtualViewAngle.x;
                var pitchRadians = Mathf.Deg2Rad * VirtualViewAngle.y;

                // Precompute cos and sin of pitch
                var cosPitch = Mathf.Cos(pitchRadians);
                var sinPitch = Mathf.Sin(pitchRadians);

                // Precompute sin and cos of yaw
                var sinYaw = Mathf.Sin(yawRadians);
                var cosYaw = Mathf.Cos(yawRadians);

                // Calculate aiming vector (including Y component)
                return new Vector3(
                    cosPitch * sinYaw,
                    sinPitch,
                    cosPitch * cosYaw
                );
            }
        }

        // public Vector3 Forward
        // {
        //     get
        //     {
        //         var yawRadians = Mathf.Deg2Rad * VirtualViewAngle.x;
        //
        //         // Calculate the forward vector on the XZ plane
        //         var x = Mathf.Sin(yawRadians);
        //         var z = Mathf.Cos(yawRadians);
        //
        //         // Create the forward vector and normalize it
        //         return new Vector3(x, 0, z).normalized;
        //     }
        // }
        //
        // public Vector3 Right
        // {
        //     get
        //     {
        //         var forward = Forward;
        //         return new Vector3(forward.z, 0, -forward.x).normalized;
        //     }
        // }

        public float lookSensitivity = 1f;

        private const float LookSensitivityReduction = 0.1f;

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
        }

        private bool _requestedJump;

        private void OnJump()
        {
            _requestedJump = true;
        }

        public bool ConsumeJump()
        {
            if (!_requestedJump) return false;

            _requestedJump = false;
            return true;
        }

        private void Start()
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }

        private void Update()
        {
            playerCamera.transform.rotation =
                Quaternion.Euler(new Vector3(-VirtualViewAngle.y, VirtualViewAngle.x, 0f));
        }
    }
}