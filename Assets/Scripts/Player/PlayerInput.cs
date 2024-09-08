using System.Runtime.CompilerServices;
using UnityEngine;

namespace Player
{
    public struct PlayerInput
    {
        public Vector2 MovementInput;
        public Vector2 VirtualViewAngle;

        public void AddViewAngle(Vector2 delta, float sensitivity)
        {
            VirtualViewAngle.x += delta.x * sensitivity;
            // normalize yaw angle in range [0, 360]
            VirtualViewAngle.x %= 360f;
            if (VirtualViewAngle.x < 0) VirtualViewAngle.x += 360f;

            // clamp y for looking down and up all the way
            VirtualViewAngle.y += delta.y * sensitivity;
            VirtualViewAngle.y = Mathf.Clamp(VirtualViewAngle.y, -90f, 90f);
        }

        /// <summary>
        /// Calculates the requested movement direction based on the current movement input and view angle.
        /// The direction is relative to the player's yaw (horizontal rotation), effectively rotating
        /// the input vector by the yaw angle.
        /// </summary>
        /// <remarks>
        /// The yaw angle is derived from the X component of <see cref="VirtualViewAngle"/>.
        /// </remarks>
        /// <returns>A <see cref="Vector3"/> representing the movement direction in world space.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly Vector3 GetRequestedMovementDirection()
        {
            var yaw = Mathf.Deg2Rad * -VirtualViewAngle.x;

            // rotate input by yaw with rotation matrix
            return new Vector3(MovementInput.x * Mathf.Cos(yaw) - MovementInput.y * Mathf.Sin(yaw), 0f,
                MovementInput.x * Mathf.Sin(yaw) + MovementInput.y * Mathf.Cos(yaw));
        }

        /// <summary>
        /// Computes the line of sight (the direction the player is looking) based on the current view angle.
        /// The X component of <see cref="VirtualViewAngle"/> is used for yaw (horizontal rotation),
        /// and the Y component is used for pitch (vertical rotation).
        /// </summary>
        /// <remarks>
        /// This method calculates a 3D direction vector that can be used for ray-casting or determining the player's aim.
        /// </remarks>
        /// <returns>A <see cref="Vector3"/> representing the direction the player is looking in world space.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly Vector3 GetLineOfSight()
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
}