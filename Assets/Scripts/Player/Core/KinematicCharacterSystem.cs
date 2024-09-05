using System.Collections.Generic;
using UnityEngine;

namespace Player.Core
{
    /// <summary>
    /// The system that manages the simulation of KinematicCharacterMotor and PhysicsMover
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class KinematicCharacterSystem : MonoBehaviour
    {
        private static KinematicCharacterSystem _instance;

        public static readonly List<KinematicCharacterMotor> CharacterMotors = new();
        public static readonly List<PhysicsMover> PhysicsMovers = new();

        private static float _lastCustomInterpolationStartTime = -1f;
        private static float _lastCustomInterpolationDeltaTime = -1f;

        public static KccSettings Settings;

        /// <summary>
        /// Creates a KinematicCharacterSystem instance if there isn't already one
        /// </summary>
        public static void EnsureCreation()
        {
            if (_instance == null)
            {
                var systemGameObject = new GameObject("KinematicCharacterSystem");
                _instance = systemGameObject.AddComponent<KinematicCharacterSystem>();

                systemGameObject.hideFlags = HideFlags.NotEditable;
                _instance.hideFlags = HideFlags.NotEditable;

                Settings = ScriptableObject.CreateInstance<KccSettings>();

                GameObject.DontDestroyOnLoad(systemGameObject);
            }
        }

        /// <summary>
        /// Gets the KinematicCharacterSystem instance if any
        /// </summary>
        /// <returns></returns>
        public static KinematicCharacterSystem GetInstance()
        {
            return _instance;
        }

        /// <summary>
        /// Sets the maximum capacity of the character motors list, to prevent allocations when adding characters
        /// </summary>
        /// <param name="capacity"></param>
        public static void SetCharacterMotorsCapacity(int capacity)
        {
            if (capacity < CharacterMotors.Count)
            {
                capacity = CharacterMotors.Count;
            }
            CharacterMotors.Capacity = capacity;
        }

        /// <summary>
        /// Registers a KinematicCharacterMotor into the system
        /// </summary>
        public static void RegisterCharacterMotor(KinematicCharacterMotor motor)
        {
            CharacterMotors.Add(motor);
        }

        /// <summary>
        /// Unregisters a KinematicCharacterMotor from the system
        /// </summary>
        public static void UnregisterCharacterMotor(KinematicCharacterMotor motor)
        {
            CharacterMotors.Remove(motor);
        }

        /// <summary>
        /// Sets the maximum capacity of the physics movers list, to prevent allocations when adding movers
        /// </summary>
        /// <param name="capacity"></param>
        public static void SetPhysicsMoversCapacity(int capacity)
        {
            if (capacity < PhysicsMovers.Count)
            {
                capacity = PhysicsMovers.Count;
            }
            PhysicsMovers.Capacity = capacity;
        }

        /// <summary>
        /// Registers a PhysicsMover into the system
        /// </summary>
        public static void RegisterPhysicsMover(PhysicsMover mover)
        {
            PhysicsMovers.Add(mover);

            mover.moverRigidbody.interpolation = RigidbodyInterpolation.None;
        }

        /// <summary>
        /// Unregisters a PhysicsMover from the system
        /// </summary>
        public static void UnregisterPhysicsMover(PhysicsMover mover)
        {
            PhysicsMovers.Remove(mover);
        }

        // This is to prevent duplicating the singleton gameobject on script recompiles
        private void OnDisable()
        {
            Destroy(this.gameObject);
        }

        private void Awake()
        {
            _instance = this;
        }

        private void FixedUpdate()
        {
            if (Settings.autoSimulation)
            {
                var deltaTime = Time.deltaTime;

                if (Settings.interpolate)
                {
                    PreSimulationInterpolationUpdate(deltaTime);
                }

                Simulate(deltaTime, CharacterMotors, PhysicsMovers);

                if (Settings.interpolate)
                {
                    PostSimulationInterpolationUpdate(deltaTime);
                }
            }
        }

        private void LateUpdate()
        {
            if (Settings.interpolate)
            {
                CustomInterpolationUpdate();
            }
        }

        /// <summary>
        /// Remembers the point to interpolate from for KinematicCharacterMotors and PhysicsMovers
        /// </summary>
        private static void PreSimulationInterpolationUpdate(float deltaTime)
        {
            // Save pre-simulation poses and place transform at transient pose
            foreach (var motor in CharacterMotors)
            {
                motor.InitialTickPosition = motor.TransientPosition;
                motor.InitialTickRotation = motor.TransientRotation;

                motor.Transform.SetPositionAndRotation(motor.TransientPosition, motor.TransientRotation);
            }

            foreach (var mover in PhysicsMovers)
            {
                mover.InitialTickPosition = mover.TransientPosition;
                mover.InitialTickRotation = mover.TransientRotation;

                mover.Transform.SetPositionAndRotation(mover.TransientPosition, mover.TransientRotation);
                mover.moverRigidbody.position = mover.TransientPosition;
                mover.moverRigidbody.rotation = mover.TransientRotation;
            }
        }

        /// <summary>
        /// Ticks characters and/or movers
        /// </summary>
        private static void Simulate(float deltaTime, List<KinematicCharacterMotor> motors, List<PhysicsMover> movers)
        {
            var characterMotorsCount = motors.Count;
            var physicsMoversCount = movers.Count;

#pragma warning disable 0162
            // Update PhysicsMover velocities
            for (var i = 0; i < physicsMoversCount; i++)
            {
                movers[i].VelocityUpdate(deltaTime);
            }

            // Character controller update phase 1
            for (var i = 0; i < characterMotorsCount; i++)
            {
                motors[i].UpdatePhase1(deltaTime);
            }

            // Simulate PhysicsMover displacement
            for (var i = 0; i < physicsMoversCount; i++)
            {
                var mover = movers[i];

                mover.Transform.SetPositionAndRotation(mover.TransientPosition, mover.TransientRotation);
                mover.moverRigidbody.position = mover.TransientPosition;
                mover.moverRigidbody.rotation = mover.TransientRotation;
            }

            // Character controller update phase 2 and move
            for (var i = 0; i < characterMotorsCount; i++)
            {
                var motor = motors[i];

                motor.UpdatePhase2(deltaTime);

                motor.Transform.SetPositionAndRotation(motor.TransientPosition, motor.TransientRotation);
            }
#pragma warning restore 0162
        }

        /// <summary>
        /// Initiates the interpolation for KinematicCharacterMotors and PhysicsMovers
        /// </summary>
        private static void PostSimulationInterpolationUpdate(float deltaTime)
        {
            _lastCustomInterpolationStartTime = Time.time;
            _lastCustomInterpolationDeltaTime = deltaTime;

            // Return interpolated roots to their initial poses
            foreach (var motor in CharacterMotors)
            {
                motor.Transform.SetPositionAndRotation(motor.InitialTickPosition, motor.InitialTickRotation);
            }

            foreach (var mover in PhysicsMovers)
            {
                if (mover.moveWithPhysics)
                {
                    mover.moverRigidbody.position = mover.InitialTickPosition;
                    mover.moverRigidbody.rotation = mover.InitialTickRotation;

                    mover.moverRigidbody.MovePosition(mover.TransientPosition);
                    mover.moverRigidbody.MoveRotation(mover.TransientRotation);
                }
                else
                {
                    mover.moverRigidbody.position = (mover.TransientPosition);
                    mover.moverRigidbody.rotation = (mover.TransientRotation);
                }
            }
        }

        /// <summary>
        /// Handles per-frame interpolation
        /// </summary>
        private static void CustomInterpolationUpdate()
        {
            var interpolationFactor = Mathf.Clamp01((Time.time - _lastCustomInterpolationStartTime) / _lastCustomInterpolationDeltaTime);

            // Handle characters interpolation
            foreach (var motor in CharacterMotors)
            {
                motor.Transform.SetPositionAndRotation(
                    Vector3.Lerp(motor.InitialTickPosition, motor.TransientPosition, interpolationFactor),
                    Quaternion.Slerp(motor.InitialTickRotation, motor.TransientRotation, interpolationFactor));
            }

            // Handle PhysicsMovers interpolation
            foreach (var mover in PhysicsMovers)
            {
                mover.Transform.SetPositionAndRotation(
                    Vector3.Lerp(mover.InitialTickPosition, mover.TransientPosition, interpolationFactor),
                    Quaternion.Slerp(mover.InitialTickRotation, mover.TransientRotation, interpolationFactor));

                var newPos = mover.Transform.position;
                var newRot = mover.Transform.rotation;
                mover.PositionDeltaFromInterpolation = newPos - mover.LatestInterpolationPosition;
                mover.RotationDeltaFromInterpolation = Quaternion.Inverse(mover.LatestInterpolationRotation) * newRot;
                mover.LatestInterpolationPosition = newPos;
                mover.LatestInterpolationRotation = newRot;
            }
        }
    }
}