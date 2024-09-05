using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace Player.Core
{
    /// <summary>
    /// Represents the entire state of a PhysicsMover that is pertinent for simulation.
    /// Use this to save state or revert to past state
    /// </summary>
    [Serializable]
    public struct PhysicsMoverState
    {
        [FormerlySerializedAs("Position")] public Vector3 position;
        [FormerlySerializedAs("Rotation")] public Quaternion rotation;
        [FormerlySerializedAs("Velocity")] public Vector3 velocity;
        [FormerlySerializedAs("AngularVelocity")] public Vector3 angularVelocity;
    }

    /// <summary>
    /// Component that manages the movement of moving kinematic rigidbodies for
    /// proper interaction with characters
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class PhysicsMover : MonoBehaviour
    {
        /// <summary>
        /// The mover's Rigidbody
        /// </summary>
        [FormerlySerializedAs("Rigidbody")] [ReadOnly]
        public Rigidbody moverRigidbody;

        /// <summary>
        /// Determines if the platform moves with rigidbody.MovePosition (when true), or with rigidbody.position (when false)
        /// </summary>
        [FormerlySerializedAs("MoveWithPhysics")] public bool moveWithPhysics = true;

        /// <summary>
        /// Index of this motor in KinematicCharacterSystem arrays
        /// </summary>
        [NonSerialized]
        public IMoverController MoverController;
        /// <summary>
        /// Remembers latest position in interpolation
        /// </summary>
        [NonSerialized]
        public Vector3 LatestInterpolationPosition;
        /// <summary>
        /// Remembers latest rotation in interpolation
        /// </summary>
        [NonSerialized]
        public Quaternion LatestInterpolationRotation;
        /// <summary>
        /// The latest movement made by interpolation
        /// </summary>
        [NonSerialized]
        public Vector3 PositionDeltaFromInterpolation;
        /// <summary>
        /// The latest rotation made by interpolation
        /// </summary>
        [NonSerialized]
        public Quaternion RotationDeltaFromInterpolation;

        /// <summary>
        /// Index of this motor in KinematicCharacterSystem arrays
        /// </summary>
        public int IndexInCharacterSystem { get; set; }
        /// <summary>
        /// Remembers initial position before all simulation are done
        /// </summary>
        public Vector3 Velocity { get; private set; }
        /// <summary>
        /// Remembers initial position before all simulation are done
        /// </summary>
        public Vector3 AngularVelocity { get; private set; }
        /// <summary>
        /// Remembers initial position before all simulation are done
        /// </summary>
        public Vector3 InitialTickPosition { get; set; }
        /// <summary>
        /// Remembers initial rotation before all simulation are done
        /// </summary>
        public Quaternion InitialTickRotation { get; set; }

        /// <summary>
        /// The mover's Transform
        /// </summary>
        public Transform Transform { get; private set; }
        /// <summary>
        /// The character's position before the movement calculations began
        /// </summary>
        public Vector3 InitialSimulationPosition { get; private set; }
        /// <summary>
        /// The character's rotation before the movement calculations began
        /// </summary>
        public Quaternion InitialSimulationRotation { get; private set; }

        private Vector3 _internalTransientPosition;

        /// <summary>
        /// The mover's rotation (always up-to-date during the character update phase)
        /// </summary>
        public Vector3 TransientPosition
        {
            get => _internalTransientPosition;
            private set => _internalTransientPosition = value;
        }

        private Quaternion _internalTransientRotation;
        /// <summary>
        /// The mover's rotation (always up-to-date during the character update phase)
        /// </summary>
        public Quaternion TransientRotation
        {
            get => _internalTransientRotation;
            private set => _internalTransientRotation = value;
        }


        private void Reset()
        {
            ValidateData();
        }

        private void OnValidate()
        {
            ValidateData();
        }

        /// <summary>
        /// Handle validating all required values
        /// </summary>
        public void ValidateData()
        {
            moverRigidbody = gameObject.GetComponent<Rigidbody>();

            moverRigidbody.centerOfMass = Vector3.zero;
            moverRigidbody.maxAngularVelocity = Mathf.Infinity;
            moverRigidbody.maxDepenetrationVelocity = Mathf.Infinity;
            moverRigidbody.isKinematic = true;
            moverRigidbody.interpolation = RigidbodyInterpolation.None;
        }

        private void OnEnable()
        {
            KinematicCharacterSystem.EnsureCreation();
            KinematicCharacterSystem.RegisterPhysicsMover(this);
        }

        private void OnDisable()
        {
            KinematicCharacterSystem.UnregisterPhysicsMover(this);
        }

        private void Awake()
        {
            Transform = transform;
            ValidateData();

            TransientPosition = moverRigidbody.position;
            TransientRotation = moverRigidbody.rotation;
            InitialSimulationPosition = moverRigidbody.position;
            InitialSimulationRotation = moverRigidbody.rotation;
            LatestInterpolationPosition = Transform.position;
            LatestInterpolationRotation = Transform.rotation;
        }

        /// <summary>
        /// Sets the mover's position directly
        /// </summary>
        public void SetPosition(Vector3 position)
        {
            Transform.position = position;
            moverRigidbody.position = position;
            InitialSimulationPosition = position;
            TransientPosition = position;
        }

        /// <summary>
        /// Sets the mover's rotation directly
        /// </summary>
        public void SetRotation(Quaternion rotation)
        {
            Transform.rotation = rotation;
            moverRigidbody.rotation = rotation;
            InitialSimulationRotation = rotation;
            TransientRotation = rotation;
        }

        /// <summary>
        /// Sets the mover's position and rotation directly
        /// </summary>
        public void SetPositionAndRotation(Vector3 position, Quaternion rotation)
        {
            Transform.SetPositionAndRotation(position, rotation);
            moverRigidbody.position = position;
            moverRigidbody.rotation = rotation;
            InitialSimulationPosition = position;
            InitialSimulationRotation = rotation;
            TransientPosition = position;
            TransientRotation = rotation;
        }

        /// <summary>
        /// Returns all the state information of the mover that is pertinent for simulation
        /// </summary>
        public PhysicsMoverState GetState()
        {
            var state = new PhysicsMoverState
            {
                position = TransientPosition,
                rotation = TransientRotation,
                velocity = Velocity,
                angularVelocity = AngularVelocity
            };

            return state;
        }

        /// <summary>
        /// Applies a mover state instantly
        /// </summary>
        public void ApplyState(PhysicsMoverState state)
        {
            SetPositionAndRotation(state.position, state.rotation);
            Velocity = state.velocity;
            AngularVelocity = state.angularVelocity;
        }

        /// <summary>
        /// Caches velocity values based on deltatime and target position/rotations
        /// </summary>
        public void VelocityUpdate(float deltaTime)
        {
            InitialSimulationPosition = TransientPosition;
            InitialSimulationRotation = TransientRotation;

            MoverController.UpdateMovement(out _internalTransientPosition, out _internalTransientRotation, deltaTime);

            if (deltaTime > 0f)
            {
                Velocity = (TransientPosition - InitialSimulationPosition) / deltaTime;
                                
                var rotationFromCurrentToGoal = TransientRotation * (Quaternion.Inverse(InitialSimulationRotation));
                AngularVelocity = (Mathf.Deg2Rad * rotationFromCurrentToGoal.eulerAngles) / deltaTime;
            }
        }
    }
}