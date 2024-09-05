using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

namespace Player.Core
{
    public enum RigidbodyInteractionType
    {
        None,
        Kinematic,
        SimulatedDynamic
    }

    public enum StepHandlingMethod
    {
        None,
        Standard,
        Extra
    }

    public enum MovementSweepState
    {
        Initial,
        AfterFirstHit,
        FoundBlockingCrease,
        FoundBlockingCorner,
    }

    /// <summary>
    /// Represents the entire state of a character motor that is pertinent for simulation.
    /// Use this to save state or revert to past state
    /// </summary>
    [Serializable]
    public struct KinematicCharacterMotorState
    {
        [FormerlySerializedAs("Position")] public Vector3 position;
        [FormerlySerializedAs("Rotation")] public Quaternion rotation;
        [FormerlySerializedAs("BaseVelocity")] public Vector3 baseVelocity;

        [FormerlySerializedAs("MustUnground")] public bool mustUnground;
        [FormerlySerializedAs("MustUngroundTime")] public float mustUngroundTime;
        [FormerlySerializedAs("LastMovementIterationFoundAnyGround")] public bool lastMovementIterationFoundAnyGround;
        public CharacterTransientGroundingReport GroundingStatus;

        [FormerlySerializedAs("AttachedRigidbody")] public Rigidbody attachedRigidbody;
        [FormerlySerializedAs("AttachedRigidbodyVelocity")] public Vector3 attachedRigidbodyVelocity;
    }

    /// <summary>
    /// Describes an overlap between the character capsule and another collider
    /// </summary>
    public struct OverlapResult
    {
        public Vector3 Normal;
        public Collider Collider;

        public OverlapResult(Vector3 normal, Collider collider)
        {
            Normal = normal;
            Collider = collider;
        }
    }

    /// <summary>
    /// Contains all the information for the motor's grounding status
    /// </summary>
    public struct CharacterGroundingReport
    {
        public bool FoundAnyGround;
        public bool IsStableOnGround;
        public bool SnappingPrevented;
        public Vector3 GroundNormal;
        public Vector3 InnerGroundNormal;
        public Vector3 OuterGroundNormal;

        public Collider GroundCollider;
        public Vector3 GroundPoint;

        public void CopyFrom(CharacterTransientGroundingReport transientGroundingReport)
        {
            FoundAnyGround = transientGroundingReport.FoundAnyGround;
            IsStableOnGround = transientGroundingReport.IsStableOnGround;
            SnappingPrevented = transientGroundingReport.SnappingPrevented;
            GroundNormal = transientGroundingReport.GroundNormal;
            InnerGroundNormal = transientGroundingReport.InnerGroundNormal;
            OuterGroundNormal = transientGroundingReport.OuterGroundNormal;

            GroundCollider = null;
            GroundPoint = Vector3.zero;
        }
    }

    /// <summary>
    /// Contains the simulation-relevant information for the motor's grounding status
    /// </summary>
    public struct CharacterTransientGroundingReport
    {
        public bool FoundAnyGround;
        public bool IsStableOnGround;
        public bool SnappingPrevented;
        public Vector3 GroundNormal;
        public Vector3 InnerGroundNormal;
        public Vector3 OuterGroundNormal;

        public void CopyFrom(CharacterGroundingReport groundingReport)
        {
            FoundAnyGround = groundingReport.FoundAnyGround;
            IsStableOnGround = groundingReport.IsStableOnGround;
            SnappingPrevented = groundingReport.SnappingPrevented;
            GroundNormal = groundingReport.GroundNormal;
            InnerGroundNormal = groundingReport.InnerGroundNormal;
            OuterGroundNormal = groundingReport.OuterGroundNormal;
        }
    }

    /// <summary>
    /// Contains all the information from a hit stability evaluation
    /// </summary>
    public struct HitStabilityReport
    {
        public bool IsStable;

        public bool FoundInnerNormal;
        public Vector3 InnerNormal;
        public bool FoundOuterNormal;
        public Vector3 OuterNormal;

        public bool ValidStepDetected;
        public Collider SteppedCollider;

        public bool LedgeDetected;
        public bool IsOnEmptySideOfLedge;
        public float DistanceFromLedge;
        public bool IsMovingTowardsEmptySideOfLedge;
        public Vector3 LedgeGroundNormal;
        public Vector3 LedgeRightDirection;
        public Vector3 LedgeFacingDirection;
    }

    /// <summary>
    /// Contains the information of hit rigidbodies during the movement phase, so they can be processed afterwards
    /// </summary>
    public struct RigidbodyProjectionHit
    {
        public Rigidbody Rigidbody;
        public Vector3 HitPoint;
        public Vector3 EffectiveHitNormal;
        public Vector3 HitVelocity;
        public bool StableOnHit;
    }

    /// <summary>
    /// Component that manages character collisions and movement solving
    /// </summary>
    [RequireComponent(typeof(CapsuleCollider))]
    public sealed class KinematicCharacterMotor : MonoBehaviour
    {
#pragma warning disable 0414
        [FormerlySerializedAs("Capsule")]
        [Header("Components")]
        [ReadOnly]
        public CapsuleCollider capsule;

        [FormerlySerializedAs("CapsuleRadius")]
        [Header("Capsule Settings")]
        [SerializeField]
        [Tooltip("Radius of the Character Capsule")]
        private float capsuleRadius = 0.5f;
        /// <summary>
        /// Height of the character's capsule
        /// </summary>
        [FormerlySerializedAs("CapsuleHeight")]
        [SerializeField]
        [Tooltip("Height of the Character Capsule")]
        private float capsuleHeight = 2f;
        /// <summary>
        /// Local y position of the character's capsule center
        /// </summary>
        [FormerlySerializedAs("CapsuleYOffset")]
        [SerializeField]
        [Tooltip("Height of the Character Capsule")]
        private float capsuleYOffset = 1f;
        /// <summary>
        /// Physics material of the character's capsule
        /// </summary>
        [FormerlySerializedAs("CapsulePhysicsMaterial")]
        [SerializeField]
        [Tooltip("Physics material of the Character Capsule (Does not affect character movement. Only affects things colliding with it)")]
#pragma warning disable 0649
        private PhysicsMaterial capsulePhysicsMaterial;
#pragma warning restore 0649


        [FormerlySerializedAs("GroundDetectionExtraDistance")]
        [Header("Grounding settings")]
        [Tooltip("Increases the range of ground detection, to allow snapping to ground at very high speeds")]
        public float groundDetectionExtraDistance;
        /// <summary>
        /// Maximum slope angle on which the character can be stable
        /// </summary>    
        [FormerlySerializedAs("MaxStableSlopeAngle")]
        [Range(0f, 89f)]
        [Tooltip("Maximum slope angle on which the character can be stable")]
        public float maxStableSlopeAngle = 60f;
        /// <summary>
        /// Which layers can the character be considered stable on
        /// </summary>    
        [FormerlySerializedAs("StableGroundLayers")] [Tooltip("Which layers can the character be considered stable on")]
        public LayerMask stableGroundLayers = -1;
        /// <summary>
        /// Notifies the Character Controller when discrete collisions are detected
        /// </summary>    
        [FormerlySerializedAs("DiscreteCollisionEvents")] [Tooltip("Notifies the Character Controller when discrete collisions are detected")]
        public bool discreteCollisionEvents;


        [FormerlySerializedAs("StepHandling")]
        [Header("Step settings")]
        [Tooltip("Handles properly detecting grounding status on steps, but has a performance cost.")]
        public StepHandlingMethod stepHandling = StepHandlingMethod.Standard;
        /// <summary>
        /// Maximum height of a step which the character can climb
        /// </summary>    
        [FormerlySerializedAs("MaxStepHeight")] [Tooltip("Maximum height of a step which the character can climb")]
        public float maxStepHeight = 0.5f;
        /// <summary>
        /// Can the character step up obstacles even if it is not currently stable?
        /// </summary>    
        [FormerlySerializedAs("AllowSteppingWithoutStableGrounding")] [Tooltip("Can the character step up obstacles even if it is not currently stable?")]
        public bool allowSteppingWithoutStableGrounding;
        /// <summary>
        /// Minimum length of a step that the character can step on (used in Extra stepping method. Use this to let the character step on steps that are smaller that its radius
        /// </summary>    
        [FormerlySerializedAs("MinRequiredStepDepth")] [Tooltip("Minimum length of a step that the character can step on (used in Extra stepping method). Use this to let the character step on steps that are smaller that its radius")]
        public float minRequiredStepDepth = 0.1f;


        [FormerlySerializedAs("LedgeAndDenivelationHandling")]
        [Header("Ledge settings")]
        [Tooltip("Handles properly detecting ledge information and grounding status, but has a performance cost.")]
        public bool ledgeAndDenivelationHandling = true;
        /// <summary>
        /// The distance from the capsule central axis at which the character can stand on a ledge and still be stable
        /// </summary>    
        [FormerlySerializedAs("MaxStableDistanceFromLedge")] [Tooltip("The distance from the capsule central axis at which the character can stand on a ledge and still be stable")]
        public float maxStableDistanceFromLedge = 0.5f;
        /// <summary>
        /// Prevents snapping to ground on ledges beyond a certain velocity
        /// </summary>    
        [FormerlySerializedAs("MaxVelocityForLedgeSnap")] [Tooltip("Prevents snapping to ground on ledges beyond a certain velocity")]
        public float maxVelocityForLedgeSnap;
        /// <summary>
        /// The maximun downward slope angle change that the character can be subjected to and still be snapping to the ground
        /// </summary>    
        [FormerlySerializedAs("MaxStableDenivelationAngle")]
        [Tooltip("The maximun downward slope angle change that the character can be subjected to and still be snapping to the ground")]
        [Range(1f, 180f)]
        public float maxStableDenivelationAngle = 180f;


        [FormerlySerializedAs("InteractiveRigidbodyHandling")]
        [Header("Rigidbody interaction settings")]
        [Tooltip("Handles properly being pushed by and standing on PhysicsMovers or dynamic rigidbodies. Also handles pushing dynamic rigidbodies")]
        public bool interactiveRigidbodyHandling = true;
        /// <summary>
        /// How the character interacts with non-kinematic rigidbodies. \"Kinematic\" mode means the character pushes the rigidbodies with infinite force (as a kinematic body would). \"SimulatedDynamic\" pushes the rigidbodies with a simulated mass value.
        /// </summary>
        [FormerlySerializedAs("RigidbodyInteractionType")] [Tooltip("How the character interacts with non-kinematic rigidbodies. \"Kinematic\" mode means the character pushes the rigidbodies with infinite force (as a kinematic body would). \"SimulatedDynamic\" pushes the rigidbodies with a simulated mass value.")]
        public RigidbodyInteractionType rigidbodyInteractionType;
        [FormerlySerializedAs("SimulatedCharacterMass")] [Tooltip("Mass used for pushing bodies")]
        public float simulatedCharacterMass = 1f;
        /// <summary>
        /// Determines if the character preserves moving platform velocities when de-grounding from them
        /// </summary>
        [FormerlySerializedAs("PreserveAttachedRigidbodyMomentum")] [Tooltip("Determines if the character preserves moving platform velocities when de-grounding from them")]
        public bool preserveAttachedRigidbodyMomentum = true;


        [FormerlySerializedAs("HasPlanarConstraint")]
        [Header("Constraints settings")]
        [Tooltip("Determines if the character's movement uses the planar constraint")]
        public bool hasPlanarConstraint;
        /// <summary>
        /// Defines the plane that the character's movement is constrained on, if HasMovementConstraintPlane is active
        /// </summary>
        [FormerlySerializedAs("PlanarConstraintAxis")] [Tooltip("Defines the plane that the character's movement is constrained on, if HasMovementConstraintPlane is active")]
        public Vector3 planarConstraintAxis = Vector3.forward;

        [FormerlySerializedAs("MaxMovementIterations")]
        [Header("Other settings")]
        [Tooltip("How many times can we sweep for movement per update")]
        public int maxMovementIterations = 5;
        /// <summary>
        /// How many times can we check for decollision per update
        /// </summary>
        [FormerlySerializedAs("MaxDecollisionIterations")] [Tooltip("How many times can we check for decollision per update")]
        public int maxDecollisionIterations = 1;
        /// <summary>
        /// Checks for overlaps before casting movement, making sure all collisions are detected even when already intersecting geometry (has a performance cost, but provides safety against tunneling through colliders)
        /// </summary>
        [FormerlySerializedAs("CheckMovementInitialOverlaps")] [Tooltip("Checks for overlaps before casting movement, making sure all collisions are detected even when already intersecting geometry (has a performance cost, but provides safety against tunneling through colliders)")]
        public bool checkMovementInitialOverlaps = true;
        /// <summary>
        /// Sets the velocity to zero if exceed max movement iterations
        /// </summary>
        [FormerlySerializedAs("KillVelocityWhenExceedMaxMovementIterations")] [Tooltip("Sets the velocity to zero if exceed max movement iterations")]
        public bool killVelocityWhenExceedMaxMovementIterations = true;
        /// <summary>
        /// Sets the remaining movement to zero if exceed max movement iterations
        /// </summary>
        [FormerlySerializedAs("KillRemainingMovementWhenExceedMaxMovementIterations")] [Tooltip("Sets the remaining movement to zero if exceed max movement iterations")]
        public bool killRemainingMovementWhenExceedMaxMovementIterations = true;

        /// <summary>
        /// Contains the current grounding information
        /// </summary>
        [NonSerialized] private CharacterGroundingReport _groundingStatus;
        /// <summary>
        /// Contains the previous grounding information
        /// </summary>
        [NonSerialized] private CharacterTransientGroundingReport _lastGroundingStatus;
        /// <summary>
        /// Specifies the LayerMask that the character's movement algorithm can detect collisions with. By default, this uses the rigidbody's layer's collision matrix
        /// </summary>
        [NonSerialized] private LayerMask _collidableLayers = -1;

        /// <summary>
        /// The Transform of the character motor
        /// </summary>
        public Transform Transform { get; private set; }

        /// <summary>
        /// The character's goal position in its movement calculations (always up-to-date during the character update phase)
        /// </summary>
        public Vector3 TransientPosition => _transientPosition;

        private Vector3 _transientPosition;
        /// <summary>
        /// The character's up direction (always up-to-date during the character update phase)
        /// </summary>
        private Vector3 CharacterUp { get; set; }

        /// <summary>
        /// The character's forward direction (always up-to-date during the character update phase)
        /// </summary>
        public Vector3 CharacterForward => _characterForward;

        private Vector3 _characterForward;
        /// <summary>
        /// The character's right direction (always up-to-date during the character update phase)
        /// </summary>
        public Vector3 CharacterRight => _characterRight;

        private Vector3 _characterRight;
        /// <summary>
        /// The character's position before the movement calculations began
        /// </summary>
        public Vector3 InitialSimulationPosition => _initialSimulationPosition;

        private Vector3 _initialSimulationPosition;
        /// <summary>
        /// The character's rotation before the movement calculations began
        /// </summary>
        public Quaternion InitialSimulationRotation => _initialSimulationRotation;

        private Quaternion _initialSimulationRotation;
        /// <summary>
        /// Represents the Rigidbody to stay attached to
        /// </summary>
        public Rigidbody AttachedRigidbody => _attachedRigidbody;

        private Rigidbody _attachedRigidbody;
        /// <summary>
        /// Vector3 from the character transform position to the capsule center
        /// </summary>
        public Vector3 CharacterTransformToCapsuleCenter => _characterTransformToCapsuleCenter;

        private Vector3 _characterTransformToCapsuleCenter;
        /// <summary>
        /// Vector3 from the character transform position to the capsule bottom
        /// </summary>
        public Vector3 CharacterTransformToCapsuleBottom => _characterTransformToCapsuleBottom;

        private Vector3 _characterTransformToCapsuleBottom;
        /// <summary>
        /// Vector3 from the character transform position to the capsule top
        /// </summary>
        public Vector3 CharacterTransformToCapsuleTop => _characterTransformToCapsuleTop;

        private Vector3 _characterTransformToCapsuleTop;
        /// <summary>
        /// Vector3 from the character transform position to the capsule bottom hemi center
        /// </summary>
        public Vector3 CharacterTransformToCapsuleBottomHemi => _characterTransformToCapsuleBottomHemi;

        private Vector3 _characterTransformToCapsuleBottomHemi;
        /// <summary>
        /// Vector3 from the character transform position to the capsule top hemi center
        /// </summary>
        public Vector3 CharacterTransformToCapsuleTopHemi => _characterTransformToCapsuleTopHemi;

        private Vector3 _characterTransformToCapsuleTopHemi;
        /// <summary>
        /// The character's velocity resulting from standing on rigidbodies or PhysicsMover
        /// </summary>
        public Vector3 AttachedRigidbodyVelocity => _attachedRigidbodyVelocity;

        private Vector3 _attachedRigidbodyVelocity;
        /// <summary>
        /// The number of overlaps detected so far during character update (is reset at the beginning of the update)
        /// </summary>
        public int OverlapsCount => _overlapsCount;

        private int _overlapsCount;
        /// <summary>
        /// The overlaps detected so far during character update
        /// </summary>
        public OverlapResult[] Overlaps => _overlaps;

        private readonly OverlapResult[] _overlaps = new OverlapResult[MaxRigidbodyOverlapsCount];

        /// <summary>
        /// The motor's assigned controller
        /// </summary>
        [NonSerialized]
        public ICharacterController CharacterController;
        /// <summary>
        /// Did the motor's last swept collision detection find a ground?
        /// </summary>
        [NonSerialized]
        public bool LastMovementIterationFoundAnyGround;
        /// <summary>
        /// Index of this motor in KinematicCharacterSystem arrays
        /// </summary>
        [NonSerialized]
        public int IndexInCharacterSystem;
        /// <summary>
        /// Remembers initial position before all simulation are done
        /// </summary>
        [NonSerialized]
        public Vector3 InitialTickPosition;
        /// <summary>
        /// Remembers initial rotation before all simulation are done
        /// </summary>
        [NonSerialized]
        public Quaternion InitialTickRotation;
        /// <summary>
        /// Specifies a Rigidbody to stay attached to
        /// </summary>
        [NonSerialized]
        public Rigidbody AttachedRigidbodyOverride;
        /// <summary>
        /// The character's velocity resulting from direct movement
        /// </summary>
        [NonSerialized]
        public Vector3 BaseVelocity;

        // Private
        private readonly RaycastHit[] _internalCharacterHits = new RaycastHit[MaxHitsBudget];
        private readonly Collider[] _internalProbedColliders = new Collider[MaxCollisionBudget];
        private readonly List<Rigidbody> _rigidbodiesPushedThisMove = new(16);
        private readonly RigidbodyProjectionHit[] _internalRigidbodyProjectionHits = new RigidbodyProjectionHit[MaxRigidbodyOverlapsCount];
        private Rigidbody _lastAttachedRigidbody;
        private bool _solveMovementCollisions = true;
        private bool _solveGrounding = true;
        private bool _movePositionDirty;
        private Vector3 _movePositionTarget = Vector3.zero;
        private bool _moveRotationDirty;
        private Quaternion _moveRotationTarget = Quaternion.identity;
        private bool _lastSolvedOverlapNormalDirty;
        private int _rigidbodyProjectionHitCount;
        private bool _isMovingFromAttachedRigidbody;
        private bool _mustUnground;
        private float _mustUngroundTimeCounter;
        private readonly Vector3 _cachedWorldUp = Vector3.up;
        private readonly Vector3 _cachedWorldForward = Vector3.forward;
        private readonly Vector3 _cachedWorldRight = Vector3.right;
        private readonly Vector3 _cachedZeroVector = Vector3.zero;

        private Quaternion _transientRotation;
        /// <summary>
        /// The character's goal rotation in its movement calculations (always up-to-date during the character update phase)
        /// </summary>
        public Quaternion TransientRotation
        {
            get => _transientRotation;
            private set
            {
                _transientRotation = value;
                CharacterUp = _transientRotation * _cachedWorldUp;
                _characterForward = _transientRotation * _cachedWorldForward;
                _characterRight = _transientRotation * _cachedWorldRight;
            }
        }

        /// <summary>
        /// The character's total velocity, including velocity from standing on rigidbodies or PhysicsMover
        /// </summary>
        public Vector3 Velocity => BaseVelocity + _attachedRigidbodyVelocity;

        // Warning: Don't touch these constants unless you know exactly what you're doing!
        public const int MaxHitsBudget = 16;
        public const int MaxCollisionBudget = 16;
        public const int MaxGroundingSweepIterations = 2;
        public const int MaxSteppingSweepIterations = 3;
        public const int MaxRigidbodyOverlapsCount = 16;
        public const float CollisionOffset = 0.01f;
        public const float GroundProbeReboundDistance = 0.02f;
        public const float MinimumGroundProbingDistance = 0.005f;
        public const float GroundProbingBackstepDistance = 0.1f;
        public const float SweepProbingBackstepDistance = 0.002f;
        public const float SecondaryProbesVertical = 0.02f;
        public const float SecondaryProbesHorizontal = 0.001f;
        public const float MinVelocityMagnitude = 0.01f;
        public const float SteppingForwardDistance = 0.03f;
        public const float MinDistanceForLedge = 0.05f;
        public const float CorrelationForVerticalObstruction = 0.01f;
        public const float ExtraSteppingForwardDistance = 0.01f;
        public const float ExtraStepHeightPadding = 0.01f;
#pragma warning restore 0414 

        private void OnEnable()
        {
            KinematicCharacterSystem.EnsureCreation();
            KinematicCharacterSystem.RegisterCharacterMotor(this);
        }

        private void OnDisable()
        {
            KinematicCharacterSystem.UnregisterCharacterMotor(this);
        }

        private void Reset()
        {
            ValidateData();
        }

        private void OnValidate()
        {
            ValidateData();
        }

        [ContextMenu("Remove Component")]
        private void HandleRemoveComponent()
        {
            var tmpCapsule = gameObject.GetComponent<CapsuleCollider>();
            DestroyImmediate(this);
            DestroyImmediate(tmpCapsule);
        }

        /// <summary>
        /// Handle validating all required values
        /// </summary>
        public void ValidateData()
        {
            capsule = GetComponent<CapsuleCollider>();
            capsuleRadius = Mathf.Clamp(capsuleRadius, 0f, capsuleHeight * 0.5f);
            capsule.direction = 1;
            capsule.sharedMaterial = capsulePhysicsMaterial;
            SetCapsuleDimensions(capsuleRadius, capsuleHeight, capsuleYOffset);

            maxStepHeight = Mathf.Clamp(maxStepHeight, 0f, Mathf.Infinity);
            minRequiredStepDepth = Mathf.Clamp(minRequiredStepDepth, 0f, capsuleRadius);
            maxStableDistanceFromLedge = Mathf.Clamp(maxStableDistanceFromLedge, 0f, capsuleRadius);

            transform.localScale = Vector3.one;

#if UNITY_EDITOR
            capsule.hideFlags = HideFlags.NotEditable;
            if (!Mathf.Approximately(transform.lossyScale.x, 1f) || !Mathf.Approximately(transform.lossyScale.y, 1f) || !Mathf.Approximately(transform.lossyScale.z, 1f))
            {
                Debug.LogError("Character's lossy scale is not (1,1,1). This is not allowed. Make sure the character's transform and all of its parents have a (1,1,1) scale.", gameObject);
            }
#endif
        }

        /// <summary>
        /// Sets whether or not the capsule collider will detect collisions
        /// </summary>
        public void SetCapsuleCollisionsActivation(bool collisionsActive)
        {
            capsule.isTrigger = !collisionsActive;
        }

        /// <summary>
        /// Sets whether or not the motor will solve collisions when moving (or moved onto)
        /// </summary>
        public void SetMovementCollisionsSolvingActivation(bool movementCollisionsSolvingActive)
        {
            _solveMovementCollisions = movementCollisionsSolvingActive;
        }

        /// <summary>
        /// Sets whether or not grounding will be evaluated for all hits
        /// </summary>
        public void SetGroundSolvingActivation(bool stabilitySolvingActive)
        {
            _solveGrounding = stabilitySolvingActive;
        }

        /// <summary>
        /// Sets the character's position directly
        /// </summary>
        public void SetPosition(Vector3 position, bool bypassInterpolation = true)
        {
            Transform.position = position;
            _initialSimulationPosition = position;
            _transientPosition = position;

            if (bypassInterpolation)
            {
                InitialTickPosition = position;
            }
        }

        /// <summary>
        /// Sets the character's rotation directly
        /// </summary>
        public void SetRotation(Quaternion rotation, bool bypassInterpolation = true)
        {
            Transform.rotation = rotation;
            _initialSimulationRotation = rotation;
            TransientRotation = rotation;

            if (bypassInterpolation)
            {
                InitialTickRotation = rotation;
            }
        }

        /// <summary>
        /// Sets the character's position and rotation directly
        /// </summary>
        public void SetPositionAndRotation(Vector3 position, Quaternion rotation, bool bypassInterpolation = true)
        {
            Transform.SetPositionAndRotation(position, rotation);
            _initialSimulationPosition = position;
            _initialSimulationRotation = rotation;
            _transientPosition = position;
            TransientRotation = rotation;

            if (bypassInterpolation)
            {
                InitialTickPosition = position;
                InitialTickRotation = rotation;
            }
        }

        /// <summary>
        /// Moves the character position, taking all movement collision solving int account. The actual move is done the next time the motor updates are called
        /// </summary>
        public void MoveCharacter(Vector3 toPosition)
        {
            _movePositionDirty = true;
            _movePositionTarget = toPosition;
        }

        /// <summary>
        /// Moves the character rotation. The actual move is done the next time the motor updates are called
        /// </summary>
        public void RotateCharacter(Quaternion toRotation)
        {
            _moveRotationDirty = true;
            _moveRotationTarget = toRotation;
        }

        /// <summary>
        /// Returns all the state information of the motor that is pertinent for simulation
        /// </summary>
        public KinematicCharacterMotorState GetState()
        {
            var state = new KinematicCharacterMotorState();

            state.position = _transientPosition;
            state.rotation = _transientRotation;

            state.baseVelocity = BaseVelocity;
            state.attachedRigidbodyVelocity = _attachedRigidbodyVelocity;

            state.mustUnground = _mustUnground;
            state.mustUngroundTime = _mustUngroundTimeCounter;
            state.lastMovementIterationFoundAnyGround = LastMovementIterationFoundAnyGround;
            state.GroundingStatus.CopyFrom(_groundingStatus);
            state.attachedRigidbody = _attachedRigidbody;

            return state;
        }

        /// <summary>
        /// Applies a motor state instantly
        /// </summary>
        public void ApplyState(KinematicCharacterMotorState state, bool bypassInterpolation = true)
        {
            SetPositionAndRotation(state.position, state.rotation, bypassInterpolation);

            BaseVelocity = state.baseVelocity;
            _attachedRigidbodyVelocity = state.attachedRigidbodyVelocity;

            _mustUnground = state.mustUnground;
            _mustUngroundTimeCounter = state.mustUngroundTime;
            LastMovementIterationFoundAnyGround = state.lastMovementIterationFoundAnyGround;
            _groundingStatus.CopyFrom(state.GroundingStatus);
            _attachedRigidbody = state.attachedRigidbody;
        }

        /// <summary>
        /// Resizes capsule. ALso caches importand capsule size data
        /// </summary>
        public void SetCapsuleDimensions(float radius, float height, float yOffset)
        {
            height = Mathf.Max(height, (radius * 2f) + 0.01f); // Safety to prevent invalid capsule geometries

            capsuleRadius = radius;
            capsuleHeight = height;
            capsuleYOffset = yOffset;

            capsule.radius = capsuleRadius;
            capsule.height = Mathf.Clamp(capsuleHeight, capsuleRadius * 2f, capsuleHeight);
            capsule.center = new Vector3(0f, capsuleYOffset, 0f);

            _characterTransformToCapsuleCenter = capsule.center;
            _characterTransformToCapsuleBottom = capsule.center + (-_cachedWorldUp * (capsule.height * 0.5f));
            _characterTransformToCapsuleTop = capsule.center + (_cachedWorldUp * (capsule.height * 0.5f));
            _characterTransformToCapsuleBottomHemi = capsule.center + (-_cachedWorldUp * (capsule.height * 0.5f)) + (_cachedWorldUp * capsule.radius);
            _characterTransformToCapsuleTopHemi = capsule.center + (_cachedWorldUp * (capsule.height * 0.5f)) + (-_cachedWorldUp * capsule.radius);
        }

        private void Awake()
        {
            Transform = transform;
            ValidateData();

            _transientPosition = Transform.position;
            TransientRotation = Transform.rotation;

            // Build CollidableLayers mask
            _collidableLayers = 0;
            for (var i = 0; i < 32; i++)
            {
                if (!Physics.GetIgnoreLayerCollision(gameObject.layer, i))
                {
                    _collidableLayers |= (1 << i);
                }
            }

            SetCapsuleDimensions(capsuleRadius, capsuleHeight, capsuleYOffset);
        }

        /// <summary>
        /// Update phase 1 is meant to be called after physics movers have calculated their velocities, but
        /// before they have simulated their goal positions/rotations. It is responsible for:
        /// - Initializing all values for update
        /// - Handling MovePosition calls
        /// - Solving initial collision overlaps
        /// - Ground probing
        /// - Handle detecting potential interactable rigidbodies
        /// </summary>
        public void UpdatePhase1(float deltaTime)
        {
            // NaN propagation safety stop
            if (float.IsNaN(BaseVelocity.x) || float.IsNaN(BaseVelocity.y) || float.IsNaN(BaseVelocity.z))
            {
                BaseVelocity = Vector3.zero;
            }
            if (float.IsNaN(_attachedRigidbodyVelocity.x) || float.IsNaN(_attachedRigidbodyVelocity.y) || float.IsNaN(_attachedRigidbodyVelocity.z))
            {
                _attachedRigidbodyVelocity = Vector3.zero;
            }

#if UNITY_EDITOR
            if (!Mathf.Approximately(Transform.lossyScale.x, 1f) || !Mathf.Approximately(Transform.lossyScale.y, 1f) || !Mathf.Approximately(Transform.lossyScale.z, 1f))
            {
                Debug.LogError("Character's lossy scale is not (1,1,1). This is not allowed. Make sure the character's transform and all of its parents have a (1,1,1) scale.", gameObject);
            }
#endif

            _rigidbodiesPushedThisMove.Clear();

            // Before update
            CharacterController.BeforeCharacterUpdate(deltaTime);

            _transientPosition = Transform.position;
            TransientRotation = Transform.rotation;
            _initialSimulationPosition = _transientPosition;
            _initialSimulationRotation = _transientRotation;
            _rigidbodyProjectionHitCount = 0;
            _overlapsCount = 0;
            _lastSolvedOverlapNormalDirty = false;

            #region Handle Move Position
            if (_movePositionDirty)
            {
                if (_solveMovementCollisions)
                {
                    var tmpVelocity = GetVelocityFromMovement(_movePositionTarget - _transientPosition, deltaTime);
                    if (InternalCharacterMove(ref tmpVelocity, deltaTime))
                    {
                        if (interactiveRigidbodyHandling)
                        {
                            ProcessVelocityForRigidbodyHits(ref tmpVelocity, deltaTime);
                        }
                    }
                }
                else
                {
                    _transientPosition = _movePositionTarget;
                }

                _movePositionDirty = false;
            }
            #endregion

            _lastGroundingStatus.CopyFrom(_groundingStatus);
            _groundingStatus = new CharacterGroundingReport();
            _groundingStatus.GroundNormal = CharacterUp;

            if (_solveMovementCollisions)
            {
                #region Resolve initial overlaps

                var iterationsMade = 0;
                var overlapSolved = false;
                while (iterationsMade < maxDecollisionIterations && !overlapSolved)
                {
                    var nbOverlaps = CharacterCollisionsOverlap(_transientPosition, _transientRotation, _internalProbedColliders);

                    if (nbOverlaps > 0)
                    {
                        // Solve overlaps that aren't against dynamic rigidbodies or physics movers
                        for (var i = 0; i < nbOverlaps; i++)
                        {
                            if (GetInteractiveRigidbody(_internalProbedColliders[i]) == null)
                            {
                                // Process overlap
                                var overlappedTransform = _internalProbedColliders[i].GetComponent<Transform>();
                                if (Physics.ComputePenetration(
                                        capsule,
                                        _transientPosition,
                                        _transientRotation,
                                        _internalProbedColliders[i],
                                        overlappedTransform.position,
                                        overlappedTransform.rotation,
                                        out var resolutionDirection,
                                        out var resolutionDistance))
                                {
                                    // Resolve along obstruction direction
                                    var mockReport = new HitStabilityReport();
                                    mockReport.IsStable = IsStableOnNormal(resolutionDirection);
                                    resolutionDirection = GetObstructionNormal(resolutionDirection, mockReport.IsStable);

                                    // Solve overlap
                                    var resolutionMovement = resolutionDirection * (resolutionDistance + CollisionOffset);
                                    _transientPosition += resolutionMovement;

                                    // Remember overlaps
                                    if (_overlapsCount < _overlaps.Length)
                                    {
                                        _overlaps[_overlapsCount] = new OverlapResult(resolutionDirection, _internalProbedColliders[i]);
                                        _overlapsCount++;
                                    }

                                    break;
                                }
                            }
                        }
                    }
                    else
                    {
                        overlapSolved = true;
                    }

                    iterationsMade++;
                }
                #endregion
            }

            #region Ground Probing and Snapping
            // Handle ungrounding
            if (_solveGrounding)
            {
                if (MustUnground())
                {
                    _transientPosition += CharacterUp * (MinimumGroundProbingDistance * 1.5f);
                }
                else
                {
                    // Choose the appropriate ground probing distance
                    var selectedGroundProbingDistance = MinimumGroundProbingDistance;
                    if (!_lastGroundingStatus.SnappingPrevented && (_lastGroundingStatus.IsStableOnGround || LastMovementIterationFoundAnyGround))
                    {
                        if (stepHandling != StepHandlingMethod.None)
                        {
                            selectedGroundProbingDistance = Mathf.Max(capsuleRadius, maxStepHeight);
                        }
                        else
                        {
                            selectedGroundProbingDistance = capsuleRadius;
                        }

                        selectedGroundProbingDistance += groundDetectionExtraDistance;
                    }

                    ProbeGround(ref _transientPosition, _transientRotation, selectedGroundProbingDistance, ref _groundingStatus);

                    if (!_lastGroundingStatus.IsStableOnGround && _groundingStatus.IsStableOnGround)
                    {
                        // Handle stable landing
                        BaseVelocity = Vector3.ProjectOnPlane(BaseVelocity, CharacterUp);
                        BaseVelocity = GetDirectionTangentToSurface(BaseVelocity, _groundingStatus.GroundNormal) * BaseVelocity.magnitude;
                    }
                }
            }

            LastMovementIterationFoundAnyGround = false;

            if (_mustUngroundTimeCounter > 0f)
            {
                _mustUngroundTimeCounter -= deltaTime;
            }
            _mustUnground = false;
            #endregion

            if (_solveGrounding)
            {
                CharacterController.PostGroundingUpdate(deltaTime);
            }

            if (interactiveRigidbodyHandling)
            {
                #region Interactive Rigidbody Handling 
                _lastAttachedRigidbody = _attachedRigidbody;
                if (AttachedRigidbodyOverride)
                {
                    _attachedRigidbody = AttachedRigidbodyOverride;
                }
                else
                {
                    // Detect interactive rigidbodies from grounding
                    if (_groundingStatus.IsStableOnGround && _groundingStatus.GroundCollider.attachedRigidbody)
                    {
                        var interactiveRigidbody = GetInteractiveRigidbody(_groundingStatus.GroundCollider);
                        if (interactiveRigidbody)
                        {
                            _attachedRigidbody = interactiveRigidbody;
                        }
                    }
                    else
                    {
                        _attachedRigidbody = null;
                    }
                }

                var tmpVelocityFromCurrentAttachedRigidbody = Vector3.zero;
                var tmpAngularVelocityFromCurrentAttachedRigidbody = Vector3.zero;
                if (_attachedRigidbody)
                {
                    GetVelocityFromRigidbodyMovement(_attachedRigidbody, _transientPosition, deltaTime, out tmpVelocityFromCurrentAttachedRigidbody, out tmpAngularVelocityFromCurrentAttachedRigidbody);
                }

                // Conserve momentum when de-stabilized from an attached rigidbody
                if (preserveAttachedRigidbodyMomentum && _lastAttachedRigidbody != null && _attachedRigidbody != _lastAttachedRigidbody)
                {
                    BaseVelocity += _attachedRigidbodyVelocity;
                    BaseVelocity -= tmpVelocityFromCurrentAttachedRigidbody;
                }

                // Process additionnal Velocity from attached rigidbody
                _attachedRigidbodyVelocity = _cachedZeroVector;
                if (_attachedRigidbody)
                {
                    _attachedRigidbodyVelocity = tmpVelocityFromCurrentAttachedRigidbody;

                    // Rotation from attached rigidbody
                    var newForward = Vector3.ProjectOnPlane(Quaternion.Euler(tmpAngularVelocityFromCurrentAttachedRigidbody * (Mathf.Rad2Deg * deltaTime)) * _characterForward, CharacterUp).normalized;
                    TransientRotation = Quaternion.LookRotation(newForward, CharacterUp);
                }

                // Cancel out horizontal velocity upon landing on an attached rigidbody
                if (_groundingStatus.GroundCollider &&
                    _groundingStatus.GroundCollider.attachedRigidbody &&
                    _groundingStatus.GroundCollider.attachedRigidbody == _attachedRigidbody &&
                    _attachedRigidbody != null &&
                    _lastAttachedRigidbody == null)
                {
                    BaseVelocity -= Vector3.ProjectOnPlane(_attachedRigidbodyVelocity, CharacterUp);
                }

                // Movement from Attached Rigidbody
                if (_attachedRigidbodyVelocity.sqrMagnitude > 0f)
                {
                    _isMovingFromAttachedRigidbody = true;

                    if (_solveMovementCollisions)
                    {
                        // Perform the move from rgdbdy velocity
                        InternalCharacterMove(ref _attachedRigidbodyVelocity, deltaTime);
                    }
                    else
                    {
                        _transientPosition += _attachedRigidbodyVelocity * deltaTime;
                    }

                    _isMovingFromAttachedRigidbody = false;
                }
                #endregion
            }
        }

        /// <summary>
        /// Update phase 2 is meant to be called after physics movers have simulated their goal positions/rotations. 
        /// At the end of this, the TransientPosition/Rotation values will be up-to-date with where the motor should be at the end of its move. 
        /// It is responsible for:
        /// - Solving Rotation
        /// - Handle MoveRotation calls
        /// - Solving potential attached rigidbody overlaps
        /// - Solving Velocity
        /// - Applying planar constraint
        /// </summary>
        public void UpdatePhase2(float deltaTime)
        {
            // Handle rotation
            CharacterController.UpdateRotation(ref _transientRotation, deltaTime);
            TransientRotation = _transientRotation;

            // Handle move rotation
            if (_moveRotationDirty)
            {
                TransientRotation = _moveRotationTarget;
                _moveRotationDirty = false;
            }

            if (_solveMovementCollisions && interactiveRigidbodyHandling)
            {
                if (interactiveRigidbodyHandling)
                {
                    #region Solve potential attached rigidbody overlap
                    if (_attachedRigidbody)
                    {
                        var upwardsOffset = capsule.radius;

                        if (CharacterGroundSweep(
                                _transientPosition + (CharacterUp * upwardsOffset),
                                _transientRotation,
                                -CharacterUp,
                                upwardsOffset,
                                out var closestHit))
                        {
                            if (closestHit.collider.attachedRigidbody == _attachedRigidbody && IsStableOnNormal(closestHit.normal))
                            {
                                var distanceMovedUp = (upwardsOffset - closestHit.distance);
                                _transientPosition = _transientPosition + (CharacterUp * distanceMovedUp) + (CharacterUp * CollisionOffset);
                            }
                        }
                    }
                    #endregion
                }

                if (interactiveRigidbodyHandling)
                {
                    #region Resolve overlaps that could've been caused by rotation or physics movers simulation pushing the character

                    var iterationsMade = 0;
                    var overlapSolved = false;
                    while (iterationsMade < maxDecollisionIterations && !overlapSolved)
                    {
                        var nbOverlaps = CharacterCollisionsOverlap(_transientPosition, _transientRotation, _internalProbedColliders);
                        if (nbOverlaps > 0)
                        {
                            for (var i = 0; i < nbOverlaps; i++)
                            {
                                // Process overlap
                                var overlappedTransform = _internalProbedColliders[i].GetComponent<Transform>();
                                if (Physics.ComputePenetration(
                                        capsule,
                                        _transientPosition,
                                        _transientRotation,
                                        _internalProbedColliders[i],
                                        overlappedTransform.position,
                                        overlappedTransform.rotation,
                                        out var resolutionDirection,
                                        out var resolutionDistance))
                                {
                                    // Resolve along obstruction direction
                                    var mockReport = new HitStabilityReport();
                                    mockReport.IsStable = IsStableOnNormal(resolutionDirection);
                                    resolutionDirection = GetObstructionNormal(resolutionDirection, mockReport.IsStable);

                                    // Solve overlap
                                    var resolutionMovement = resolutionDirection * (resolutionDistance + CollisionOffset);
                                    _transientPosition += resolutionMovement;

                                    // If interactiveRigidbody, register as rigidbody hit for velocity
                                    if (interactiveRigidbodyHandling)
                                    {
                                        var probedRigidbody = GetInteractiveRigidbody(_internalProbedColliders[i]);
                                        if (probedRigidbody != null)
                                        {
                                            var tmpReport = new HitStabilityReport();
                                            tmpReport.IsStable = IsStableOnNormal(resolutionDirection);
                                            if (tmpReport.IsStable)
                                            {
                                                LastMovementIterationFoundAnyGround = true;
                                            }
                                            if (probedRigidbody != _attachedRigidbody)
                                            {
                                                var estimatedCollisionPoint = _transientPosition;


                                                StoreRigidbodyHit(
                                                    probedRigidbody,
                                                    Velocity,
                                                    estimatedCollisionPoint,
                                                    resolutionDirection,
                                                    tmpReport);
                                            }
                                        }
                                    }

                                    // Remember overlaps
                                    if (_overlapsCount < _overlaps.Length)
                                    {
                                        _overlaps[_overlapsCount] = new OverlapResult(resolutionDirection, _internalProbedColliders[i]);
                                        _overlapsCount++;
                                    }

                                    break;
                                }
                            }
                        }
                        else
                        {
                            overlapSolved = true;
                        }

                        iterationsMade++;
                    }
                    #endregion
                }
            }

            // Handle velocity
            CharacterController.UpdateVelocity(ref BaseVelocity, deltaTime);

            //this.CharacterController.UpdateVelocity(ref BaseVelocity, deltaTime);
            if (BaseVelocity.magnitude < MinVelocityMagnitude)
            {
                BaseVelocity = Vector3.zero;
            }

            #region Calculate Character movement from base velocity   
            // Perform the move from base velocity
            if (BaseVelocity.sqrMagnitude > 0f)
            {
                if (_solveMovementCollisions)
                {
                    InternalCharacterMove(ref BaseVelocity, deltaTime);
                }
                else
                {
                    _transientPosition += BaseVelocity * deltaTime;
                }
            }

            // Process rigidbody hits/overlaps to affect velocity
            if (interactiveRigidbodyHandling)
            {
                ProcessVelocityForRigidbodyHits(ref BaseVelocity, deltaTime);
            }
            #endregion

            // Handle planar constraint
            if (hasPlanarConstraint)
            {
                _transientPosition = _initialSimulationPosition + Vector3.ProjectOnPlane(_transientPosition - _initialSimulationPosition, planarConstraintAxis.normalized);
            }

            // Discrete collision detection
            if (discreteCollisionEvents)
            {
                var nbOverlaps = CharacterCollisionsOverlap(_transientPosition, _transientRotation, _internalProbedColliders, CollisionOffset * 2f);
                for (var i = 0; i < nbOverlaps; i++)
                {
                    CharacterController.OnDiscreteCollisionDetected(_internalProbedColliders[i]);
                }
            }

            CharacterController.AfterCharacterUpdate(deltaTime);
        }

        /// <summary>
        /// Determines if motor can be considered stable on given slope normal
        /// </summary>
        private bool IsStableOnNormal(Vector3 normal)
        {
            return Vector3.Angle(CharacterUp, normal) <= maxStableSlopeAngle;
        }

        /// <summary>
        /// Determines if motor can be considered stable on given slope normal
        /// </summary>
        private bool IsStableWithSpecialCases(ref HitStabilityReport stabilityReport, Vector3 velocity)
        {
            if (ledgeAndDenivelationHandling)
            {
                if (stabilityReport.LedgeDetected)
                {
                    if (stabilityReport.IsMovingTowardsEmptySideOfLedge)
                    {
                        // Max snap vel
                        var velocityOnLedgeNormal = Vector3.Project(velocity, stabilityReport.LedgeFacingDirection);
                        if (velocityOnLedgeNormal.magnitude >= maxVelocityForLedgeSnap)
                        {
                            return false;
                        }
                    }

                    // Distance from ledge
                    if (stabilityReport.IsOnEmptySideOfLedge && stabilityReport.DistanceFromLedge > maxStableDistanceFromLedge)
                    {
                        return false;
                    }
                }

                // "Launching" off of slopes of a certain denivelation angle
                if (_lastGroundingStatus.FoundAnyGround && stabilityReport.InnerNormal.sqrMagnitude != 0f && stabilityReport.OuterNormal.sqrMagnitude != 0f)
                {
                    var denivelationAngle = Vector3.Angle(stabilityReport.InnerNormal, stabilityReport.OuterNormal);
                    if (denivelationAngle > maxStableDenivelationAngle)
                    {
                        return false;
                    }
                    else
                    {
                        denivelationAngle = Vector3.Angle(_lastGroundingStatus.InnerGroundNormal, stabilityReport.OuterNormal);
                        if (denivelationAngle > maxStableDenivelationAngle)
                        {
                            return false;
                        }
                    }
                }
            }

            return true;
        }

        /// <summary>
        /// Probes for valid ground and midifies the input transientPosition if ground snapping occurs
        /// </summary>
        public void ProbeGround(ref Vector3 probingPosition, Quaternion atRotation, float probingDistance, ref CharacterGroundingReport groundingReport)
        {
            if (probingDistance < MinimumGroundProbingDistance)
            {
                probingDistance = MinimumGroundProbingDistance;
            }

            var groundSweepsMade = 0;
            var groundSweepingIsOver = false;
            var groundSweepPosition = probingPosition;
            var groundSweepDirection = (atRotation * -_cachedWorldUp);
            var groundProbeDistanceRemaining = probingDistance;
            while (groundProbeDistanceRemaining > 0 && (groundSweepsMade <= MaxGroundingSweepIterations) && !groundSweepingIsOver)
            {
                // Sweep for ground detection
                if (CharacterGroundSweep(
                        groundSweepPosition, // position
                        atRotation, // rotation
                        groundSweepDirection, // direction
                        groundProbeDistanceRemaining, // distance
                        out var groundSweepHit)) // hit
                {
                    var targetPosition = groundSweepPosition + (groundSweepDirection * groundSweepHit.distance);
                    var groundHitStabilityReport = new HitStabilityReport();
                    EvaluateHitStability(groundSweepHit.collider, groundSweepHit.normal, groundSweepHit.point, targetPosition, _transientRotation, BaseVelocity, ref groundHitStabilityReport);

                    groundingReport.FoundAnyGround = true;
                    groundingReport.GroundNormal = groundSweepHit.normal;
                    groundingReport.InnerGroundNormal = groundHitStabilityReport.InnerNormal;
                    groundingReport.OuterGroundNormal = groundHitStabilityReport.OuterNormal;
                    groundingReport.GroundCollider = groundSweepHit.collider;
                    groundingReport.GroundPoint = groundSweepHit.point;
                    groundingReport.SnappingPrevented = false;

                    // Found stable ground
                    if (groundHitStabilityReport.IsStable)
                    {
                        // Find all scenarios where ground snapping should be canceled
                        groundingReport.SnappingPrevented = !IsStableWithSpecialCases(ref groundHitStabilityReport, BaseVelocity);

                        groundingReport.IsStableOnGround = true;

                        // Ground snapping
                        if (!groundingReport.SnappingPrevented)
                        {
                            probingPosition = groundSweepPosition + (groundSweepDirection * (groundSweepHit.distance - CollisionOffset));
                        }

                        CharacterController.OnGroundHit(groundSweepHit.collider, groundSweepHit.normal, groundSweepHit.point, ref groundHitStabilityReport);
                        groundSweepingIsOver = true;
                    }
                    else
                    {
                        // Calculate movement from this iteration and advance position
                        var sweepMovement = (groundSweepDirection * groundSweepHit.distance) + ((atRotation * _cachedWorldUp) * Mathf.Max(CollisionOffset, groundSweepHit.distance));
                        groundSweepPosition = groundSweepPosition + sweepMovement;

                        // Set remaining distance
                        groundProbeDistanceRemaining = Mathf.Min(GroundProbeReboundDistance, Mathf.Max(groundProbeDistanceRemaining - sweepMovement.magnitude, 0f));

                        // Reorient direction
                        groundSweepDirection = Vector3.ProjectOnPlane(groundSweepDirection, groundSweepHit.normal).normalized;
                    }
                }
                else
                {
                    groundSweepingIsOver = true;
                }

                groundSweepsMade++;
            }
        }

        /// <summary>
        /// Forces the character to unground itself on its next grounding update
        /// </summary>
        public void ForceUnground(float time = 0.1f)
        {
            _mustUnground = true;
            _mustUngroundTimeCounter = time;
        }

        public bool MustUnground()
        {
            return _mustUnground || _mustUngroundTimeCounter > 0f;
        }

        /// <summary>
        /// Returns the direction adjusted to be tangent to a specified surface normal relatively to the character's up direction.
        /// Useful for reorienting a direction on a slope without any lateral deviation in trajectory
        /// </summary>
        public Vector3 GetDirectionTangentToSurface(Vector3 direction, Vector3 surfaceNormal)
        {
            var directionRight = Vector3.Cross(direction, CharacterUp);
            return Vector3.Cross(surfaceNormal, directionRight).normalized;
        }

        /// <summary>
        /// Moves the character's position by given movement while taking into account all physics simulation, step-handling and 
        /// velocity projection rules that affect the character motor
        /// </summary>
        /// <returns> Returns false if movement could not be solved until the end </returns>
        private bool InternalCharacterMove(ref Vector3 transientVelocity, float deltaTime)
        {
            if (deltaTime <= 0f)
                return false;

            // Planar constraint
            if (hasPlanarConstraint)
            {
                transientVelocity = Vector3.ProjectOnPlane(transientVelocity, planarConstraintAxis.normalized);
            }

            var wasCompleted = true;
            var remainingMovementDirection = transientVelocity.normalized;
            var remainingMovementMagnitude = transientVelocity.magnitude * deltaTime;
            var sweepsMade = 0;
            var hitSomethingThisSweepIteration = true;
            var tmpMovedPosition = _transientPosition;
            var previousHitIsStable = false;
            var previousVelocity = _cachedZeroVector;
            var previousObstructionNormal = _cachedZeroVector;
            var sweepState = MovementSweepState.Initial;

            // Project movement against current overlaps before doing the sweeps
            for (var i = 0; i < _overlapsCount; i++)
            {
                var overlapNormal = _overlaps[i].Normal;
                if (Vector3.Dot(remainingMovementDirection, overlapNormal) < 0f)
                {
                    var stableOnHit = IsStableOnNormal(overlapNormal) && !MustUnground();
                    var velocityBeforeProjection = transientVelocity;
                    var obstructionNormal = GetObstructionNormal(overlapNormal, stableOnHit);

                    InternalHandleVelocityProjection(
                        stableOnHit,
                        obstructionNormal,
                        ref sweepState,
                        previousHitIsStable,
                        previousVelocity,
                        previousObstructionNormal,
                        ref transientVelocity,
                        ref remainingMovementMagnitude,
                        ref remainingMovementDirection);

                    previousHitIsStable = stableOnHit;
                    previousVelocity = velocityBeforeProjection;
                    previousObstructionNormal = obstructionNormal;
                }
            }

            // Sweep the desired movement to detect collisions
            while (remainingMovementMagnitude > 0f &&
                (sweepsMade <= maxMovementIterations) &&
                hitSomethingThisSweepIteration)
            {
                var foundClosestHit = false;
                Vector3 closestSweepHitPoint = default;
                Vector3 closestSweepHitNormal = default;
                var closestSweepHitDistance = 0f;
                Collider closestSweepHitCollider = null;

                if (checkMovementInitialOverlaps)
                {
                    var numOverlaps = CharacterCollisionsOverlap(
                                        tmpMovedPosition,
                                        _transientRotation,
                                        _internalProbedColliders);
                    if (numOverlaps > 0)
                    {
                        closestSweepHitDistance = 0f;

                        var mostObstructingOverlapNormalDotProduct = 2f;

                        for (var i = 0; i < numOverlaps; i++)
                        {
                            var tmpCollider = _internalProbedColliders[i];

                            if (Physics.ComputePenetration(
                                capsule,
                                tmpMovedPosition,
                                _transientRotation,
                                tmpCollider,
                                tmpCollider.transform.position,
                                tmpCollider.transform.rotation,
                                out var resolutionDirection,
                                out var resolutionDistance))
                            {
                                var dotProduct = Vector3.Dot(remainingMovementDirection, resolutionDirection);
                                if (dotProduct < 0f && dotProduct < mostObstructingOverlapNormalDotProduct)
                                {
                                    mostObstructingOverlapNormalDotProduct = dotProduct;

                                    closestSweepHitNormal = resolutionDirection;
                                    closestSweepHitCollider = tmpCollider;
                                    closestSweepHitPoint = tmpMovedPosition + (_transientRotation * CharacterTransformToCapsuleCenter) + (resolutionDirection * resolutionDistance);

                                    foundClosestHit = true;
                                }
                            }
                        }
                    }
                }

                if (!foundClosestHit && CharacterCollisionsSweep(
                        tmpMovedPosition, // position
                        _transientRotation, // rotation
                        remainingMovementDirection, // direction
                        remainingMovementMagnitude + CollisionOffset, // distance
                        out var closestSweepHit, // closest hit
                        _internalCharacterHits) // all hits
                    > 0)
                {
                    closestSweepHitNormal = closestSweepHit.normal;
                    closestSweepHitDistance = closestSweepHit.distance;
                    closestSweepHitCollider = closestSweepHit.collider;
                    closestSweepHitPoint = closestSweepHit.point;

                    foundClosestHit = true;
                }

                if (foundClosestHit)
                {
                    // Calculate movement from this iteration
                    var sweepMovement = (remainingMovementDirection * (Mathf.Max(0f, closestSweepHitDistance - CollisionOffset)));
                    tmpMovedPosition += sweepMovement;
                    remainingMovementMagnitude -= sweepMovement.magnitude;

                    // Evaluate if hit is stable
                    var moveHitStabilityReport = new HitStabilityReport();
                    EvaluateHitStability(closestSweepHitCollider, closestSweepHitNormal, closestSweepHitPoint, tmpMovedPosition, _transientRotation, transientVelocity, ref moveHitStabilityReport);

                    // Handle stepping up steps points higher than bottom capsule radius
                    var foundValidStepHit = false;
                    if (_solveGrounding && stepHandling != StepHandlingMethod.None && moveHitStabilityReport.ValidStepDetected)
                    {
                        var obstructionCorrelation = Mathf.Abs(Vector3.Dot(closestSweepHitNormal, CharacterUp));
                        if (obstructionCorrelation <= CorrelationForVerticalObstruction)
                        {
                            var stepForwardDirection = Vector3.ProjectOnPlane(-closestSweepHitNormal, CharacterUp).normalized;
                            var stepCastStartPoint = (tmpMovedPosition + (stepForwardDirection * SteppingForwardDistance)) +
                                                     (CharacterUp * maxStepHeight);

                            // Cast downward from the top of the stepping height
                            var nbStepHits = CharacterCollisionsSweep(
                                                stepCastStartPoint, // position
                                                _transientRotation, // rotation
                                                -CharacterUp, // direction
                                                maxStepHeight, // distance
                                                out _, // closest hit
                                                _internalCharacterHits,
                                                0f,
                                                true); // all hits 

                            // Check for hit corresponding to stepped collider
                            for (var i = 0; i < nbStepHits; i++)
                            {
                                if (_internalCharacterHits[i].collider == moveHitStabilityReport.SteppedCollider)
                                {
                                    var endStepPosition = stepCastStartPoint + (-CharacterUp * (_internalCharacterHits[i].distance - CollisionOffset));
                                    tmpMovedPosition = endStepPosition;
                                    foundValidStepHit = true;

                                    // Project velocity on ground normal at step
                                    transientVelocity = Vector3.ProjectOnPlane(transientVelocity, CharacterUp);
                                    remainingMovementDirection = transientVelocity.normalized;

                                    break;
                                }
                            }
                        }
                    }

                    // Handle movement solving
                    if (!foundValidStepHit)
                    {
                        var obstructionNormal = GetObstructionNormal(closestSweepHitNormal, moveHitStabilityReport.IsStable);

                        // Movement hit callback
                        CharacterController.OnMovementHit(closestSweepHitCollider, closestSweepHitNormal, closestSweepHitPoint, ref moveHitStabilityReport);

                        // Handle remembering rigidbody hits
                        if (interactiveRigidbodyHandling && closestSweepHitCollider.attachedRigidbody)
                        {
                            StoreRigidbodyHit(
                                closestSweepHitCollider.attachedRigidbody,
                                transientVelocity,
                                closestSweepHitPoint,
                                obstructionNormal,
                                moveHitStabilityReport);
                        }

                        var stableOnHit = moveHitStabilityReport.IsStable && !MustUnground();
                        var velocityBeforeProj = transientVelocity;

                        // Project velocity for next iteration
                        InternalHandleVelocityProjection(
                            stableOnHit,
                            obstructionNormal,
                            ref sweepState,
                            previousHitIsStable,
                            previousVelocity,
                            previousObstructionNormal,
                            ref transientVelocity,
                            ref remainingMovementMagnitude,
                            ref remainingMovementDirection);

                        previousHitIsStable = stableOnHit;
                        previousVelocity = velocityBeforeProj;
                        previousObstructionNormal = obstructionNormal;
                    }
                }
                // If we hit nothing...
                else
                {
                    hitSomethingThisSweepIteration = false;
                }

                // Safety for exceeding max sweeps allowed
                sweepsMade++;
                if (sweepsMade > maxMovementIterations)
                {
                    if (killRemainingMovementWhenExceedMaxMovementIterations)
                    {
                        remainingMovementMagnitude = 0f;
                    }

                    if (killVelocityWhenExceedMaxMovementIterations)
                    {
                        transientVelocity = Vector3.zero;
                    }
                    wasCompleted = false;
                }
            }

            // Move position for the remainder of the movement
            tmpMovedPosition += (remainingMovementDirection * remainingMovementMagnitude);
            _transientPosition = tmpMovedPosition;

            return wasCompleted;
        }

        /// <summary>
        /// Gets the effective normal for movement obstruction depending on current grounding status
        /// </summary>
        private Vector3 GetObstructionNormal(Vector3 hitNormal, bool stableOnHit)
        {
            // Find hit/obstruction/offset normal
            var obstructionNormal = hitNormal;
            if (_groundingStatus.IsStableOnGround && !MustUnground() && !stableOnHit)
            {
                var obstructionLeftAlongGround = Vector3.Cross(_groundingStatus.GroundNormal, obstructionNormal).normalized;
                obstructionNormal = Vector3.Cross(obstructionLeftAlongGround, CharacterUp).normalized;
            }

            // Catch cases where cross product between parallel normals returned 0
            if (obstructionNormal.sqrMagnitude == 0f)
            {
                obstructionNormal = hitNormal;
            }

            return obstructionNormal;
        }

        /// <summary>
        /// Remembers a rigidbody hit for processing later
        /// </summary>
        private void StoreRigidbodyHit(Rigidbody hitRigidbody, Vector3 hitVelocity, Vector3 hitPoint, Vector3 obstructionNormal, HitStabilityReport hitStabilityReport)
        {
            if (_rigidbodyProjectionHitCount < _internalRigidbodyProjectionHits.Length)
            {
                if (!hitRigidbody.GetComponent<KinematicCharacterMotor>())
                {
                    var rph = new RigidbodyProjectionHit();
                    rph.Rigidbody = hitRigidbody;
                    rph.HitPoint = hitPoint;
                    rph.EffectiveHitNormal = obstructionNormal;
                    rph.HitVelocity = hitVelocity;
                    rph.StableOnHit = hitStabilityReport.IsStable;

                    _internalRigidbodyProjectionHits[_rigidbodyProjectionHitCount] = rph;
                    _rigidbodyProjectionHitCount++;
                }
            }
        }

        public void SetTransientPosition(Vector3 newPos)
        {
            _transientPosition = newPos;
        }

        /// <summary>
        /// Processes movement projection upon detecting a hit
        /// </summary>
        private void InternalHandleVelocityProjection(bool stableOnHit, Vector3 obstructionNormal,
            ref MovementSweepState sweepState, bool previousHitIsStable, Vector3 previousVelocity, Vector3 previousObstructionNormal,
            ref Vector3 transientVelocity, ref float remainingMovementMagnitude, ref Vector3 remainingMovementDirection)
        {
            if (transientVelocity.sqrMagnitude <= 0f)
            {
                return;
            }

            var velocityBeforeProjection = transientVelocity;

            if (stableOnHit)
            {
                LastMovementIterationFoundAnyGround = true;
                HandleVelocityProjection(ref transientVelocity, obstructionNormal, true);
            }
            else
            {
                // Handle projection
                if (sweepState == MovementSweepState.Initial)
                {
                    HandleVelocityProjection(ref transientVelocity, obstructionNormal, false);
                    sweepState = MovementSweepState.AfterFirstHit;
                }
                // Blocking crease handling
                else if (sweepState == MovementSweepState.AfterFirstHit)
                {
                    EvaluateCrease(
                        transientVelocity,
                        previousVelocity,
                        obstructionNormal,
                        previousObstructionNormal,
                        false,
                        previousHitIsStable,
                        _groundingStatus.IsStableOnGround && !MustUnground(),
                        out var foundCrease,
                        out var creaseDirection);

                    if (foundCrease)
                    {
                        if (_groundingStatus.IsStableOnGround && !MustUnground())
                        {
                            transientVelocity = Vector3.zero;
                            sweepState = MovementSweepState.FoundBlockingCorner;
                        }
                        else
                        {
                            transientVelocity = Vector3.Project(transientVelocity, creaseDirection);
                            sweepState = MovementSweepState.FoundBlockingCrease;
                        }
                    }
                    else
                    {
                        HandleVelocityProjection(ref transientVelocity, obstructionNormal, false);
                    }
                }
                // Blocking corner handling
                else if (sweepState == MovementSweepState.FoundBlockingCrease)
                {
                    transientVelocity = Vector3.zero;
                    sweepState = MovementSweepState.FoundBlockingCorner;
                }
            }

            if (hasPlanarConstraint)
            {
                transientVelocity = Vector3.ProjectOnPlane(transientVelocity, planarConstraintAxis.normalized);
            }

            var newVelocityFactor = transientVelocity.magnitude / velocityBeforeProjection.magnitude;
            remainingMovementMagnitude *= newVelocityFactor;
            remainingMovementDirection = transientVelocity.normalized;
        }

        private void EvaluateCrease(
            Vector3 currentCharacterVelocity,
            Vector3 previousCharacterVelocity,
            Vector3 currentHitNormal,
            Vector3 previousHitNormal,
            bool currentHitIsStable,
            bool previousHitIsStable,
            bool characterIsStable,
            out bool isValidCrease,
            out Vector3 creaseDirection)
        {
            isValidCrease = false;
            creaseDirection = default;

            if (!characterIsStable || !currentHitIsStable || !previousHitIsStable)
            {
                var tmpBlockingCreaseDirection = Vector3.Cross(currentHitNormal, previousHitNormal).normalized;
                var dotPlanes = Vector3.Dot(currentHitNormal, previousHitNormal);
                var isVelocityConstrainedByCrease = false;

                // Avoid calculations if the two planes are the same
                if (dotPlanes < 0.999f)
                {
                    // TODO: can this whole part be made simpler? (with 2d projections, etc)
                    var normalAOnCreasePlane = Vector3.ProjectOnPlane(currentHitNormal, tmpBlockingCreaseDirection).normalized;
                    var normalBOnCreasePlane = Vector3.ProjectOnPlane(previousHitNormal, tmpBlockingCreaseDirection).normalized;
                    var dotPlanesOnCreasePlane = Vector3.Dot(normalAOnCreasePlane, normalBOnCreasePlane);

                    var enteringVelocityDirectionOnCreasePlane = Vector3.ProjectOnPlane(previousCharacterVelocity, tmpBlockingCreaseDirection).normalized;

                    if (dotPlanesOnCreasePlane <= (Vector3.Dot(-enteringVelocityDirectionOnCreasePlane, normalAOnCreasePlane) + 0.001f) &&
                        dotPlanesOnCreasePlane <= (Vector3.Dot(-enteringVelocityDirectionOnCreasePlane, normalBOnCreasePlane) + 0.001f))
                    {
                        isVelocityConstrainedByCrease = true;
                    }
                }

                if (isVelocityConstrainedByCrease)
                {
                    // Flip crease direction to make it representative of the real direction our velocity would be projected to
                    if (Vector3.Dot(tmpBlockingCreaseDirection, currentCharacterVelocity) < 0f)
                    {
                        tmpBlockingCreaseDirection = -tmpBlockingCreaseDirection;
                    }

                    isValidCrease = true;
                    creaseDirection = tmpBlockingCreaseDirection;
                }
            }
        }

        /// <summary>
        /// Allows you to override the way velocity is projected on an obstruction
        /// </summary>
        public void HandleVelocityProjection(ref Vector3 velocity, Vector3 obstructionNormal, bool stableOnHit)
        {
            if (_groundingStatus.IsStableOnGround && !MustUnground())
            {
                // On stable slopes, simply reorient the movement without any loss
                if (stableOnHit)
                {
                    velocity = GetDirectionTangentToSurface(velocity, obstructionNormal) * velocity.magnitude;
                }
                // On blocking hits, project the movement on the obstruction while following the grounding plane
                else
                {
                    var obstructionRightAlongGround = Vector3.Cross(obstructionNormal, _groundingStatus.GroundNormal).normalized;
                    var obstructionUpAlongGround = Vector3.Cross(obstructionRightAlongGround, obstructionNormal).normalized;
                    velocity = GetDirectionTangentToSurface(velocity, obstructionUpAlongGround) * velocity.magnitude;
                    velocity = Vector3.ProjectOnPlane(velocity, obstructionNormal);
                }
            }
            else
            {
                if (stableOnHit)
                {
                    // Handle stable landing
                    velocity = Vector3.ProjectOnPlane(velocity, CharacterUp);
                    velocity = GetDirectionTangentToSurface(velocity, obstructionNormal) * velocity.magnitude;
                }
                // Handle generic obstruction
                else
                {
                    velocity = Vector3.ProjectOnPlane(velocity, obstructionNormal);
                }
            }
        }

        /// <summary>
        /// Allows you to override the way hit rigidbodies are pushed / interacted with. 
        /// ProcessedVelocity is what must be modified if this interaction affects the character's velocity.
        /// </summary>
        public void HandleSimulatedRigidbodyInteraction(ref Vector3 processedVelocity, RigidbodyProjectionHit hit, float deltaTime)
        {
        }

        /// <summary>
        /// Takes into account rigidbody hits for adding to the velocity
        /// </summary>
        private void ProcessVelocityForRigidbodyHits(ref Vector3 processedVelocity, float deltaTime)
        {
            for (var i = 0; i < _rigidbodyProjectionHitCount; i++)
            {
                var bodyHit = _internalRigidbodyProjectionHits[i];

                if (bodyHit.Rigidbody && !_rigidbodiesPushedThisMove.Contains(bodyHit.Rigidbody))
                {
                    if (_internalRigidbodyProjectionHits[i].Rigidbody != _attachedRigidbody)
                    {
                        // Remember we hit this rigidbody
                        _rigidbodiesPushedThisMove.Add(bodyHit.Rigidbody);

                        var characterMass = simulatedCharacterMass;
                        var characterVelocity = bodyHit.HitVelocity;

                        var hitCharacterMotor = bodyHit.Rigidbody.GetComponent<KinematicCharacterMotor>();
                        var hitBodyIsCharacter = hitCharacterMotor != null;
                        var hitBodyIsDynamic = !bodyHit.Rigidbody.isKinematic;
                        var hitBodyMassAtPoint = bodyHit.Rigidbody.mass; // todo
                        var hitBodyVelocity = bodyHit.Rigidbody.linearVelocity;
                        if (hitBodyIsCharacter)
                        {
                            hitBodyMassAtPoint = hitCharacterMotor.simulatedCharacterMass; // todo
                            hitBodyVelocity = hitCharacterMotor.BaseVelocity;
                        }
                        else if (!hitBodyIsDynamic)
                        {
                            var physicsMover = bodyHit.Rigidbody.GetComponent<PhysicsMover>();
                            if(physicsMover)
                            {
                                hitBodyVelocity = physicsMover.Velocity;
                            }
                        }

                        // Calculate the ratio of the total mass that the character mass represents
                        float characterToBodyMassRatio;
                        {
                            if (characterMass + hitBodyMassAtPoint > 0f)
                            {
                                characterToBodyMassRatio = characterMass / (characterMass + hitBodyMassAtPoint);
                            }
                            else
                            {
                                characterToBodyMassRatio = 0.5f;
                            }

                            // Hitting a non-dynamic body
                            if (!hitBodyIsDynamic)
                            {
                                characterToBodyMassRatio = 0f;
                            }
                            // Emulate kinematic body interaction
                            else if (rigidbodyInteractionType == RigidbodyInteractionType.Kinematic && !hitBodyIsCharacter)
                            {
                                characterToBodyMassRatio = 1f;
                            }
                        }

                        ComputeCollisionResolutionForHitBody(
                            bodyHit.EffectiveHitNormal,
                            characterVelocity,
                            hitBodyVelocity,
                            characterToBodyMassRatio,
                            out var velocityChangeOnCharacter,
                            out var velocityChangeOnBody);

                        processedVelocity += velocityChangeOnCharacter;

                        if (hitBodyIsCharacter)
                        {
                            hitCharacterMotor.BaseVelocity += velocityChangeOnCharacter;
                        }
                        else if (hitBodyIsDynamic)
                        {
                            bodyHit.Rigidbody.AddForceAtPosition(velocityChangeOnBody, bodyHit.HitPoint, ForceMode.VelocityChange);
                        }

                        if (rigidbodyInteractionType == RigidbodyInteractionType.SimulatedDynamic)
                        {
                            HandleSimulatedRigidbodyInteraction(ref processedVelocity, bodyHit, deltaTime);
                        }
                    }
                }
            }

        }

        public void ComputeCollisionResolutionForHitBody(
            Vector3 hitNormal,
            Vector3 characterVelocity,
            Vector3 bodyVelocity,
            float characterToBodyMassRatio,
            out Vector3 velocityChangeOnCharacter,
            out Vector3 velocityChangeOnBody)
        {
            velocityChangeOnCharacter = default;
            velocityChangeOnBody = default;

            var bodyToCharacterMassRatio = 1f - characterToBodyMassRatio;
            var characterVelocityMagnitudeOnHitNormal = Vector3.Dot(characterVelocity, hitNormal);
            var bodyVelocityMagnitudeOnHitNormal = Vector3.Dot(bodyVelocity, hitNormal);

            // if character velocity was going against the obstruction, restore the portion of the velocity that got projected during the movement phase
            if (characterVelocityMagnitudeOnHitNormal < 0f)
            {
                var restoredCharacterVelocity = hitNormal * characterVelocityMagnitudeOnHitNormal;
                velocityChangeOnCharacter += restoredCharacterVelocity;
            }

            // solve impulse velocities on both bodies, but only if the body velocity would be giving resistance to the character in any way
            if (bodyVelocityMagnitudeOnHitNormal > characterVelocityMagnitudeOnHitNormal)
            {
                var relativeImpactVelocity = hitNormal * (bodyVelocityMagnitudeOnHitNormal - characterVelocityMagnitudeOnHitNormal);
                velocityChangeOnCharacter += relativeImpactVelocity * bodyToCharacterMassRatio;
                velocityChangeOnBody += -relativeImpactVelocity * characterToBodyMassRatio;
            }
        }

        /// <summary>
        /// Determines if the input collider is valid for collision processing
        /// </summary>
        /// <returns> Returns true if the collider is valid </returns>
        private bool CheckIfColliderValidForCollisions(Collider coll)
        {
            // Ignore self
            if (coll == capsule)
            {
                return false;
            }

            if (!InternalIsColliderValidForCollisions(coll))
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Determines if the input collider is valid for collision processing
        /// </summary>
        private bool InternalIsColliderValidForCollisions(Collider coll)
        {
            var colliderAttachedRigidbody = coll.attachedRigidbody;
            if (colliderAttachedRigidbody)
            {
                var isRigidbodyKinematic = colliderAttachedRigidbody.isKinematic;

                // If movement is made from AttachedRigidbody, ignore the AttachedRigidbody
                if (_isMovingFromAttachedRigidbody && (!isRigidbodyKinematic || colliderAttachedRigidbody == _attachedRigidbody))
                {
                    return false;
                }

                // don't collide with dynamic rigidbodies if our RigidbodyInteractionType is kinematic
                if (rigidbodyInteractionType == RigidbodyInteractionType.Kinematic && !isRigidbodyKinematic)
                {
                    // wake up rigidbody
                    if (coll.attachedRigidbody)
                    {
                        coll.attachedRigidbody.WakeUp();
                    }

                    return false;
                }
            }

            // Custom checks
            var colliderValid = CharacterController.IsColliderValidForCollisions(coll);
            if (!colliderValid)
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Determines if the motor is considered stable on a given hit
        /// </summary>
        public void EvaluateHitStability(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, Vector3 atCharacterPosition, Quaternion atCharacterRotation, Vector3 withCharacterVelocity, ref HitStabilityReport stabilityReport)
        {
            if (!_solveGrounding)
            {
                stabilityReport.IsStable = false;
                return;
            }

            var atCharacterUp = atCharacterRotation * _cachedWorldUp;
            var innerHitDirection = Vector3.ProjectOnPlane(hitNormal, atCharacterUp).normalized;

            stabilityReport.IsStable = IsStableOnNormal(hitNormal);

            stabilityReport.FoundInnerNormal = false;
            stabilityReport.FoundOuterNormal = false;
            stabilityReport.InnerNormal = hitNormal;
            stabilityReport.OuterNormal = hitNormal;

            // Ledge handling
            if (ledgeAndDenivelationHandling)
            {
                var ledgeCheckHeight = MinDistanceForLedge;
                if (stepHandling != StepHandlingMethod.None)
                {
                    ledgeCheckHeight = maxStepHeight;
                }

                var isStableLedgeInner = false;
                var isStableLedgeOuter = false;

                if (CharacterCollisionsRaycast(
                        hitPoint + (atCharacterUp * SecondaryProbesVertical) + (innerHitDirection * SecondaryProbesHorizontal),
                        -atCharacterUp,
                        ledgeCheckHeight + SecondaryProbesVertical,
                        out var innerLedgeHit,
                        _internalCharacterHits) > 0)
                {
                    var innerLedgeNormal = innerLedgeHit.normal;
                    stabilityReport.InnerNormal = innerLedgeNormal;
                    stabilityReport.FoundInnerNormal = true;
                    isStableLedgeInner = IsStableOnNormal(innerLedgeNormal);
                }

                if (CharacterCollisionsRaycast(
                        hitPoint + (atCharacterUp * SecondaryProbesVertical) + (-innerHitDirection * SecondaryProbesHorizontal),
                        -atCharacterUp,
                        ledgeCheckHeight + SecondaryProbesVertical,
                        out var outerLedgeHit,
                        _internalCharacterHits) > 0)
                {
                    var outerLedgeNormal = outerLedgeHit.normal;
                    stabilityReport.OuterNormal = outerLedgeNormal;
                    stabilityReport.FoundOuterNormal = true;
                    isStableLedgeOuter = IsStableOnNormal(outerLedgeNormal);
                }

                stabilityReport.LedgeDetected = (isStableLedgeInner != isStableLedgeOuter);
                if (stabilityReport.LedgeDetected)
                {
                    stabilityReport.IsOnEmptySideOfLedge = isStableLedgeOuter && !isStableLedgeInner;
                    stabilityReport.LedgeGroundNormal = isStableLedgeOuter ? stabilityReport.OuterNormal : stabilityReport.InnerNormal;
                    stabilityReport.LedgeRightDirection = Vector3.Cross(hitNormal, stabilityReport.LedgeGroundNormal).normalized;
                    stabilityReport.LedgeFacingDirection = Vector3.ProjectOnPlane(Vector3.Cross(stabilityReport.LedgeGroundNormal, stabilityReport.LedgeRightDirection), CharacterUp).normalized;
                    stabilityReport.DistanceFromLedge = Vector3.ProjectOnPlane((hitPoint - (atCharacterPosition + (atCharacterRotation * _characterTransformToCapsuleBottom))), atCharacterUp).magnitude;
                    stabilityReport.IsMovingTowardsEmptySideOfLedge = Vector3.Dot(withCharacterVelocity.normalized, stabilityReport.LedgeFacingDirection) > 0f;
                }

                if (stabilityReport.IsStable)
                {
                    stabilityReport.IsStable = IsStableWithSpecialCases(ref stabilityReport, withCharacterVelocity);
                }
            }

            // Step handling
            if (stepHandling != StepHandlingMethod.None && !stabilityReport.IsStable)
            {
                // Stepping not supported on dynamic rigidbodies
                var hitRigidbody = hitCollider.attachedRigidbody;
                if (!(hitRigidbody && !hitRigidbody.isKinematic))
                {
                    DetectSteps(atCharacterPosition, atCharacterRotation, hitPoint, innerHitDirection, ref stabilityReport);

                    if (stabilityReport.ValidStepDetected)
                    {
                        stabilityReport.IsStable = true;
                    }
                }
            }

            CharacterController.ProcessHitStabilityReport(hitCollider, hitNormal, hitPoint, atCharacterPosition, atCharacterRotation, ref stabilityReport);
        }

        private void DetectSteps(Vector3 characterPosition, Quaternion characterRotation, Vector3 hitPoint, Vector3 innerHitDirection, ref HitStabilityReport stabilityReport)
        {
            var characterUp = characterRotation * _cachedWorldUp;
            var verticalCharToHit = Vector3.Project((hitPoint - characterPosition), characterUp);
            var horizontalCharToHitDirection = Vector3.ProjectOnPlane((hitPoint - characterPosition), characterUp).normalized;
            var stepCheckStartPos = (hitPoint - verticalCharToHit) + (characterUp * maxStepHeight) + (horizontalCharToHitDirection * (CollisionOffset * 3f));

            // Do outer step check with capsule cast on hit point
            var nbStepHits = CharacterCollisionsSweep(
                stepCheckStartPos,
                characterRotation,
                -characterUp,
                maxStepHeight + CollisionOffset,
                out _,
                _internalCharacterHits,
                0f,
                true);

            // Check for overlaps and obstructions at the hit position
            if (CheckStepValidity(nbStepHits, characterPosition, characterRotation, innerHitDirection, stepCheckStartPos, out var tmpCollider))
            {
                stabilityReport.ValidStepDetected = true;
                stabilityReport.SteppedCollider = tmpCollider;
            }

            if (stepHandling == StepHandlingMethod.Extra && !stabilityReport.ValidStepDetected)
            {
                // Do min reach step check with capsule cast on hit point
                stepCheckStartPos = characterPosition + (characterUp * maxStepHeight) + (-innerHitDirection * minRequiredStepDepth);
                nbStepHits = CharacterCollisionsSweep(
                                stepCheckStartPos,
                                characterRotation,
                                -characterUp,
                                maxStepHeight - CollisionOffset,
                                out _,
                                _internalCharacterHits,
                                0f,
                                true);

                // Check for overlaps and obstructions at the hit position
                if (CheckStepValidity(nbStepHits, characterPosition, characterRotation, innerHitDirection, stepCheckStartPos, out tmpCollider))
                {
                    stabilityReport.ValidStepDetected = true;
                    stabilityReport.SteppedCollider = tmpCollider;
                }
            }
        }

        private bool CheckStepValidity(int nbStepHits, Vector3 characterPosition, Quaternion characterRotation, Vector3 innerHitDirection, Vector3 stepCheckStartPos, out Collider hitCollider)
        {
            hitCollider = null;
            var characterUp = characterRotation * Vector3.up;

            // Find the farthest valid hit for stepping

            while (nbStepHits > 0)
            {
                // Get the farthest hit among the remaining hits
                var farthestHit = new RaycastHit();
                var farthestDistance = 0f;
                var farthestIndex = 0;
                for (var i = 0; i < nbStepHits; i++)
                {
                    var hitDistance = _internalCharacterHits[i].distance;
                    if (hitDistance > farthestDistance)
                    {
                        farthestDistance = hitDistance;
                        farthestHit = _internalCharacterHits[i];
                        farthestIndex = i;
                    }
                }

                var characterPositionAtHit = stepCheckStartPos + (-characterUp * (farthestHit.distance - CollisionOffset));

                var atStepOverlaps = CharacterCollisionsOverlap(characterPositionAtHit, characterRotation, _internalProbedColliders);
                if (atStepOverlaps <= 0)
                {
                    // Check for outer hit slope normal stability at the step position
                    if (CharacterCollisionsRaycast(
                            farthestHit.point + (characterUp * SecondaryProbesVertical) + (-innerHitDirection * SecondaryProbesHorizontal),
                            -characterUp,
                            maxStepHeight + SecondaryProbesVertical,
                            out var outerSlopeHit,
                            _internalCharacterHits,
                            true) > 0)
                    {
                        if (IsStableOnNormal(outerSlopeHit.normal))
                        {
                            // Cast upward to detect any obstructions to moving there
                            if (CharacterCollisionsSweep(
                                                characterPosition, // position
                                                characterRotation, // rotation
                                                characterUp, // direction
                                                maxStepHeight - farthestHit.distance, // distance
                                                out _, // closest hit
                                                _internalCharacterHits) // all hits
                                    <= 0)
                            {
                                // Do inner step check...
                                var innerStepValid = false;
                                RaycastHit innerStepHit;

                                if (allowSteppingWithoutStableGrounding)
                                {
                                    innerStepValid = true;
                                }
                                else
                                {
                                    // At the capsule center at the step height
                                    if (CharacterCollisionsRaycast(
                                            characterPosition + Vector3.Project((characterPositionAtHit - characterPosition), characterUp),
                                            -characterUp,
                                            maxStepHeight,
                                            out innerStepHit,
                                            _internalCharacterHits,
                                            true) > 0)
                                    {
                                        if (IsStableOnNormal(innerStepHit.normal))
                                        {
                                            innerStepValid = true;
                                        }
                                    }
                                }

                                if (!innerStepValid)
                                {
                                    // At inner step of the step point
                                    if (CharacterCollisionsRaycast(
                                            farthestHit.point + (innerHitDirection * SecondaryProbesHorizontal),
                                            -characterUp,
                                            maxStepHeight,
                                            out innerStepHit,
                                            _internalCharacterHits,
                                            true) > 0)
                                    {
                                        if (IsStableOnNormal(innerStepHit.normal))
                                        {
                                            innerStepValid = true;
                                        }
                                    }
                                }

                                // Final validation of step
                                if (innerStepValid)
                                {
                                    hitCollider = farthestHit.collider;
                                    return true;
                                }
                            }
                        }
                    }
                }

                // Discard hit if not valid step
                nbStepHits--;
                if (farthestIndex < nbStepHits)
                {
                    _internalCharacterHits[farthestIndex] = _internalCharacterHits[nbStepHits];
                }
            }

            return false;
        }

        /// <summary>
        /// Get true linear velocity (taking into account rotational velocity) on a given point of a rigidbody
        /// </summary>
        public void GetVelocityFromRigidbodyMovement(Rigidbody interactiveRigidbody, Vector3 atPoint, float deltaTime, out Vector3 linearVelocity, out Vector3 angularVelocity)
        {
            if (deltaTime > 0f)
            {
                linearVelocity = interactiveRigidbody.linearVelocity;
                angularVelocity = interactiveRigidbody.angularVelocity;
                if(interactiveRigidbody.isKinematic)
                {
                    var physicsMover = interactiveRigidbody.GetComponent<PhysicsMover>();
                    if (physicsMover)
                    {
                        linearVelocity = physicsMover.Velocity;
                        angularVelocity = physicsMover.AngularVelocity;
                    }
                }

                if (angularVelocity != Vector3.zero)
                {
                    var centerOfRotation = interactiveRigidbody.transform.TransformPoint(interactiveRigidbody.centerOfMass);

                    var centerOfRotationToPoint = atPoint - centerOfRotation;
                    var rotationFromInteractiveRigidbody = Quaternion.Euler(angularVelocity * (Mathf.Rad2Deg * deltaTime));
                    var finalPointPosition = centerOfRotation + (rotationFromInteractiveRigidbody * centerOfRotationToPoint);
                    linearVelocity += (finalPointPosition - atPoint) / deltaTime;
                }
            }
            else
            {
                linearVelocity = default;
                angularVelocity = default;
            }
        }

        /// <summary>
        /// Determines if a collider has an attached interactive rigidbody
        /// </summary>
        private Rigidbody GetInteractiveRigidbody(Collider onCollider)
        {
            var colliderAttachedRigidbody = onCollider.attachedRigidbody;
            if (colliderAttachedRigidbody)
            {
                if (colliderAttachedRigidbody.gameObject.GetComponent<PhysicsMover>())
                {
                    return colliderAttachedRigidbody;
                }

                if (!colliderAttachedRigidbody.isKinematic)
                {
                    return colliderAttachedRigidbody;
                }
            }
            return null;
        }

        /// <summary>
        /// Calculates the velocity required to move the character to the target position over a specific deltaTime.
        /// Useful for when you wish to work with positions rather than velocities in the UpdateVelocity callback 
        /// </summary>
        public Vector3 GetVelocityForMovePosition(Vector3 fromPosition, Vector3 toPosition, float deltaTime)
        {
            return GetVelocityFromMovement(toPosition - fromPosition, deltaTime);
        }

        public Vector3 GetVelocityFromMovement(Vector3 movement, float deltaTime)
        {
            if (deltaTime <= 0f)
                return Vector3.zero;

            return movement / deltaTime;
        }

        /// <summary>
        /// Detect if the character capsule is overlapping with anything collidable
        /// </summary>
        /// <returns> Returns number of overlaps </returns>
        public int CharacterCollisionsOverlap(Vector3 position, Quaternion rotation, Collider[] overlappedColliders, float inflate = 0f, bool acceptOnlyStableGroundLayer = false)
        {
            int queryLayers = _collidableLayers;
            if (acceptOnlyStableGroundLayer)
            {
                queryLayers = _collidableLayers & stableGroundLayers;
            }

            var bottom = position + (rotation * _characterTransformToCapsuleBottomHemi);
            var top = position + (rotation * _characterTransformToCapsuleTopHemi);
            if (inflate != 0f)
            {
                bottom += (rotation * Vector3.down * inflate);
                top += (rotation * Vector3.up * inflate);
            }

            var nbUnfilteredHits = Physics.OverlapCapsuleNonAlloc(
                        bottom,
                        top,
                        capsule.radius + inflate,
                        overlappedColliders,
                        queryLayers,
                        QueryTriggerInteraction.Ignore);

            // Filter out invalid colliders
            var nbHits = nbUnfilteredHits;
            for (var i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                if (!CheckIfColliderValidForCollisions(overlappedColliders[i]))
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        overlappedColliders[i] = overlappedColliders[nbHits];
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Detect if the character capsule is overlapping with anything
        /// </summary>
        /// <returns> Returns number of overlaps </returns>
        public int CharacterOverlap(Vector3 position, Quaternion rotation, Collider[] overlappedColliders, LayerMask layers, QueryTriggerInteraction triggerInteraction, float inflate = 0f)
        {
            var bottom = position + (rotation * _characterTransformToCapsuleBottomHemi);
            var top = position + (rotation * _characterTransformToCapsuleTopHemi);
            if (inflate != 0f)
            {
                bottom += (rotation * Vector3.down * inflate);
                top += (rotation * Vector3.up * inflate);
            }

            var nbUnfilteredHits = Physics.OverlapCapsuleNonAlloc(
                        bottom,
                        top,
                        capsule.radius + inflate,
                        overlappedColliders,
                        layers,
                        triggerInteraction);

            // Filter out the character capsule itself
            var nbHits = nbUnfilteredHits;
            for (var i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                if (overlappedColliders[i] == capsule)
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        overlappedColliders[i] = overlappedColliders[nbHits];
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Sweeps the capsule's volume to detect collision hits
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        public int CharacterCollisionsSweep(Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit, RaycastHit[] hits, float inflate = 0f, bool acceptOnlyStableGroundLayer = false)
        {
            int queryLayers = _collidableLayers;
            if (acceptOnlyStableGroundLayer)
            {
                queryLayers = _collidableLayers & stableGroundLayers;
            }

            var bottom = position + (rotation * _characterTransformToCapsuleBottomHemi) - (direction * SweepProbingBackstepDistance);
            var top = position + (rotation * _characterTransformToCapsuleTopHemi) - (direction * SweepProbingBackstepDistance);
            if (inflate != 0f)
            {
                bottom += (rotation * Vector3.down * inflate);
                top += (rotation * Vector3.up * inflate);
            }

            // Capsule cast
            var nbUnfilteredHits = Physics.CapsuleCastNonAlloc(
                    bottom,
                    top,
                    capsule.radius + inflate,
                    direction,
                    hits,
                    distance + SweepProbingBackstepDistance,
                    queryLayers,
                    QueryTriggerInteraction.Ignore);

            // Hits filter
            closestHit = new RaycastHit();
            var closestDistance = Mathf.Infinity;
            var nbHits = nbUnfilteredHits;
            for (var i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                hits[i].distance -= SweepProbingBackstepDistance;

                var hit = hits[i];
                var hitDistance = hit.distance;

                // Filter out the invalid hits
                if (hitDistance <= 0f || !CheckIfColliderValidForCollisions(hit.collider))
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits];
                    }
                }
                else
                {
                    // Remember closest valid hit
                    if (hitDistance < closestDistance)
                    {
                        closestHit = hit;
                        closestDistance = hitDistance;
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Sweeps the capsule's volume to detect hits
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        public int CharacterSweep(Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit, RaycastHit[] hits, LayerMask layers, QueryTriggerInteraction triggerInteraction, float inflate = 0f)
        {
            closestHit = new RaycastHit();

            var bottom = position + (rotation * _characterTransformToCapsuleBottomHemi);
            var top = position + (rotation * _characterTransformToCapsuleTopHemi);
            if (inflate != 0f)
            {
                bottom += (rotation * Vector3.down * inflate);
                top += (rotation * Vector3.up * inflate);
            }

            // Capsule cast
            var nbUnfilteredHits = Physics.CapsuleCastNonAlloc(
                bottom,
                top,
                capsule.radius + inflate,
                direction,
                hits,
                distance,
                layers,
                triggerInteraction);

            // Hits filter
            var closestDistance = Mathf.Infinity;
            var nbHits = nbUnfilteredHits;
            for (var i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                var hit = hits[i];

                // Filter out the character capsule
                if (hit.distance <= 0f || hit.collider == capsule)
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits];
                    }
                }
                else
                {
                    // Remember closest valid hit
                    var hitDistance = hit.distance;
                    if (hitDistance < closestDistance)
                    {
                        closestHit = hit;
                        closestDistance = hitDistance;
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Casts the character volume in the character's downward direction to detect ground
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        private bool CharacterGroundSweep(Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit)
        {
            closestHit = new RaycastHit();

            // Capsule cast
            var nbUnfilteredHits = Physics.CapsuleCastNonAlloc(
                position + (rotation * _characterTransformToCapsuleBottomHemi) - (direction * GroundProbingBackstepDistance),
                position + (rotation * _characterTransformToCapsuleTopHemi) - (direction * GroundProbingBackstepDistance),
                capsule.radius,
                direction,
                _internalCharacterHits,
                distance + GroundProbingBackstepDistance,
                _collidableLayers & stableGroundLayers,
                QueryTriggerInteraction.Ignore);

            // Hits filter
            var foundValidHit = false;
            var closestDistance = Mathf.Infinity;
            for (var i = 0; i < nbUnfilteredHits; i++)
            {
                var hit = _internalCharacterHits[i];
                var hitDistance = hit.distance;

                // Find the closest valid hit
                if (hitDistance > 0f && CheckIfColliderValidForCollisions(hit.collider))
                {
                    if (hitDistance < closestDistance)
                    {
                        closestHit = hit;
                        closestHit.distance -= GroundProbingBackstepDistance;
                        closestDistance = hitDistance;

                        foundValidHit = true;
                    }
                }
            }

            return foundValidHit;
        }

        /// <summary>
        /// Raycasts to detect collision hits
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        public int CharacterCollisionsRaycast(Vector3 position, Vector3 direction, float distance, out RaycastHit closestHit, RaycastHit[] hits, bool acceptOnlyStableGroundLayer = false)
        {
            int queryLayers = _collidableLayers;
            if (acceptOnlyStableGroundLayer)
            {
                queryLayers = _collidableLayers & stableGroundLayers;
            }

            // Raycast
            var nbUnfilteredHits = Physics.RaycastNonAlloc(
                position,
                direction,
                hits,
                distance,
                queryLayers,
                QueryTriggerInteraction.Ignore);

            // Hits filter
            closestHit = new RaycastHit();
            var closestDistance = Mathf.Infinity;
            var nbHits = nbUnfilteredHits;
            for (var i = nbUnfilteredHits - 1; i >= 0; i--)
            {
                var hit = hits[i];
                var hitDistance = hit.distance;

                // Filter out the invalid hits
                if (hitDistance <= 0f ||
                    !CheckIfColliderValidForCollisions(hit.collider))
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits];
                    }
                }
                else
                {
                    // Remember closest valid hit
                    if (hitDistance < closestDistance)
                    {
                        closestHit = hit;
                        closestDistance = hitDistance;
                    }
                }
            }

            return nbHits;
        }
    }
}