using UGizmo;
using UnityEngine;

namespace Player
{
    public class PlayerController : MonoBehaviour
    {
        public Camera playerCamera;
        public float gravity = -0.35f;
        public float accelerationGround = 1.2f;
        public float accelerationAir = 1.4f;
        public float airStopSpeed = 0.15f;
        public float maxVelocityGround = 0.1f;
        public float groundFriction = 8f;
        public float airFriction = 4f;
        public float groundCheckDistance = 0.05f;
        public float groundCheckDistanceInAir = 0.07f;
        public LayerMask groundCheckLayers = -1;
        public float jumpForce = 0.1f;

        private PlayerInputHandler _inputHandler;
        private CharacterController _characterController;

        public Vector3 velocity = Vector3.zero;

        private void Awake()
        {
            _inputHandler = GetComponent<PlayerInputHandler>();
            _characterController = GetComponent<CharacterController>();
        }

        private void Update()
        {
            // Visually move the Camera
            playerCamera.transform.rotation =
                Quaternion.Euler(new Vector3(-_inputHandler.VirtualViewAngle.y, _inputHandler.VirtualViewAngle.x, 0f));
        }

        private void FixedUpdate()
        {
            Debug.Log(velocity.magnitude);
            var acceleration = Vector3.zero;
            GroundCheck();
            var jumped = _inputHandler.ConsumeJump();

            var horizontalMovement = new Vector3(velocity.x, 0f, velocity.z);

            if (IsGrounded)
            {
                // reset down
                velocity.y = 0f;

                // ground movement
                acceleration += _inputHandler.RequestedMovementDirection * accelerationGround;

                // ground friction
                acceleration += -horizontalMovement.normalized * (horizontalMovement.magnitude * groundFriction);

                HasJumpedThisFrame = false;
                if (jumped)
                {
                    acceleration.y = jumpForce;
                    // set state
                    HasJumpedThisFrame = true;
                    _lastTimeJumped = Time.time;
                    IsGrounded = false;
                    _groundNormal = Vector3.up;
                }
            }
            else
            {
                // air friction
                // acceleration += -horizontalMovement.normalized * (horizontalMovement.magnitude * airFriction);
                
                // gravity, has to be last since slope sliding can do a completely different velocity
                var gravityForce = Vector3.up * gravity;
                var nextVelocity = velocity + (acceleration + gravityForce) * Time.deltaTime;
                Debug.DrawRay(gameObject.transform.position, nextVelocity, Color.green);
                
                // before adding gravity to the total acceleration do the slope check
                
                var slidingCastHit = Physics.CapsuleCast(GetCapsuleBottomHemisphere(),
                    GetCapsuleTopHemisphere(_characterController.height), _characterController.radius, nextVelocity.normalized,
                    out var hit, nextVelocity.magnitude, groundCheckLayers);
                UGizmos.DrawCapsuleCast(GetCapsuleBottomHemisphere(),
                    GetCapsuleTopHemisphere(_characterController.height), _characterController.radius, nextVelocity.normalized,
                    nextVelocity.magnitude, slidingCastHit, hit);
                // var slidingCastHit = Physics.Raycast(GetCapsuleBottomHemisphere(), nextVelocity.normalized, out var hit,
                //     nextVelocity.magnitude + _characterController.radius, groundCheckLayers);
                // UGizmos.DrawRaycast(GetCapsuleBottomHemisphere(), nextVelocity.normalized,
                //     nextVelocity.magnitude + _characterController.radius, slidingCastHit, hit);

                if (slidingCastHit)
                {
                    // split velocity from the slope hit into orthogonal and parrallel component
                    var parrallelVelocity = Vector3.ProjectOnPlane(nextVelocity, hit.normal);
                    var orthogonalVelocity = Vector3.Project(nextVelocity, hit.normal);

                    // use the transform where we hit the slope and apply orthogonalVel from there
                    var hitTransform = nextVelocity.normalized * hit.distance;
                    
                    var adjustedVelocity = hitTransform + parrallelVelocity;
                    adjustedVelocity *= parrallelVelocity.magnitude / adjustedVelocity.magnitude;
                    
                    Debug.DrawRay(gameObject.transform.position, hitTransform, Color.cyan);
                    Debug.DrawRay(gameObject.transform.position, nextVelocity, Color.green);
                    Debug.DrawRay(gameObject.transform.position, parrallelVelocity, Color.blue);
                    Debug.DrawRay(gameObject.transform.position, orthogonalVelocity, Color.red);
                    Debug.DrawRay(gameObject.transform.position, adjustedVelocity, Color.magenta);
                    Debug.DrawRay(gameObject.transform.position + parrallelVelocity, hitTransform, Color.cyan);

                    // down sliding
                    if (Vector3.Dot(Vector3.down, adjustedVelocity) > 0f)
                    {
                        var feetFoundGround = Physics.Raycast(gameObject.transform.position, Vector3.down, out var feetHit);
                        var nextFeetFoundGround = Physics.Raycast(gameObject.transform.position + adjustedVelocity, Vector3.down, out var nextFeetHit);
                        if (feetFoundGround && nextFeetFoundGround)
                        {
                            if (IsNormalUnderSlopeLimit(feetHit.normal) && IsNormalUnderSlopeLimit(nextFeetHit.normal))
                            {
                                goto slidingDownNeverHappened;
                            }
                        }
                    }
                    
                    velocity = adjustedVelocity;
                    
                    _characterController.Move(velocity);
                    velocity += gravityForce * Time.deltaTime;
                    
                    UGizmos.DrawWireCapsule(GetCapsuleBottomHemisphere(), GetCapsuleTopHemisphere(_characterController.height), _characterController.radius, Color.gray);
                    return;
                }
                slidingDownNeverHappened:

                acceleration += gravityForce;
            }
            
            velocity += acceleration * Time.fixedDeltaTime;
            Debug.DrawRay(gameObject.transform.position, velocity, Color.magenta);
            _characterController.Move(velocity);
        }

        private Vector3 Jump(Vector3 velocity, bool jumped)
        {
            velocity = new Vector3(velocity.x, 0f, velocity.z);
            HasJumpedThisFrame = false;

            velocity += Vector3.up * jumpForce;

            _lastTimeJumped = Time.time;
            HasJumpedThisFrame = true;

            IsGrounded = false;
            _groundNormal = Vector3.up;

            return velocity;
        }

        public bool HasJumpedThisFrame { get; private set; }

        private Vector3 Gravity(Vector3 velocity)
        {
            return transform.up * gravity;
        }

        private Vector3 GroundMovement(Vector3 acceleration)
        {
            var desiredDirection = _inputHandler.RequestedMovementDirection;
            var currentVelocity = ApplyFriction(acceleration, groundFriction);

            var projection = Vector3.Dot(currentVelocity, desiredDirection);
            var magnitudeAlongTarget = Mathf.Abs(projection);
            var addSpeed = accelerationGround * Time.fixedDeltaTime;

            // how much velocity can we add in this frame
            var availableAcceleration = maxVelocityGround - addSpeed;

            // is magnitude smaller than the amount we can move
            if (magnitudeAlongTarget < availableAcceleration)
            {
                // add the velocity along the target
                currentVelocity += desiredDirection * addSpeed;
            }

            // is magnitude greater than what we can accelerate && are we under the speed limit
            if (availableAcceleration <= magnitudeAlongTarget && magnitudeAlongTarget < maxVelocityGround)
            {
                // only add velocity to hit the max velocity
                currentVelocity += desiredDirection * (maxVelocityGround - magnitudeAlongTarget);
            }

            // currentVelocity = Vector3.ClampMagnitude(currentVelocity, maxVelocityGround);

            acceleration.x = currentVelocity.x;
            acceleration.z = currentVelocity.z;
            return acceleration;
        }

        private Vector3 ApplyFriction(Vector3 velocity, float friction)
        {
            var currentVelocity = new Vector3(velocity.x, 0, velocity.z);

            var currentSpeed = currentVelocity.magnitude;
            if (currentSpeed > 0.0001f)
            {
                // when we are moving, calculate the speed reduction using friction
                // using the simplified friction model
                var speedReduction = currentSpeed * friction * Time.fixedDeltaTime;

                // normalize the vector by dividing by the current speed, and size it to the new speed
                var newSpeed = Mathf.Max(currentSpeed - speedReduction, 0f);
                currentVelocity *= newSpeed / currentSpeed;
            }

            return currentVelocity;
        }

        private void GroundCheck()
        {
            // Make sure that the ground check distance while already in air is very small, to prevent suddenly snapping to ground
            float chosenGroundCheckDistance =
                IsGrounded ? (_characterController.skinWidth + groundCheckDistance) : groundCheckDistanceInAir;

            // reset values before the ground check
            IsGrounded = false;
            IsGroundedOnSlope = false;
            _groundNormal = Vector3.up;

            // only try to detect ground if it's been a short amount of time since last jump; otherwise we may snap to the ground instantly after we try jumping
            if (Time.time >= _lastTimeJumped + KJumpGroundingPreventionTime)
            {
                var hasCastHit = Physics.CapsuleCast(GetCapsuleBottomHemisphere(),
                    GetCapsuleTopHemisphere(_characterController.height),
                    _characterController.radius, Vector3.down, out var hit, chosenGroundCheckDistance,
                    groundCheckLayers,
                    QueryTriggerInteraction.Ignore);

                // UGizmos.DrawCapsuleCast(GetCapsuleBottomHemisphere(),
                //     GetCapsuleTopHemisphere(_characterController.height),
                //     _characterController.radius, Vector3.down, chosenGroundCheckDistance, hasCastHit, hit);

                // if we're grounded, collect info about the ground normal with a downward capsule cast representing our character capsule
                if (hasCastHit)
                {
                    // storing the upward direction for the surface found
                    _groundNormal = hit.normal;
                    IsGroundedOnSlope = true;

                    // Only consider this a valid ground hit if the ground normal goes in the same direction as the character up
                    // and if the slope angle is lower than the character controller's limit

                    // Debug.Log(Vector3.Dot(hit.normal, transform.up) > 0f &&
                    //           IsNormalUnderSlopeLimit(_groundNormal));
                    if (Vector3.Dot(hit.normal, transform.up) > 0f &&
                        IsNormalUnderSlopeLimit(_groundNormal))
                    {
                        IsGrounded = true;

                        // handle snapping to the ground
                        if (hit.distance - 0.0001 > _characterController.skinWidth)
                        {
                            // Debug.Log($"Snap! {hit.distance} {_characterController.skinWidth}");
                            _characterController.Move(Vector3.down * hit.distance);
                        }
                    }
                }
            }
        }

        private Vector3 GetCapsuleBottomHemisphere()
        {
            return gameObject.transform.position + (gameObject.transform.up * _characterController.radius);
        }

        private Vector3 GetCapsuleTopHemisphere(float atHeight)
        {
            return gameObject.transform.position + (transform.up * (atHeight - _characterController.radius));
        }

        private bool IsNormalUnderSlopeLimit(Vector3 normal)
        {
            return Vector3.Angle(gameObject.transform.up, normal) < _characterController.slopeLimit;
        }

        public bool IsGrounded { get; set; }
        public bool IsGroundedOnSlope { get; set; }
        private Vector3 _groundNormal;
        private float _lastTimeJumped;
        private const float KJumpGroundingPreventionTime = 0.2f;
    }
}