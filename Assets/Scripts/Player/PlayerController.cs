using UnityEngine;

namespace Player
{
    public class PlayerController : MonoBehaviour
    {
        public Camera playerCamera;
        public float gravity = -8f;
        public float accelerationGround = 0.2f;
        public float maxVelocityGround = 0.1f;
        public float friction = 0.2f;

        private PlayerInputHandler _inputHandler;
        private CharacterController _characterController;

        private Vector3 _velocity = Vector3.zero;

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
            var velocity = _velocity;

            if (_inputHandler.ConsumeJump() && _characterController.isGrounded)
            {
                velocity.y = 10f;
            }

            // gravity
            if (!_characterController.isGrounded)
            {
                velocity.y += gravity * Time.fixedDeltaTime;
            }
            else
            {
                velocity.y = 0;
            }

            // movement
            var currentVelocity = new Vector3(velocity.x, 0, velocity.z);
            var desiredDirection = _inputHandler.RequestedMovementDirection;
            
            var currentSpeed = currentVelocity.magnitude;
            if (currentSpeed > 0.001f)
            {
                // when we are moving, calculate the speed reduction using friction
                // using the simplified friction model
                var speedReduction = currentSpeed * friction * Time.fixedDeltaTime;
                
                // normalize the vector by dividing by the current speed, and size it to the new speed
                var newSpeed = Mathf.Max(currentSpeed - speedReduction, 0f);
                currentVelocity *= newSpeed / currentSpeed;
            }

            var projection = Vector3.Dot(currentVelocity, desiredDirection);
            var magnitudeAlongTarget = Mathf.Abs(projection);
            var addSpeed = accelerationGround * Time.fixedDeltaTime;

            // how much velocity can we add in this frame
            var availableAcceleration = maxVelocityGround - addSpeed;

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
            
            currentVelocity = Vector3.ClampMagnitude(currentVelocity, maxVelocityGround);

            velocity.x = currentVelocity.x;
            velocity.z = currentVelocity.z;

            // finishing up
            _velocity = velocity;
            _characterController.Move(_velocity);
            // Debug.Log(_characterController.isGrounded);
            // Debug.Log(_velocity.magnitude);
            // Debug.DrawRay(gameObject.transform.position, _velocity, Color.red);
            // Debug.DrawRay(gameObject.transform.position, _inputHandler.Forward, Color.blue);
            // Debug.DrawRay(gameObject.transform.position, _inputHandler.RequestedMovementDirection, Color.green);
        }
    }
}