using UnityEngine;
using UnityEngine.InputSystem;

namespace Player
{
    public class RealPlayerManager : MonoBehaviour
    {
        [Header("Camera")]
        public FirstPersonCamera playerCamera;
        public Transform cameraFollowPoint;
        public float lookSensitivity = 1f;
        
        private const float SensitivityReduction = 0.1f;

        [Header("Controller")]
        public CharacterController character;

        private PlayerInput _input;

        private IInputObserver _characterObserver;
        private IInputObserver _cameraObserver;

        private void Awake()
        {
            playerCamera.SetFollow(cameraFollowPoint);
            _characterObserver = character;
            _cameraObserver = playerCamera;
        }

        private void OnLook(InputValue value)
        {
            _input.AddViewAngle(value.Get<Vector2>(), lookSensitivity * SensitivityReduction);
            _characterObserver.OnInputUpdate(_input);
            _cameraObserver.OnInputUpdate(_input);
        }

        private void OnMove(InputValue value)
        {
            _input.MovementInput = value.Get<Vector2>();
            _characterObserver.OnInputUpdate(_input);
        }
    }
}