using UnityEngine;

public class FirstPersonShooter : MonoBehaviour
{
    private void Start()
    {
        HideAndLockMouse();
    }

    private static void HideAndLockMouse()
    {
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }
}