using UnityEngine;

public struct CollisionInfo
{
    public bool Hit { get; set; }
    
    public RaycastHit HitInfo { get; set; }
    
    
    public Vector3 NearPoint { get; set; }
}