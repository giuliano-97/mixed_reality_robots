using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CamearFollow3rd : MonoBehaviour
{
    public Transform target;
    public float minDistance;
    public float height;

    private Transform _aTransform;
    // Start is called before the first frame update
    void Start()
    {
        if(target == null)
            Debug.LogWarning("Target not set!");

        _aTransform = transform;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void LateUpdate()
    {
        _aTransform.position = 
            new Vector3(target.position.x, target.position.y + height, target.position.z - minDistance);
    }
}
