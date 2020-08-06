using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class rotate : MonoBehaviour
{
    public float speed = 25f;
    void Update()
    {
        //Let the Manipulation Target rotate slowly
        transform.Rotate(0, speed * Time.deltaTime, 0);
    }
}
