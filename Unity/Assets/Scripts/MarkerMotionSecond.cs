using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// This script can be used instead of the virtual marker to test the follow manipulation
// function and the other related to the manipulation

public class MarkerMotionSecond : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float turnSpeed = 50f;
    public float upSpeed = 5f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(KeyCode.U))
            transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime);

        if (Input.GetKey(KeyCode.J))
            transform.Translate(-Vector3.forward * moveSpeed * Time.deltaTime);

        if (Input.GetKey(KeyCode.H))
            transform.Rotate(Vector3.up, -turnSpeed * Time.deltaTime);

        if (Input.GetKey(KeyCode.K))
            transform.Rotate(Vector3.up, turnSpeed * Time.deltaTime);

        if (Input.GetKey(KeyCode.Z))
            transform.Translate(Vector3.up * upSpeed * Time.deltaTime);

        if (Input.GetKey(KeyCode.I))
            transform.Translate(Vector3.down * upSpeed * Time.deltaTime);
    }
}
