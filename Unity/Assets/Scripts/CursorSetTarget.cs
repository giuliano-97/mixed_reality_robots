using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel.Design;
using UnityEngine;
using UnityEngine.XR.WSA.Input;

public class CursorSetTarget : MonoBehaviour
{
    private MeshRenderer meshRenderer;
    public static CursorSetTarget Instance { get; private set; }
    public GameObject FocusedObject { get; private set; }

    private Renderer cursorrend;

    GestureRecognizer recognizer;

    public GameObject target;

    // Start is called before the first frame update
    void Start()
    {
        meshRenderer = this.gameObject.GetComponentInChildren<MeshRenderer>();
        cursorrend = GetComponent<Renderer>();
    }

    // Update is called once per frame
    void Update()
    {
        var headPosition = Camera.main.transform.position;
        var gazeDirection = Camera.main.transform.forward;
        RaycastHit hitInfo;

        if (Physics.Raycast(headPosition, gazeDirection, out hitInfo))
        {
            this.transform.position = hitInfo.point;
            this.transform.rotation = Quaternion.FromToRotation(Vector3.up, hitInfo.normal);
            FocusedObject = hitInfo.collider.gameObject;
        }
        else
        {
            FocusedObject = null;
        }
        if (cursorrend.enabled)
        {
            Instance = this;
            recognizer = new GestureRecognizer();
            recognizer.Tapped += (args) =>
            {
                if (FocusedObject != null)
                {
                    target.transform.position = hitInfo.point;
                    target.GetComponent<Renderer>().enabled = true;
                    cursorrend.enabled = false;
                }
            };
            recognizer.StartCapturingGestures();
        }
    }
}
