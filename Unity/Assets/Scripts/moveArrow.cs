using JetBrains.Annotations;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class moveArrow : MonoBehaviour
{
    public float speed = 5f;
    public GameObject targetpos;
    private Renderer arrowRend;

    // Start is called before the first frame update
    void Start()
    {
        arrowRend = GetComponent<Renderer>();
        arrowRend.enabled = false;
    }

    // Update is called once per frame
    void Update()
    {
        arrowRend.enabled = targetpos.GetComponent<Renderer>().enabled;
        //Make the arrow move up and down over the target position
        Vector3 newpos = new Vector3(0, 0.25f + 0.05f * Mathf.Sin(speed * Time.time), 0);
        Vector3 current = targetpos.transform.position;
        transform.position = current + newpos;
    }
}
