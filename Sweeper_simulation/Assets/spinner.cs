using UnityEngine;
using System.Collections;
using System;
using System.Threading;
public class SpinWheel : MonoBehaviour
{
    public float speed = 0f;
    public float duration = 2.0f; // Duration in seconds
    private bool isSpinning = false;

    // Indicates the spinning direction, 1 for one way, -1 for the opposite
    private int spinDirection = -1; 
    public void Start()
    {
        StartCoroutine(SpinCycle());
    }
    IEnumerator SpinForDuration(float duration)
    {
        isSpinning = true;
        float endTime = Time.time + duration;
        // Keep spinning until the end time is reached
        while (Time.time < endTime)
        {
            transform.Rotate(Vector3.up, speed * Time.deltaTime * spinDirection);
            yield return null;
        }
        // After spinning, reset isSpinning to allow the next spin cycle
        isSpinning = false;
    }
    // Optionally, call this method at the moment you want to start spinning

    public IEnumerator SpinCycle()
    {
        int count = 0;
        while (true)
        {
            // Spin for the duration
            yield return StartCoroutine(SpinForDuration(duration));
            // Stay stationary for the specified duration
            yield return new WaitForSeconds(3.0f);
            // Change the direction of spin
            spinDirection *= -1;
            count ++;
            if (count == 2)
            {
                break;
            }
        }
    }
}