using UnityEngine;
using System.Collections;

public class spinner_navigation : MonoBehaviour
{
    private SpinWheel spinWheel;
    public float distance = 5.0f; // The distance the wheel should move
    public float duration = 5.0f; // The time it takes to move the full distance
    private Vector3 startingPosition;
    void Start()
    {
        StartCoroutine(demo());
        // }
    }
    IEnumerator demo()
    {
        startingPosition = transform.position;
        yield return StartCoroutine(Move(0, 0, -10));
        spinWheel = GetComponent<SpinWheel>();

        yield return spinWheel.SpinCycle();
    }
    IEnumerator Move(int x, int y, int z)
    {
        Vector3 endPosition = startingPosition + new Vector3(x, y, z);
        yield return StartCoroutine(MoveOverTime(startingPosition, endPosition, duration));
    }
    IEnumerator MoveOverTime(Vector3 start, Vector3 end, float time)
    {
        float elapsedTime = 0;
        while (elapsedTime < time)
        {
            transform.position = Vector3.Lerp(start, end, (elapsedTime / time));
            elapsedTime += Time.deltaTime;
            yield return null;
        }
        transform.position = end;
    }
}