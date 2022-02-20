using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Destructable : MonoBehaviour
{

    public bool is_weak = false;
    public float health = 100f;
    public float start_health = 100f;

    public Image healthBar;

    // Start is called before the first frame update
    void Start()
    {
        //start_health = health;
        health = start_health;
        if (is_weak)
        {
            health = 0.01f * start_health;
        }
        healthBar.fillAmount = health / start_health;

    }

    // Update is called once per frame
    void Update()
    {
        if(health < 0)
        {
            Debug.Log(transform.name + ": I am dying!");
            Destroy(gameObject, 0.1f);
        }
    }

    // To be used to inflict damage to object
    public void DoDamage(float amount)
    {
        health -= amount;
        healthBar.fillAmount = health / start_health;
    }
}
