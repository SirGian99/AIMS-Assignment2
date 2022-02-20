using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GatlingGun : MonoBehaviour
{
    public bool is_long_range = false;

    private float time_between_updates = 0.5f;
    private float time_of_last_update = 0f;

    // target the gun will aim at
    GameObject go_target;

    public List<GameObject> target_list;

    // Gameobjects need to control rotation and aiming
    public Transform go_baseRotation;
    public Transform go_GunBody;
    public Transform go_barrel;

    // Gun barrel rotation
    public float barrelRotationSpeed;
    float currentRotationSpeed;

    // Distance the turret can aim and fire from
    public float firingRange;

    public string enemy_tag;

    // Particle system for the muzzel flash
    public ParticleSystem muzzelFlash;

    // Used to start and stop the turret firing
    bool canFire = false;

    
    void Start()
    {
        // Set the firing range distance
        if (is_long_range)
        {
            firingRange = firingRange * 1000f;
        }

        this.GetComponent<SphereCollider>().radius = firingRange;
    }

    //void Update3()
    //{
    //    if(Time.time > time_of_last_update + time_between_updates)
    //    {
    //        //Update();
    //        time_of_last_update = Time.time;
    //    }
    //    AimAndFire();



    //}

    void FixedUpdate()
    {
       
        target_list.RemoveAll(item => item == null);
        canFire = false;


        if (target_list.Count > 0)
        {
        
        float closest_distance = firingRange * 2f;
            foreach(GameObject g_obj in target_list)
            {
                //if (g_obj == null)
                //{
                //    target_list.Remove(g_obj);
                //}
                //{
                float distance = (g_obj.transform.position - transform.position).magnitude;
                Vector3 direction = (g_obj.transform.position - transform.position).normalized;

                RaycastHit hit;
                // Does the ray intersect any objects excluding the player layer
                int layer_mask = LayerMask.GetMask("CubeWalls");

                if (Physics.Raycast(transform.position + direction, direction, out hit, distance -1f, layer_mask))
                {
                    Color line_color;
                    if(enemy_tag == "Enemy") //i.e. from car to turret
                    {
                        line_color = Color.red;
                    }
                    else
                    {
                        line_color = Color.blue;
                    }

                    Debug.DrawLine(transform.position, hit.point, line_color);
                    //Debug.DrawLine(transform.position, g_obj.transform.position, Color.yellow);
                    //Debug.Log("Did Hit" + hit.transform.root.gameObject + "aimed for" + g_obj);

                    //bool hit_right_object = (hit.transform.root.position - g_obj.transform.root.position).magnitude < 1f;

                    //bool hit_right_object = (hit.transform.root.gameObject.GetInstanceID() == g_obj.transform.root.gameObject.GetInstanceID());
                    //if (distance < closest_distance && hit_right_object)
                    //{
                    //    closest_distance = distance;
                    //    go_target = g_obj;
                    //    canFire = true;
                    //    //Debug.DrawRay(g_obj.transform.position, -direction, Color.blue);
                    //    Debug.Log("Viable target:" + g_obj);

                    //}

                }
                else
                {
                    Debug.DrawRay(transform.position, direction * 1000, Color.white);
                    //Debug.Log("Did not Hit");
                    if (distance < closest_distance)
                    {
                        closest_distance = distance;
                        go_target = g_obj;
                        canFire = true;
                        //Debug.DrawRay(g_obj.transform.position, -direction, Color.blue);
                        Debug.Log("Viable target:" + g_obj);

                    }
                }
                   
                //}

            }

        }
        //else
        //{
        //    canFire = false;
        //}
        AimAndFire(); // also to wind down if no target

    }

    void OnDrawGizmosSelected()
    {
        // Draw a red sphere at the transform's position to show the firing range
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, firingRange);
    }

    // Detect an Enemy, aim and fire
    void OnTriggerEnter(Collider other)
    {
        if (!target_list.Contains(other.transform.root.gameObject) && other.transform.root.tag == enemy_tag)
        {
            target_list.Add(other.transform.root.gameObject);
        }
        //target_list.Add(other.transform.parent.gameObject);

        //Debug.Log("Enemy nearby...");
        //Debug.Log(other.transform.root.tag);
        //if (other.gameObject.tag == enemy_tag)
        //{
        //    go_target = other.transform;
        //    canFire = true;
        //    //Debug.Log("Enemy nearby...2");

        //}

    }
    // Stop firing
    void OnTriggerExit(Collider other)
    {
        target_list.Remove(other.transform.root.gameObject);
        //if (other.gameObject.tag == enemy_tag)
        //{
        //    canFire = false;
        //}
    }

    void AimAndFire()
    {
        // Gun barrel rotation
        go_barrel.transform.Rotate(0, 0, currentRotationSpeed * Time.deltaTime);

        //Debug.Log("Enemy nearby...3");

        // if can fire turret activates
        if (canFire)
        {
            // start rotation
            currentRotationSpeed = barrelRotationSpeed;

            // aim at enemy
            Vector3 baseTargetPostition = new Vector3(go_target.transform.position.x, this.transform.position.y, go_target.transform.position.z);
            Vector3 gunBodyTargetPostition = new Vector3(go_target.transform.position.x, go_target.transform.position.y, go_target.transform.position.z);

            go_baseRotation.transform.LookAt(baseTargetPostition);
            go_GunBody.transform.LookAt(gunBodyTargetPostition);

            //go_target.Destructable.
            Destructable enemy_script;
            enemy_script = go_target.GetComponent<Destructable>();
            if(enemy_script != null)
            {
                enemy_script.DoDamage(1f);
                if (enemy_tag == "Enemy") //i.e. from car to turret
                {
                    float dist = (go_target.transform.position - this.transform.position).magnitude;
                    enemy_script.DoDamage(Mathf.Clamp(1f - dist/400f, 0f, 1f)); 
                }
            }

            // start particle system 
            if (!muzzelFlash.isPlaying)
            {
                //muzzelFlash.Play();
            }
        }
        else
        {
            // slow down barrel rotation and stop
            currentRotationSpeed = Mathf.Lerp(currentRotationSpeed, 0, 10 * Time.deltaTime);

            // stop the particle system
            if (muzzelFlash.isPlaying)
            {
                muzzelFlash.Stop();
            }
        }
    }
}