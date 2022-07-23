using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    float dt = 0.015f;
    Vector3 v = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0);   // angular velocity
    

    float mass;                                 // mass
    Matrix4x4 I_ref;                            // reference inertia

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;
    float restitution = 0.5f;                   // for collision
        

    float uT = 0.5f;
    float uN = 0.5f;

    Vector3 gravity = new Vector3(0, -9.8f, 0);
    Vector3 impulse = Vector3.zero;
    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    //An impulse method assumes that collision changes the 
    //position and the velocity all of sudden.
    void Collision_Impulse(Vector3 P, Vector3 N)
    {

        Vector3 xAvg = transform.position;

            System.Func<Vector3, float> penetrate = (xi) =>
        {
            return Vector3.Dot(xi - P, N);
        };
        if (penetrate(xAvg) >= 0)
        {
            return;
        }

        Vector3 RrAvg = xAvg - transform.position;
        Vector3 vAvg = v + Vector3.Cross(w, RrAvg);


        if (Vector3.Dot(vAvg, N) >= 0)
        {
            return;
        }

        
        Vector3 vNAvg = Vector3.Dot(vAvg, N) * N;
        Vector3 vTAvg = vAvg - vNAvg;

        float a = Mathf.Max(1 - uT * (1 + uN) * vNAvg.magnitude /
            vTAvg.magnitude, .0f);

        //这里的vNAvg应该和法线同方向
        vNAvg = -uN * vNAvg;
        vTAvg = a * vTAvg;
        Vector3 vAvgNew = vNAvg + vTAvg;

        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
        Matrix4x4 I = R * I_ref * R.transpose;

        Matrix4x4 RrMatrix = Get_Cross_Matrix(RrAvg);
        Matrix4x4 K = minusMatrix4x4(
            multiplyMatrix4x4(Matrix4x4.identity, (1 / mass)),
            RrMatrix * I.inverse * RrMatrix);

        Vector4 j = K.inverse * (new Vector4(vAvgNew.x, vAvgNew.y, vAvgNew.z, 1)
            - new Vector4(vAvg.x, vAvg.y, vAvg.z, 1));

        //discard w
        impulse = j;
        Debug.Log("impulse : " +  impulse.ToString());
        v = v + impulse * (1 / mass);
        Vector3 tmp = I.inverse * (Vector3.Cross(RrAvg, impulse));
        w = w + tmp;
    }


    Matrix4x4 multiplyMatrix4x4(Matrix4x4 m, float s)
    {
        for (int i = 0; i <= 3; i++)
        {
            m.SetRow(i, m.GetRow(i) * s);
        }

        return m;
    }

    Matrix4x4 minusMatrix4x4(Matrix4x4 m, Matrix4x4 n)
    {
        for (int i = 0; i <= 3; i++)
        {
            m.SetRow(i, m.GetRow(i) - n.GetRow(i));
        }
        return m;
    }

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
        }
        if (Input.GetKey("l"))
        {
            v = new Vector3(0.5f, 0.2f, 0);
            w = new Vector3(0, 0, 1f);
            launched = true;
        }

        // Part I: Update velocities
        if (launched == true)
        {
            Vector3 x_0 = transform.position;
            v = linear_decay * v;

            //dt:delta t  gravity:F mass:M^-1
            Vector3 v_mid = v + dt * gravity / mass;
            //x^1 = x^0 +  deltaT * v 当v等于v_0.5时误差为deltaT^3 
            Vector3 x_1 = x_0 + dt * v_mid;

            v = v_mid;
            transform.position = new Vector3(x_1.x, x_1.y, x_1.z);

            Quaternion rot = transform.rotation;
            //暂时关闭了角速度衰减
            // w = angular_decay * w;
            Quaternion w_rotted = new Quaternion(dt * w.x / 2, dt * w.y / 2, dt * w.z / 2, 0) * rot;
            transform.rotation = new Quaternion(rot.x + w_rotted.x, rot.y + w_rotted.y, rot.z + w_rotted.z,
            rot.w + w_rotted.w);
        }

        Debug.Log("pos : " + transform.position.ToString());
        // Part II: Collision Impulse
       Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
        Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));
        Debug.Log("after collision pos : " + transform.position.ToString());
        // Part III: Update position & orientation
        //Update linear status
        Vector3 x = transform.position;
        //Update angular status
        Quaternion q = transform.rotation;

        // Part IV: Assign to the object
        transform.position = x;
        transform.rotation = q;
    }
}
