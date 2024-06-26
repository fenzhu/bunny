﻿using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    float dt = 0.015f;
    /*
    v(0.5) = v(-0.5) + dt * F / M
    x(1) = x(0) + dt * v(0.5) error o(dt^3) < o(dt^2) 
    */
    Vector3 vMid = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0);   // angular velocity


    float mass;                                 // mass
    Matrix4x4 I_ref;                            // reference inertia

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;
    float restitution = 0.5f;                   // for collision


    float uT = 0.5f;

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
        System.Func<Vector3, float> penetrate = (xi) =>
        {
            return Vector3.Dot(xi - P, N);
        };

        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        Vector3 avg = Vector3.zero;
        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);

        int num = 0;
        Vector3 xAvg = Vector3.zero;
        Vector3 vAvg = Vector3.zero;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 Rri = R * vertices[i];
            Vector3 xi = transform.position + Rri;
            if (penetrate(xi) >= 0)
            {
                continue;
            }
            Vector3 vi = vMid + Vector3.Cross(w, Rri);
            if (Vector3.Dot(vi, N) >= 0)
            {
                continue;
            }
            num++;
            xAvg += xi;
            vAvg += vi;
        }

        if (num == 0)
        {
            return;
        }


        xAvg /= num;
        vAvg /= num;
        Vector3 RrAvg = xAvg - transform.position;


        Vector3 vNAvg = Vector3.Dot(vAvg, N) * N;
        Vector3 vTAvg = vAvg - vNAvg;

        float a = Mathf.Max(1 - uT * (1 + restitution) * vNAvg.magnitude /
            vTAvg.magnitude, .0f);

        //这里的vNAvg应该和法线同方向
        vNAvg = -restitution * vNAvg;
        vTAvg = a * vTAvg;
        Vector3 vAvgNew = vNAvg + vTAvg;


        Matrix4x4 I = R * I_ref * R.transpose;

        Matrix4x4 RrMatrix = Get_Cross_Matrix(RrAvg);
        Matrix4x4 K = minusMatrix4x4(
            multiplyMatrix4x4(Matrix4x4.identity, (1 / mass)),
            RrMatrix * I.inverse * RrMatrix);

        Vector4 j = K.inverse * (new Vector4(vAvgNew.x, vAvgNew.y, vAvgNew.z, 1)
            - new Vector4(vAvg.x, vAvg.y, vAvg.z, 1));


        impulse = j;
        vMid = vMid + impulse * (1 / mass);
        Vector3 tmp = I.inverse * (Vector3.Cross(RrAvg, impulse));
        w = w + tmp;
        restitution *= 0.1f;
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
            vMid = new Vector3(5f, 2f, 0);
            w = new Vector3(0, 0, 1f);
            launched = true;
        }


        if (!launched)
        {
            return;
        }

        // Part I: Update velocities
        vMid = linear_decay * vMid;

        //dt:delta t  gravity:g
        Vector3 v_mid = vMid + dt * gravity;
        //x^1 = x^0 +  deltaT * v 当v等于v_0.5时误差为deltaT^3，也就是这里的v_mid
        vMid = v_mid;



        // Part II: Collision Impulse
        Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
        Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

        // Part III: Update position & orientation
        //Update linear status
        Vector3 x = transform.position + dt * vMid;
        //Update angular status
        Quaternion rot = transform.rotation;
        w = angular_decay * w;
        Quaternion w_rotted = new Quaternion(dt * w.x / 2, dt * w.y / 2, dt * w.z / 2, 0) * rot;


        // Part IV: Assign to the object
        transform.position = x;
        transform.rotation = new Quaternion(rot.x + w_rotted.x, rot.y + w_rotted.y, rot.z + w_rotted.z,
                rot.w + w_rotted.w);
    }
}
