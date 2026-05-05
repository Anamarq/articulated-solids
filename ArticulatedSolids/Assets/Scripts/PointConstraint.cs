using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic point constraint between two rigid bodies.
/// </summary>
public class PointConstraint : MonoBehaviour, IConstraint
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public PointConstraint()
    {
        Manager = null;
    }

    #region EditorVariables

    public float Stiffness;

    public RigidBody bodyA;
    public RigidBody bodyB;

    #endregion

    #region OtherVariables

    int index;
    private PhysicsManager Manager;

    protected Vector3 pointA;
    protected Vector3 pointB;

    #endregion

    #region MonoBehaviour

    // Update is called once per frame
    void Update()
    {
        // Compute the average position
        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        Vector3 pos = 0.5f * (posA + posB);

        // Apply the position
        Transform xform = GetComponent<Transform>();
        xform.position = pos;
    }

    #endregion

    #region IConstraint

    public void Initialize(int ind, PhysicsManager m)
    {
        index = ind;
        Manager = m;

        // Initialize local positions. We assume that the object is connected to a Sphere mesh.
        Transform xform = GetComponent<Transform>();
        if (xform == null)
        {
            System.Console.WriteLine("[ERROR] Couldn't find any transform to the constraint");
        }
        else
        {
            System.Console.WriteLine("[TRACE] Succesfully found transform connected to the constraint");
        }

        // Initialize kinematics
        Vector3 pos = xform.position;

        // Local positions on objects
        pointA = (bodyA != null) ? bodyA.PointGlobalToLocal(pos) : pos;
        pointB = (bodyB != null) ? bodyB.PointGlobalToLocal(pos) : pos;

    }

    public int GetNumConstraints()
    {
        return 3;
    }

    public void GetConstraints(VectorXD c)
    {
        VectorXD C = GetC();
        c.SetSubVector(index, 3, C);
    }

    public void GetConstraintJacobian(MatrixXD dcdx)
    {
        MatrixXD J = GetJ();
        if(bodyB==null)
            dcdx.SetSubMatrix(index,  bodyA.index,  J.SubMatrix(0, 3 ,bodyA.index, 6));
        else if (bodyA == null)
            dcdx.SetSubMatrix(index ,  bodyB.index,  J.SubMatrix(0, 3, bodyB.index, 6));
        else if ((bodyA != null) && (bodyB != null))
        {
            dcdx.SetSubMatrix(index, bodyA.index, J.SubMatrix(0, 3, bodyA.index, 6));
            dcdx.SetSubMatrix(index, bodyB.index, J.SubMatrix(0, 3, bodyB.index, 6));
        }


    }

    private VectorXD GetC()
    {
        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;

        Vector3 c = (posA - posB);

        return Utils.ToVectorXD(c);
    }
    private MatrixXD GetJ()
    {
        MatrixXD J = new DenseMatrixXD(3,12);
        MatrixXD dCdxa = new DenseMatrixXD(3);
        MatrixXD dCdta = new DenseMatrixXD(3);
        MatrixXD dCdxb = new DenseMatrixXD(3);
        MatrixXD dCdtb = new DenseMatrixXD(3);
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);


        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;

        if (bodyA != null)
        {
            dCdxa = I;
            dCdta = -Utils.Skew((posA - bodyA.m_pos));
        }
        if (bodyB != null)
        {
            dCdxb = -I;
            dCdtb = Utils.Skew((posB - bodyB.m_pos));
        }

        J.SetSubMatrix(0, 0, dCdxa);
        J.SetSubMatrix(0, 3, dCdta);
        J.SetSubMatrix(0, 6, dCdxb);
        J.SetSubMatrix(0, 9, dCdtb);

        return J;
    }
    

    public void GetForce(VectorXD force)
    {
        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        bool metodoNuevo = true;
        if (metodoNuevo)
        {
            VectorXD C = GetC();
            MatrixXD J = GetJ();
            //Calcular transpuesta de J
            MatrixXD Jt = J.Transpose();
            VectorXD f = -Stiffness * Jt * C;
            if (bodyA != null)
            {
                force.SetSubVector(bodyA.index, 6, force.SubVector(bodyA.index, 6) + f.SubVector(0, 6));
            }
            if (bodyB != null)
            {
                force.SetSubVector(bodyB.index, 6, force.SubVector(bodyB.index, 6) + f.SubVector(6, 6));
            }
        }
        else
        {

            Vector3 f = -Stiffness * (posA - posB);

            if (bodyA != null)
            {
                force.SetSubVector(bodyA.index, 3, force.SubVector(bodyA.index, 3) + Utils.ToVectorXD(f));
                force.SetSubVector(bodyA.index + 3, 3, force.SubVector(bodyA.index + 3, 3)
                    + Utils.ToVectorXD(Vector3.Cross(posA - bodyA.m_pos, f)));
            }
            if (bodyB != null)
            {
                force.SetSubVector(bodyB.index, 3, force.SubVector(bodyB.index, 3) + Utils.ToVectorXD(-f));
                force.SetSubVector(bodyB.index + 3, 3, force.SubVector(bodyB.index + 3, 3)
                    + Utils.ToVectorXD(Vector3.Cross(posB - bodyB.m_pos, -f)));
            }
        }
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        MatrixXD J = GetJ();          // 3 x 12
        MatrixXD Jt = J.Transpose();  // 12 x 3

        MatrixXD K = -Stiffness * (Jt * J); // 12 x 12

        if (bodyA != null && bodyB != null)
        {
            dFdx.SetSubMatrix(bodyA.index, bodyA.index,
                dFdx.SubMatrix(bodyA.index, 6, bodyA.index, 6)
                + K.SubMatrix(0, 6, 0, 6));

            dFdx.SetSubMatrix(bodyA.index, bodyB.index,
                dFdx.SubMatrix(bodyA.index, 6, bodyB.index, 6)
                + K.SubMatrix(0, 6, 6, 6));

            dFdx.SetSubMatrix(bodyB.index, bodyA.index,
                dFdx.SubMatrix(bodyB.index, 6, bodyA.index, 6)
                + K.SubMatrix(6, 6, 0, 6));

            dFdx.SetSubMatrix(bodyB.index, bodyB.index,
                dFdx.SubMatrix(bodyB.index, 6, bodyB.index, 6)
                + K.SubMatrix(6, 6, 6, 6));
        }
        else if (bodyA != null)
        {
            dFdx.SetSubMatrix(bodyA.index, bodyA.index,
                dFdx.SubMatrix(bodyA.index, 6, bodyA.index, 6)
                + K.SubMatrix(0, 6, 0, 6));
        }
        else if (bodyB != null)
        {
            dFdx.SetSubMatrix(bodyB.index, bodyB.index,
                dFdx.SubMatrix(bodyB.index, 6, bodyB.index, 6)
                + K.SubMatrix(6, 6, 6, 6));
        }
    }

    #endregion

}
