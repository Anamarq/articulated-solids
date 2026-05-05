
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;


/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		//Paused = true;
		TimeStep = 0.01f;
		Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		IntegrationMethod = Integration.Symplectic;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Symplectic = 1,
        Implicit = 2,
        SymplecticConstraints = 3,
    };

	#region InEditorVariables

	public bool Paused;
	public float TimeStep;
    public Vector3 Gravity;
    public List<GameObject> SimObjects;
    public List<GameObject> Constraints;
    public Integration IntegrationMethod;

    #endregion

    #region OtherVariables

    private List<ISimulable> m_objs;
    private List<IConstraint> m_constraints;
    private int m_numDoFs;
    private int m_numConstraints;

    #endregion

    #region MonoBehaviour

    public void Start()
    {
        //Parse the simulable objects and initialize their state indices
        m_numDoFs = 0;
        m_objs = new List<ISimulable>(SimObjects.Count);

        foreach (GameObject obj in SimObjects)
        {
            ISimulable simobj = obj.GetComponent<ISimulable>();
            if (simobj != null)
            {
                m_objs.Add(simobj);

                // Initialize simulable object
                simobj.Initialize(m_numDoFs, this);

                // Retrieve pos and vel size
                m_numDoFs += simobj.GetNumDoFs();
            }
        }

        //Parse the constraints
        m_numConstraints = 0;
        m_constraints = new List<IConstraint>(Constraints.Count);

        foreach (GameObject obj in Constraints)
        {
            IConstraint constraint = obj.GetComponent<IConstraint>();
            if (constraint != null)
            {
                m_constraints.Add(constraint);

                // Initialize constraint
                constraint.Initialize(m_numConstraints, this);

                // Retrieve the number of constraints
                m_numConstraints += constraint.GetNumConstraints();
            }
        }

    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;

    }

    public void FixedUpdate()
    {
        if (this.Paused)
            return; // Not simulating

        // Select integration method
        switch (this.IntegrationMethod)
        {
            case Integration.Symplectic: this.stepSymplectic(); break;
            case Integration.Implicit: this.stepImplicit(); break;
            case Integration.SymplecticConstraints: this.stepSymplecticConstraints(); break;
            default:
                throw new System.Exception("[ERROR] Should never happen!");
        }
    }

    #endregion

    /// <summary>
    /// Performs a simulation step using Symplectic integration.
    /// </summary>
    private void stepSymplectic()
	{
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        f.Clear();
        Minv.Clear();

        //Recorrer los solidos rigidos y rellenar
        foreach (ISimulable obj in m_objs)
        {
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }
        //Restricciones
        foreach (IConstraint constraint in m_constraints)
        {
            constraint.GetForce(f);
        }

        //No hace falta para esta práctica
        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        v += TimeStep * (Minv * f);
        VectorXD x = new DenseVectorXD(m_numDoFs);

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
        }
        VectorXD dx = TimeStep * v;

        foreach (ISimulable obj in m_objs)
        {
            obj.AdvanceIncrementalPosition(dx);
            obj.SetVelocity(v);
        }

        //foreach (ISimulable obj in m_objs)
        //{
        //    obj.AdvanceIncrementalPosition(x);
        //    obj.SetVelocity(v);
        //}
    }

    /// <summary>
    /// Performs a simulation step using Implicit integration.
    /// </summary>
    private void stepImplicit()
    {
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);

        MatrixXD M = new DenseMatrixXD(m_numDoFs, m_numDoFs);
        MatrixXD dfdx = new DenseMatrixXD(m_numDoFs, m_numDoFs);
        MatrixXD dfdv = new DenseMatrixXD(m_numDoFs, m_numDoFs);

        f.Clear();
        M.Clear();
        dfdx.Clear();
        dfdv.Clear();

        // 1. RECOGER ESTADO
        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMass(M);
            obj.GetForceJacobian(dfdx, dfdv);
        }

        foreach (IConstraint c in m_constraints)
        {
            c.GetForce(f);
            c.GetForceJacobian(dfdx, dfdv);
        }

        // 2. APLICAR RESTRICCIONES (FIXED DOFs)
        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(M);
            obj.FixMatrix(dfdx);
            obj.FixMatrix(dfdv);
        }

        // 3. CONSTRUIR SISTEMA
        MatrixXD A = M - (TimeStep * dfdv + TimeStep * TimeStep * dfdx);
        VectorXD b = M * v + TimeStep * f;

        // 4. RESOLVER
        VectorXD v_new = A.Solve(b);

        // 5. ACTUALIZAR POSICIÓN
        VectorXD dx = TimeStep * v_new;

        foreach (ISimulable obj in m_objs)
        {
            obj.AdvanceIncrementalPosition(dx);
            obj.SetVelocity(v_new);
        }
    }

    /// <summary>
    /// Performs a simulation step using Symplectic integration with constrained dynamics.
    /// The constraints are treated as implicit
    /// </summary>
    private void stepSymplecticConstraints()
    {
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        MatrixXD M = new DenseMatrixXD(m_numDoFs);

        MatrixXD J = new DenseMatrixXD(m_numConstraints, m_numDoFs);
        VectorXD c = new DenseVectorXD(m_numConstraints);

        f.Clear();
        M.Clear();

        // 1. Obtener estado actual
        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMass(M);
        }

        // 2. Obtener constraints
        foreach (IConstraint constraint in m_constraints)
        {
            constraint.GetConstraintJacobian(J);
            constraint.GetConstraints(c);
        }

        // 3. Fix DOFs (si aplica)
        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(M);
        }

        // 4. Construir sistema
        int n = m_numDoFs;
        int m = m_numConstraints;

        MatrixXD A = new DenseMatrixXD(n + m, n + m);
        VectorXD b = new DenseVectorXD(n + m);

        // Bloques
        MatrixXD Jt = J.Transpose();

        // A
        A.SetSubMatrix(0, 0, M);
        A.SetSubMatrix(0, n, -TimeStep * Jt);
        A.SetSubMatrix(n, 0, J);

        // b
        b.SetSubVector(0, n, M * v + TimeStep * f);
        b.SetSubVector(n, m, (-1.0 / TimeStep) * c);

        // 5. Resolver
        VectorXD sol = new DenseVectorXD(n + m);
        A.Solve(b, sol);

        VectorXD v_new = sol.SubVector(0, n);

        // 6. Integrar posición (symplectic)
        VectorXD dx = TimeStep * v_new;

        foreach (ISimulable obj in m_objs)
        {
            obj.AdvanceIncrementalPosition(dx);
            obj.SetVelocity(v_new);
        }
    }

}
