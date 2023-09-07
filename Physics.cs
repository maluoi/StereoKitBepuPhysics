using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;

// Types based on the docs here:
// https://github.com/bepu/bepuphysics2/blob/master/Demos/Demos/SimpleSelfContainedDemo.cs

struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
{
	public void Initialize(Simulation simulation) {	}
	public void Dispose(){}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
		=> a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
	
	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
		=> true;

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
	{
		pairMaterial.FrictionCoefficient = 1f;
		pairMaterial.MaximumRecoveryVelocity = 2f;
		pairMaterial.SpringSettings = new SpringSettings(30, 1);
		return true;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
		=> true;
}

public struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
{
	public void Initialize(Simulation simulation) {}
	public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

	public readonly bool AllowSubstepsForUnconstrainedBodies => false;

	public readonly bool IntegrateVelocityForKinematics => false;

	public Vector3 Gravity;
	private Vector3Wide gravityWideDt;

	public PoseIntegratorCallbacks(Vector3 gravity) : this()
		=> Gravity = gravity;
	
	public void PrepareForIntegration(float dt)
		=> gravityWideDt = Vector3Wide.Broadcast(Gravity * dt);
	
	public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
		=> velocity.Linear += gravityWideDt;
}