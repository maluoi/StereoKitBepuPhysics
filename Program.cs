using StereoKit;
using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;

using Matrix = StereoKit.Matrix;
using Mesh   = StereoKit.Mesh;
using Sphere = BepuPhysics.Collidables.Sphere;

// Initialize StereoKit
if (!SK.Initialize(new SKSettings {
		appName      = "StereoKit Physics",
		assetsFolder = "Assets",
		origin	     = OriginMode.Floor,
	})) return;

// Setup physics
BufferPool pool    = new();
Simulation sim     = Simulation.Create(pool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks(new Vec3(0, -10, 0)), new SolveDescription(8, 1));
double     simTime = 0;
float      simStep = 1.0f/90.0f;
ThreadDispatcher dispatcher = new ThreadDispatcher(Environment.ProcessorCount);

// Make a solid floor
Material floorMaterial = new Material("floor.hlsl");
floorMaterial.Transparency = Transparency.Blend;
Vec3 floorSize = new(200,20,200);
Vec3 floorPos  = new(0,-10,0);
sim.Statics.Add(new StaticDescription(floorPos, Quat.Identity, sim.Shapes.Add(new Box(floorSize.x, floorSize.y, floorSize.z))));

// Add some spheres!
float sphereSize = 0.1f;
List<BodyHandle> bodies = new();
var sphere        = new Sphere(sphereSize*0.5f);
var sphereInertia = sphere.ComputeInertia(1);
var sphereIdx     = sim.Shapes.Add(sphere);
for	(int x=-1; x<=1; x+=1) { for (int y=-1; y<=1; y+=1) { for (int z=0; z<=4; z+=1) {
	RigidPose pose = new RigidPose();
	pose.Orientation = Quat.Identity;
	pose.Position    = new Vec3(x , z*2, y) * sphereSize + new Vec3(0,0,-2);
	var sphereDesc = BodyDescription.CreateDynamic(
		pose,
		sphereInertia,
		new CollidableDescription(sphereIdx, 0.1f), new BodyActivityDescription(0.01f));
	bodies.Add(sim.Bodies.Add(sphereDesc));
}}}

// Core application loop
SK.Run(() =>
{
	// Step the physics simulation
	while (simTime < Time.Total)
	{
		sim.Timestep(simStep, dispatcher);
		simTime += simStep;
	}

	// Draw the floor
	if (SK.System.displayType == Display.Opaque)
		Mesh.Cube.Draw(floorMaterial, Matrix.TS(floorPos, floorSize));
	
	// Get the positions from physics, and draw the spheres
	foreach(var body in bodies) {
		var desc = sim.Bodies.GetDescription(body);
		Mesh.Sphere.Draw(Material.Default, Matrix.TRS(desc.Pose.Position, desc.Pose.Orientation, sphereSize));
	}
});