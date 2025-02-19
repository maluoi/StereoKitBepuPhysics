using StereoKit;
using System;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;

using Matrix = StereoKit.Matrix;
using Mesh   = StereoKit.Mesh;
using Box    = BepuPhysics.Collidables.Box;

public static class Program
{
	public static void Main(string[] args)
	{
		// Initialize StereoKit
		if (!SK.Initialize(new SKSettings {
				appName      = "StereoKit Physics",
				assetsFolder = "Assets",
				origin       = OriginMode.Floor,
			})) return;

		Pose windowPose = new Pose(0,1.5f,-0.5f, Quat.LookDir(0,0,1));

		// Setup physics
		BufferPool       pool       = new();
		Simulation       sim        = Simulation.Create(pool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks(new Vec3(0, -10, 0)), new SolveDescription(2, 1));
		ThreadDispatcher dispatcher = new(Environment.ProcessorCount);

		// Make a solid floor
		Material floorMaterial = new Material("floor.hlsl");
		floorMaterial.Transparency = Transparency.Blend;
		Vec3 floorSize = new(200,20,200);
		Vec3 floorPos  = new(0,-10,0);
		sim.Statics.Add(new StaticDescription(floorPos, Quat.Identity, sim.Shapes.Add(new Box(floorSize.x, floorSize.y, floorSize.z))));

		// Variables for the UI and experience
		float boxSize    = 0.1f;
		float layerCount = 2;
		float velocityIt = sim.Solver.VelocityIterationCount;
		float substeps   = sim.Solver.SubstepCount;
		float simFPS     = 60;
		List<BodyHandle> bodies = new();

		double     simTime = 0;
		float      simStep = 1.0f/simFPS;

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
			if (Device.DisplayBlend == DisplayBlend.Opaque)
				Mesh.Cube.Draw(floorMaterial, Matrix.TS(floorPos, floorSize));
			
			// Get the positions from physics, and draw the spheres
			foreach(var body in bodies) {
				var desc = sim.Bodies.GetDescription(body);
				Mesh.Cube.Draw(Material.Default, Matrix.TRS(desc.Pose.Position, desc.Pose.Orientation, boxSize));
			}

			// UI for working with the simulation
			UI.WindowBegin("Simulation", ref windowPose);

			// Spawn a grid of boxes
			if (UI.Button("Spawn Objects")) {
				Box         box        = new (boxSize, boxSize, boxSize);
				BodyInertia boxInertia = box.ComputeInertia(1);
				TypedIndex  boxIdx     = sim.Shapes.Add(box);
				for	(int x=-1; x<=1; x+=1) { for (int y=-1; y<=1; y+=1) { for (int z=0; z<=layerCount; z+=1) {
					bodies.Add(sim.Bodies.Add(BodyDescription.CreateDynamic(
						new RigidPose(new Vec3(x, z*2, y) * boxSize + new Vec3(0,4,-2), Quat.Identity),
						boxInertia,
						new CollidableDescription(boxIdx, 0.1f), new BodyActivityDescription(0.01f))));
				}}}
			}
			// Let the user configure ths size of the grid
			UI.Label($"Layers: {(int)layerCount}");
			UI.HSlider("Layers", ref layerCount, 0, 10, 1);

			// And let them clear things out
			if (UI.Button("Clear")) {
				for (int i=0; i<bodies.Count; i+=1) {
					sim.Bodies.Remove(bodies[i]);
				}
				bodies.Clear();
			}

			// Display some info about the current simulation
			UI.HSeparator();
			
			UI.Label($"Active: {sim.Bodies.ActiveSet.Count}");

			// This method of counting sleeping objects seems to include bodies
			// that have been deleted in some kind of pooling strategy, so it's
			// not accurate after a 'Clear'
			int sleeping = 0;
			for (int i=1; i<sim.Bodies.Sets.Length; i+=1)
				sleeping += sim.Bodies.Sets[i].Count;
			UI.Label($"Sleeping: {sleeping}");

			// Some options for tweaking perf/quality at runtime for testing
			UI.Label($"Velocity iterations: {sim.Solver.VelocityIterationCount}");
			if (UI.HSlider("velocity", ref velocityIt, 1, 8, 1)) sim.Solver.VelocityIterationCount = (int)velocityIt;
			UI.Label($"Substeps: {sim.Solver.SubstepCount}");
			if (UI.HSlider("substeps", ref substeps, 1, 8, 1)) sim.Solver.SubstepCount = (int)substeps;
			UI.Label($"Physics FPS: {simFPS}");
			if (UI.HSlider("fps", ref simFPS, 24, 90, 1)) simStep = 1.0f/simFPS;
			

			UI.WindowEnd();
		});
	}
}