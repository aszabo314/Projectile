namespace Deagle

open System
open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.UI
open Aardvark.UI.Primitives
open Aardvark.Base.Rendering
open Deagle.Model
open BulletSharp
open BulletSharp.Math
open System.Threading.Tasks

type Message =
    | SetBoxTrafo of Trafo3d
    | SetBallTrafo of Trafo3d
    | Bumm
    | Nop
    | CameraMessage of FreeFlyController.Message

module BulletStuff =


    let inline m44 (m : Matrix) =
        M44d(float m.M11, float m.M12, float m.M13, float m.M14, float m.M21, float m.M22, float m.M23, float m.M24, float m.M31, float m.M32, float m.M33, float m.M34, float m.M41, float m.M42, float m.M43, float m.M44).Transposed

    let inline trafo (m : Matrix) =
        let mat = m44 m
        Trafo3d(mat,mat.Inverse)

    let initBullet() =

        let cfg = new DefaultCollisionConfiguration()
        let disp = new CollisionDispatcher(cfg)
        let bp = new DbvtBroadphase()
        let world = new DiscreteDynamicsWorld(disp,bp,null,cfg)
        world.Gravity <- Vector3(0.0f,0.0f,-10.0f)

        let floor = 
            let shape = new BoxShape(1000.0f,1000.0f,1.0f)
            let info = new RigidBodyConstructionInfo(0.0f, new DefaultMotionState(), shape)
            new RigidBody(info)

        let box = 
            let shape = new BoxShape(Vector3(1.0f,1.0f,1.0f))
            let mutable inertia = Vector3()
            shape.CalculateLocalInertia(50.0f, &inertia)
            let state = new DefaultMotionState(Matrix.Translation(0.0f,0.0f,1.0f))
            let info = new RigidBodyConstructionInfo(50.0f, state, shape,inertia)
            let res = new RigidBody(info)
            res.ActivationState <- ActivationState.DisableDeactivation
            res.SetDamping(0.01f,0.01f)
            res

        let ball = 
            let shape = new SphereShape(1.0f)
            let mutable inertia = Vector3()
            shape.CalculateLocalInertia(5.0f, &inertia)
            let state = new DefaultMotionState(Matrix.Translation(2.0f,0.0f,1.0f))
            let info = new RigidBodyConstructionInfo(5.0f, state, shape, inertia)
            let res = new RigidBody(info)
            res.ActivationState <- ActivationState.DisableDeactivation
            res.SetDamping(0.01f,0.01f)
            res

        let spring = new Generic6DofSpring2Constraint(box, ball, Matrix.Identity, Matrix.Identity)

        spring.LinearLowerLimit <- Vector3(5.0f,5.0f,5.0f)
        spring.LinearUpperLimit <- Vector3(10.0f,10.0f,10.0f)
        spring.AngularLowerLimit <- Vector3(0.0f,0.0f,0.0f)
        spring.AngularUpperLimit <- Vector3(1.0f,1.0f,1.0f)

        spring.EnableSpring(1, true)
        spring.SetStiffness(1, 35.0f)
        spring.SetDamping(1, 0.5f)
        spring.EnableSpring(0, true)
        spring.SetStiffness(0, 35.0f)
        spring.SetDamping(0, 0.5f)
        spring.EnableSpring(2, true)
        spring.SetStiffness(2, 35.0f)
        spring.SetDamping(2, 0.5f)
        spring.SetEquilibriumPoint()

        world.AddConstraint(spring)

        world.AddCollisionObject floor
        world.AddRigidBody box
        world.AddRigidBody ball
        ball.ApplyCentralImpulse(Vector3(0.0f,0.0f,1.0f))

        world,ball,box

    let rec simulant (sw : System.Diagnostics.Stopwatch) (world : DiscreteDynamicsWorld) (ball : RigidBody) (box : RigidBody) =
        proclist {    
            sw.Restart()
            do! Async.Sleep(32)
            sw.Stop()    
            if sw.Elapsed.TotalMilliseconds < 100.0 then 
                world.StepSimulation(float32 sw.Elapsed.TotalSeconds) |> ignore
                let balltrafo = trafo ball.MotionState.WorldTransform
                let boxtrafo = trafo box.MotionState.WorldTransform
                yield SetBallTrafo balltrafo
                yield SetBoxTrafo boxtrafo  
            yield! simulant sw world ball box
        }

module App =

    let mutable bball : RigidBody = null
    let mutable bbox : RigidBody = null
    let applyImpulse() =
        let pos = bball.MotionState.WorldTransform.Origin
        let dir = -pos / pos.Length + Vector3(0.0f,0.0f,0.0f)
        bball.ApplyCentralImpulse(Vector3(0.0f,100.0f,0.0f))
        // let pos = bbox.MotionState.WorldTransform.Origin
        // let dir = -pos / pos.Length + Vector3(0.0f,0.0f,0.0f)
        // bbox.ApplyCentralImpulse(dir * 100.0f + Vector3(0.0f,0.0f,10.0f))
        printfn "impulse! %A" dir


    let initial = 
        { 
            ballTrafo = Trafo3d.Identity
            boxTrafo = Trafo3d.Identity
            cameraState = FreeFlyController.initial 
        }

    let update (m : Model) (msg : Message) =
        match msg with
            | SetBallTrafo t ->  { m with ballTrafo = t }
            | SetBoxTrafo t ->  { m with boxTrafo = t }
            | Bumm -> 
                applyImpulse()
                m
            | Nop -> m
            | CameraMessage msg ->
                { m with cameraState = FreeFlyController.update m.cameraState msg }

    let view (m : AdaptiveModel) =
        
        let frustum = 
            Frustum.perspective 90.0 0.1 100.0 1.0 
                |> AVal.constant

        let boxsg =
            Sg.box (AVal.constant C4b.Blue) (AVal.constant (Box3d(-V3d.III, V3d.III)))
            |> Sg.trafo m.boxTrafo

        let ballsg =
            Sg.sphere 5 (AVal.constant C4b.Green) (AVal.constant 1.0)
            |> Sg.trafo m.ballTrafo

        let floorsg =
            let box = Box3d(V3d(-1000.0,-1000.0,-1.0), V3d(1000.0,1000.0,1.0))
            Sg.box (AVal.constant C4b.Gray) (AVal.constant box)

        let sg =
            Sg.ofList [boxsg; ballsg; floorsg]
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.simpleLighting
            }

        let att =
            [
                style "position: fixed; left: 0; top: 0; width: 100%; height: 100%"
            ]

        body [
            onKeyDown (fun k -> match k with Aardvark.Application.Keys.Space -> Bumm | _ -> Nop )
        ] [
            FreeFlyController.controlledControl m.cameraState CameraMessage frustum (AttributeMap.ofList att) sg
        ]

    let app =
        let world,ball,box = BulletStuff.initBullet()
        bball <- ball
        bbox <- box
        let sw = System.Diagnostics.Stopwatch()
        let simulant = BulletStuff.simulant sw world ball box
        let tp = ThreadPool.add "simulant" simulant ThreadPool.empty

        {
            initial = initial
            update = update
            view = view
            threads = fun m -> tp
            unpersist = Unpersist.instance
        }