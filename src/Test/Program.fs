﻿// Learn more about F# at http://fsharp.org

open System
open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.Application
open Aardvark.SceneGraph
open Aardvark.Base.Rendering
open System

[<AutoOpen>]
module BoxExtensions =

    type OrientedBox3d with
            
        member x.Intersects(ray : Ray3d, [<System.Runtime.InteropServices.Out>] t : byref<float>) =
            let m : M44d = Euclidean3d.op_Explicit x.Trafo.Inverse
            let r = ray.Transformed m
            x.Box.Intersects(r, &t)

        member x.EnlargedBy(eps : float) =
            OrientedBox3d(x.Box.EnlargedBy eps, x.Trafo)
                
        member x.ShrunkBy(eps : float) =
            OrientedBox3d(x.Box.ShrunkBy eps, x.Trafo)

        member x.Contains (point : V3d, eps : float) =
            let lp = x.Trafo.InvTransformPos point
            x.Box.EnlargedBy(eps).Contains(lp)
                
        member x.Contains (point : V3d) =
            let lp = x.Trafo.InvTransformPos point
            x.Box.Contains(lp)

        member x.OutsideFlags (point : V3d) =
            let lp = x.Trafo.InvTransformPos point
            x.Box.OutsideFlags(point)
                
        member x.OutsideFlags (point : V3d, eps : float) =
            let lp = x.Trafo.InvTransformPos point
            x.Box.EnlargedBy(eps).OutsideFlags(lp)


        member x.Intersects (o : OrientedBox3d, eps : float) =
            let ha = Hull3d.Create(x.Box).Transformed(Trafo3d x.Trafo).PlaneArray |> Array.map (fun p -> p.Normalized)
            let hb = Hull3d.Create(o.Box).Transformed(Trafo3d o.Trafo).PlaneArray |> Array.map (fun p -> p.Normalized)
            
            let clusterEps = 1E-4
            let clusters = System.Collections.Generic.List<V3d * V3d>()

            let inline contained (na : Set<int>) (nb : Set<int>) (pt : V3d) =
                let mutable inside = true
                for i in 0 .. ha.Length - 1 do
                    if not (Set.contains i na) then
                        inside <- inside && ha.[i].Height pt <= eps

                for j in 0 .. hb.Length - 1 do
                    if not (Set.contains j nb) then
                        inside <- inside && hb.[j].Height pt <= eps

                inside

            let addPoint (pt : V3d) (clustering : System.Collections.Generic.List<struct (V3d * int)>) =
                let rec addPoint (i : int) (pt : V3d) (pts : System.Collections.Generic.List<struct (V3d * int)>) =
                    if i >= pts.Count then
                        pts.Add(struct(pt, 1))
                    else
                        let struct (s, c) = pts.[i]
                        let avg = s / float c
                        if Vec.distance avg pt <= clusterEps then
                            pts.[i] <- struct (s + pt, c + 1)
                        else
                            addPoint (i + 1) pt pts
                addPoint 0 pt clustering


            for i0 in 0 .. ha.Length - 1 do
                let p0 = ha.[i0]
                let pts = System.Collections.Generic.List<struct (V3d * int)>()

                for j0 in 0 .. hb.Length - 1 do
                    let p1 = hb.[j0]
                    for j1 in j0 + 1 .. hb.Length - 1 do
                        let p2 = hb.[j1]
                        let mutable pt = V3d.Zero
                        if p0.Intersects(p1, p2, &pt) && not pt.AnyNaN && not pt.AnyInfinity then
                            if contained (Set.singleton i0) (Set.ofList [j0; j1]) pt then
                                addPoint pt pts

                for struct (s, c) in pts do
                    clusters.Add(s / float c, p0.Normal)

            if clusters.Count = 0 then
                for j0 in 0 .. hb.Length - 1 do
                    let p0 = hb.[j0]
                    let pts = System.Collections.Generic.List<struct (V3d * int)>()

                    for i0 in 0 .. ha.Length - 1 do
                        let p1 = ha.[i0]
                        for i1 in i0 + 1 .. ha.Length - 1 do
                            let p2 = ha.[i1]

                            let mutable pt = V3d.Zero
                            if p0.Intersects(p1, p2, &pt) && not pt.AnyNaN && not pt.AnyInfinity then
                                if contained (Set.ofList [i0; i1]) (Set.singleton j0) pt then
                                    addPoint pt pts
                                
                    //let pts = pts.MapToArray (fun struct (s, c) -> s / float c)
                    for struct (s, c) in pts do
                        clusters.Add(s / float c, -p0.Normal)



            Seq.toArray clusters





module Projectile =
    open System.Threading

    type Shape =
        | Point
        | Sphere of radius : float
        | Box of size : V3d
        
    module Shape =
        let bounds (s : Shape) =
            match s with
            | Point ->
                Box3d(V3d.Zero, V3d.Zero)
            | Sphere r -> 
                Box3d(-V3d(r,r,r), V3d(r,r,r))
            | Box s -> 
                let half = s / 2.0
                Box3d(-half, half)

        let inertia (mass : float) (s : Shape) =
            match s with
            | Point ->
                M33d.Zero, M33d.Zero
            | Sphere r ->
                let i = 2.0 * mass * sqr r / 5.0
                let inv = 1.0 / i
                M33d.FromDiagonal(i), M33d.FromDiagonal(inv)
            | Box s ->
                let ix = mass * (sqr s.Y + sqr s.Z) / 12.0
                let iy = mass * (sqr s.X + sqr s.Z) / 12.0
                let iz = mass * (sqr s.X + sqr s.Y) / 12.0
                M33d.FromDiagonal(ix, iy, iz), M33d.FromDiagonal(1.0 / ix, 1.0 / iy, 1.0 / iz)


    type MotionState(trafo : Euclidean3d, velocity : V3d, angularMomentum : V3d) =  
        member x.Trafo = trafo
        member x.Velocity = velocity
        member x.AngularMomentum = angularMomentum
        

        new (position : V3d, velocity : V3d, angularMomentum : V3d) =
            MotionState(Euclidean3d.Translation position, velocity, angularMomentum)

        new (position : V3d, velocity : V3d) =
            MotionState(Euclidean3d.Translation position, velocity, V3d.Zero)

        new (position : V3d) =
            MotionState(Euclidean3d.Translation position, V3d.Zero, V3d.Zero)

        new (trafo : Euclidean3d, velocity : V3d) =
            MotionState(trafo, velocity, V3d.Zero)

        new (trafo : Euclidean3d) =
            MotionState(trafo, V3d.Zero, V3d.Zero)

    type Body private(shape : Shape, mass : float, inertia : M33d, invInertia : M33d, state : MotionState) =
        member x.Shape = shape
        member x.Mass = mass
        member x.Trafo = state.Trafo
        member x.Velocity = state.Velocity
        member x.AngularVelocity = state.Trafo.TransformDir (invInertia * state.Trafo.InvTransformDir state.AngularMomentum)
        member x.MotionState = state
        member x.Inertia = inertia
        member x.InverseInertia = invInertia
        
        member x.ApplyMomentum(p : V3d) =
            let p = state.Trafo.InvTransformDir p
            let s = MotionState(state.Trafo, state.Velocity + p / mass, state.AngularMomentum)
            Body(shape, mass, inertia, invInertia, s)

        member x.ApplyMomentum(position : V3d, p : V3d) =
            let lPos = state.Trafo.InvTransformPos position
            if Fun.IsTiny(lPos, 1E-8) then
                let s = MotionState(state.Trafo, state.Velocity + p / mass, state.AngularMomentum)
                Body(shape, mass, inertia, invInertia, s)
            else
                let lImpulse = state.Trafo.InvTransformDir p
                let ldl = Vec.cross lPos lImpulse
                let ldp = (Vec.dot lImpulse lPos * lPos) / lPos.LengthSquared

                let dv = state.Trafo.TransformDir (ldp / mass)
                let dl = state.Trafo.TransformDir ldl

                let s = MotionState(state.Trafo, state.Velocity + dv, state.AngularMomentum + dl)
                Body(shape, mass, inertia, invInertia, s)
            
        member x.GetVelocity(position : V3d) =
            let lPosition = state.Trafo.InvTransformPos position
            if Fun.IsTiny(lPosition, 1E-8) then
                state.Velocity
            elif Fun.IsTiny(state.AngularMomentum, 1E-8) then
                state.Velocity
            else
                let ll = state.Trafo.InvTransformDir state.AngularMomentum
                let lw = invInertia * ll
                let vr = state.Trafo.TransformDir (Vec.cross lw lPosition)
                let vl = state.Velocity
                vl + vr
   
        member x.KineticEnergy =
            let w = invInertia * state.Trafo.InvTransformDir state.AngularMomentum
            let v2 = state.Velocity.LengthSquared
            
            let mov = if Fun.IsTiny v2 then 0.0 else 0.5 * mass * v2
            let rot = if Fun.IsTiny w then 0.0 else 0.5 * Vec.dot w (inertia * w)
            mov + rot
            
        member x.WithMotionState(state : MotionState) =
            Body(shape, mass, inertia, invInertia, state)

        member x.WithTrafo(trafo : Euclidean3d) =
            Body(shape, mass, inertia, invInertia, MotionState(trafo, state.Velocity, state.AngularMomentum))

        new(shape : Shape, mass : float, state : MotionState) =
            let inertia, inv = Shape.inertia mass  shape
            Body(shape, mass, inertia, inv, state)

    module Body =

        let performHit (e : float) (pt : V3d) (n : V3d) (l : Body) (r : Body) =
            let vl = l.Velocity
            let vr = r.Velocity
            let vpl = l.GetVelocity pt
            let vpr = r.GetVelocity pt
            
            let lr = l.Trafo.InvTransformPos pt
            let rr = r.Trafo.InvTransformPos pt

            let ln = l.Trafo.InvTransformDir n
            let rn = r.Trafo.InvTransformDir n

            let vrr = vpr - vpl //Vec.dot bn (vp2 - vp1) * bn
            if Vec.dot n vrr >= 0.0 then
                None
            else
                let a = Vec.dot (Vec.cross (l.InverseInertia * (Vec.cross lr ln)) lr) ln
                let b = Vec.dot (Vec.cross (r.InverseInertia * (Vec.cross rr rn)) rr) rn
                let d = a + b + 1.0 / l.Mass + 1.0 / r.Mass
                let jr = (-(1.0 + e) * Vec.dot vrr n) / d

                let p = jr * n
                let vl1 = vl - p / l.Mass
                let vr1 = vr + p / r.Mass


                let ll1 = l.MotionState.AngularMomentum - Vec.cross (l.Trafo.TransformDir lr) p
                let rl1 = r.MotionState.AngularMomentum + Vec.cross (r.Trafo.TransformDir rr) p

                Some (
                    l.WithMotionState(MotionState(l.Trafo, vl1, ll1)),
                    r.WithMotionState(MotionState(r.Trafo, vr1, rl1))
                )

        let integrate (dt : float) (b : Body) =
            let v = b.Velocity
            let w = b.AngularVelocity

            let dRot = Rot3d.FromAngleAxis(w * dt)
            let dPos = v * dt
            b.WithTrafo(
                Euclidean3d(dRot * b.Trafo.Rot, dPos + b.Trafo.Trans)
            )


        module Intersections =
            
            let spherePoint (eps : float) (sphere : MotionState) (radius : float) (point : MotionState) =
                let c = sphere.Trafo.Trans
                let p = point.Trafo.Trans

                let d = p - c
                let r = Vec.length d
                if r <= radius + eps then
                    let n = d / r

                    let vSphere = sphere.Velocity
                    let vPoint = point.Velocity
                    let vRel = vPoint - vSphere

                    if Vec.dot vRel n < 0.0 then Some [p, n]
                    else None
                else
                    None

            let sphereSphere (eps : float) (l : MotionState) (lr : float) (r : MotionState) (rr : float) =
                let lc = l.Trafo.Trans
                let rc = r.Trafo.Trans

                let d = rc - lc
                let distance = Vec.length d
                let dd = lr + rr

                if distance <= dd + eps then
                    let n = d / distance
                    let vRel = r.Velocity - l.Velocity

                    if Vec.dot vRel n < 0.0 then Some [lc + lr*n, n]
                    else None
                else
                    None

            let intersectsBox (eps : float) (ray : Ray3d) (box : Box3d) (tmin : byref<float>) (tmax : byref<float>) =
                let origin = ray.Origin
                let invDir = 1.0 / ray.Direction

                let mutable temp = Unchecked.defaultof<_>
                let mutable t0 = (box.Min - origin) * invDir
                let mutable t1 = (box.Max - origin) * invDir


                if invDir.X < 0.0 then temp <- t0.X; t0.X <- t1.X; t1.X <- temp
                if invDir.Y < 0.0 then temp <- t0.Y; t0.Y <- t1.Y; t1.Y <- temp
         
                if (t0.X > t1.Y + eps || t0.Y > t1.X + eps) then
                    false
                else
                    if invDir.Z < 0.0 then temp <- t0.Z; t0.Z <- t1.Z; t1.Z <- temp

                    t0.X <- max t0.X t0.Y
                    t1.X <- min t1.X t1.Y
            
                    if (t0.X > t1.Z + eps || t0.Z > t1.X + eps) then
                        false
                    else
                        tmin <- max t0.X (max t0.Y t0.Z)
                        tmax <- min t1.X (min t1.Y t1.Z)

                        true       
                        
            let boxPoint (eps : float) (dt : float) (box : Body) (size : V3d) (point : Body) =
                
                let pt = point.Trafo.Trans
                let hs = size / 2.0
                let ob = OrientedBox3d(Box3d(-hs, hs), box.Trafo)

                if ob.Contains(pt, eps) then
                    let trafo = ob.Trafo
                    let bv = box.GetVelocity pt
                    let vRel = point.Velocity - bv

                    if Fun.IsTiny(vRel, eps) then
                        None
                    else
                        let lr = 
                            let dir = trafo.InvTransformDir vRel
                            Ray3d(trafo.InvTransformPos pt, dir)
                        let bb = ob.Box

                        let mutable tmin = System.Double.NegativeInfinity
                        let mutable tmax = System.Double.PositiveInfinity

                        if intersectsBox eps lr bb &tmin &tmax && tmin <= eps && tmin >= -dt - eps then
                            let t = tmin
                            let lhit = lr.GetPointOnRay t

                            let mutable ln = V3d.Zero
                            if lhit.X >= bb.Max.X - eps then ln.X <- 1.0
                            elif lhit.X <= bb.Min.X + eps then ln.X <- -1.0
                        
                            if lhit.Y >= bb.Max.Y - eps then ln.Y <- 1.0
                            elif lhit.Y <= bb.Min.Y + eps then ln.Y <- -1.0
                        
                            if lhit.Z >= bb.Max.Z - eps then ln.Z <- 1.0
                            elif lhit.Z <= bb.Min.Z + eps then ln.Z <- -1.0

                            let n = ob.Trafo.TransformDir ln |> Vec.normalize
                        
                            if Vec.dot n vRel < 0.0 then Some [pt, n]
                            else None
                        else
                            None
                else
                    None

            let sphereBox (eps : float) (sphere : Body) (radius : float) (box : Body) (size : V3d) =
                
                let hs = size / 2.0
                let lb = Box3d(-hs, hs)
                let c = sphere.Trafo.Trans
                let lc = box.Trafo.InvTransformPos c
                let lhit = lc.GetClosestPointOn lb

                let ld = lhit - lc
                let len = Vec.length ld
                if len <= radius + eps then
                    let ln = ld / len
                    let n = box.Trafo.TransformDir ln
                    let p = box.Trafo.TransformPos lhit

                    let sv = sphere.Velocity
                    let bv = box.GetVelocity p

                    let vRel = bv - sv
                    if Vec.dot n vRel < 0.0 then 
                        Some [p, n]
                    else
                        None
                else
                    None
                
            let boxBox (eps : float) (l : Body) (ls : V3d) (r : Body) (rs : V3d) =

                let lb =
                    let hs = ls / 2.0
                    OrientedBox3d(Box3d(-hs, hs), l.Trafo)

                let rb =
                    let hs = rs / 2.0
                    OrientedBox3d(Box3d(-hs, hs), r.Trafo)


                let pts = lb.Intersects(rb, eps)

                let pts =
                    pts |> Array.filter (fun (p, n) ->
                        let vl = l.GetVelocity p
                        let vr = r.GetVelocity p
                        let vRel = vr - vl
                        Vec.dot n vRel < 0.0
                    )

                if pts.Length > 0 then Some (Array.toList pts)
                else None

        let intersect (eps : float) (dt : float) (l : Body) (r : Body) =
            match l.Shape with
            | Shape.Point ->
                match r.Shape with
                | Shape.Point -> None
                | Shape.Sphere radius -> Intersections.spherePoint eps l.MotionState radius r.MotionState
                | Shape.Box size -> Intersections.boxPoint eps dt r size l
            | Shape.Sphere lr ->
                match r.Shape with
                | Shape.Point -> Intersections.spherePoint eps l.MotionState lr r.MotionState
                | Shape.Sphere rr -> Intersections.sphereSphere eps l.MotionState lr r.MotionState rr
                | Shape.Box size -> Intersections.sphereBox eps l lr r size
            | Shape.Box lsize ->
                match r.Shape with
                | Shape.Point -> Intersections.boxPoint eps dt l lsize r
                | Shape.Sphere radius -> Intersections.sphereBox eps r radius l lsize
                | Shape.Box rsize -> Intersections.boxBox eps l lsize r rsize

        let eps = 1E-8

        let rec private findIntersectionTime (offset : float) (size : float) (l : Body) (r : Body) =
            if size <= eps then
                offset + size / 2.0
            else
                let halfSize = size / 2.0
                let tm = offset + halfSize
                let lm = integrate tm l
                let rm = integrate tm r
                match intersect eps halfSize lm rm with
                | Some _ ->
                    findIntersectionTime offset halfSize l r
                | None ->
                    findIntersectionTime tm halfSize l r
                
        let intersectWithin (dt : float) (l : Body) (r : Body) : option<float> =
            let l1 = integrate dt l
            let r1 = integrate dt r

            match intersect eps dt l1 r1 with
            | Some _ ->
                let t = findIntersectionTime 0.0 dt l r
                Some t
            | None ->
                None
                









    type Id private (value : int) =
        static let mutable currentId = 0

        member private x.Value = value

        override x.Equals o =
            match o with
            | :? Id as o -> value = o.Value
            | _ -> false

        override x.GetHashCode() = 
            value

        new() = Id(Interlocked.Increment(&currentId))


    module Hull =
        let intersects (l : Hull3d) (r : Hull3d) =
            let h = Hull3d (Array.append l.PlaneArray r.PlaneArray)
            let pts = h.ComputeCorners() 
            if pts.Count >= 3 then
                let r = Regression3d.ofSeq pts 
                Some (r.Centroid, Regression3d.plane r)
            else
                None


    type World(bodies : HashMap<Id, Body>) =
        static let eps = 1E-6

        static let boxNormal (b : Box3d) (pt : V3d) =
            
            let s = b.Size
            let c = b.Center

            let d = 
                let d = (pt - c).Abs() - s / 2.0
                d.Abs().MinorDim
                
            let s = pt.[d] >= c.[d]

            let mutable v = V3d.Zero
            v.[d] <- 1.0
            if not s then v <- -v
            v

        static let integrate (dt : float) (bodies : HashMap<'a, Body>) =
            if dt <= 0.0 then bodies
            else bodies |> HashMap.map (fun _ -> Body.integrate dt)

        static let intersect (l : Body) (r : Body) =
            match l.Shape, r.Shape with
            | Point, Point -> 
                None
            | Box ls, Box rs ->
                let lb = OrientedBox3d(Box3d(-ls/2.0, ls/2.0), l.Trafo)
                let rb = OrientedBox3d(Box3d(-rs/2.0, rs/2.0), r.Trafo)
                
                let lh = Hull3d.Create(Box3d(-ls/2.0, ls/2.0)).Transformed(Trafo3d l.Trafo)
                let rh = Hull3d.Create(Box3d(-rs/2.0, rs/2.0)).Transformed(Trafo3d r.Trafo)

                match Hull.intersects lh rh with
                | Some (pt, plane) ->
                    let n = plane.Normal
                    Body.performHit 1.0 pt n l r
                | None ->
                    None
                //if pts.Count > 0 then Log.line "asdasd: %A" pts

                //let hit = rb.Corners |> Array.tryFind (fun c -> lb.Box.Contains (l.Trafo.InvTransformPos c))

                //match hit with
                //| Some pt ->
                //    let n = l.Trafo.TransformDir (boxNormal lb.Box (l.Trafo.InvTransformPos pt))
                //    Body.hit 1.0 pt n l r
                //| None ->
                //    let hit = lb.Corners |> Array.tryFind (fun c -> rb.Box.Contains (r.Trafo.InvTransformPos c))
                //    match hit with
                //    | Some pt ->
                //        let n = r.Trafo.TransformDir (boxNormal rb.Box (r.Trafo.InvTransformPos pt))
                //        Body.hit 1.0 pt n l r
                //    | None ->
                //        None
            | _ -> 
                let box, point, size =
                    match l.Shape, r.Shape with
                    | Box bs, Point -> l, r, bs
                    | Point, Box bs -> r, l, bs
                    | _ -> failwith ""


                let bb = Box3d(-size/2.0, size/2.0)
                let pt = point.Trafo.TransformPos V3d.Zero

                let pl = box.Trafo.InvTransformPos pt
                if bb.Contains pl then
                    let bc = box.Trafo.TransformPos V3d.Zero


                    let vp = point.Velocity
                    let vb = box.GetVelocity(pt)

                    let d = 
                        let d = pl.Abs() - (size / 2.0)
                        d.Abs().MinorDim
                        //pl / (size/2.0)
                
                    let s = pl.[d] >= bb.Center.[d]

                    let lbn = 
                        let mutable v = V3d.Zero
                        v.[d] <- 1.0
                        if not s then v <- -v
                        v
                    let bn = 
                        box.Trafo.TransformDir lbn


                    if Vec.dot bn (vp - vb) < 0.0 then

                        Body.performHit 1.0 pt bn box point


                        ////let laxis = Vec.cross lbn pl |> Vec.normalize |> box.Trafo.InvTransformDir
                        ////let i = Vec.dot laxis (box.Inertia * laxis)
                        ////let r2 = Vec.cross laxis pl |> Vec.lengthSquared
                        //let m1 = box.Mass
                        //let m2 = point.Mass

                        //let w1 = box.InverseInertia * box.MotionState.AngularMomentum
                        //let w2 = V3d.Zero

                        //let v1 = box.Velocity
                        //let v2 = point.Velocity
                        //let vp1 = vb
                        //let vp2 = vp
                        //let n = bn

                        //let ii1 = box.InverseInertia
                        //let ii2 = point.InverseInertia


                        //let vr = vp2 - vp1 //Vec.dot bn (vp2 - vp1) * bn
                        //let e = 0.0001
                        //let a = Vec.dot (Vec.cross (ii1 * (Vec.cross pl lbn)) pl) lbn
                        //let d = a + 1.0 / m1 + 1.0 / m2
                        //let jr = (-(1.0 + e) * Vec.dot vr bn) / d

                        //let p = jr * bn

                        //let v1' = v1 - p / m1
                        //let v2' = v2 + p / m2

                        //let ll1 = l.MotionState.AngularMomentum + Vec.cross (bc - pt) p 


                        ////let w1' = w1 - jr * ii1 * (Vec.cross pl lbn)
                        ////let w2' = w2


                        //let l1 = l.WithMotionState(MotionState(l.Trafo, v1', ll1))
                        //let r1 = r.WithMotionState(MotionState(r.Trafo, v2', r.MotionState.AngularMomentum))

                        //Some (l1, r1)


                        //let lv = Vec.dot vb bn
                        //let rv = Vec.dot vp bn
                    
                        //let lv1 = ((m1-m2)/(m1+m2))*lv + (2.0*m2/(m1+m2))*rv
                        //let rv1 = (2.0*m1/(m1+m2))*lv + ((m2-m1)/(m1+m2))*rv
                
                        //let l1 = l.AddVelocity(pt, bn * ( -lv + lv1 ))
                        //let r1 = r.AddVelocity(pt, bn * ( -rv + rv1 ))

                        ////let lv1 = ls.Velocity + dlr * ( -lv + lv1 )
                        ////let rv1 = rs.Velocity + dlr * ( -rv + rv1 )

                        //Some (l1, r1)
                    else
                        None
                else
                    None





        static let rec allPairs (l : list<'a>) =
            match l with
            | [] -> []
            | h :: t ->
                List.append 
                    (t |> List.map (fun t -> (h, t)))
                    (allPairs t)

        static let rec fixIntersections (bodies : HashMap<'a, Body>) =
            let mutable bodies = bodies
            let keys = HashMap.keys bodies |> Seq.toList |> allPairs

            let mutable any = false
            for (l, r) in keys do
                let lb = bodies.[l]
                let rb = bodies.[r]
                match intersect lb rb with
                | Some (l1, r1) ->
                    any <- true
                    bodies <- 
                        bodies
                        |> HashMap.add l l1
                        |> HashMap.add r r1
                | None ->
                    ()
            if any then fixIntersections bodies
            else bodies


        static let rec step (dt : float) (bodies : HashMap<'a, Body>) =
            let bodies = bodies
            let keys = bodies |> Seq.toList |> allPairs

            let mutable results = MapExt.empty<float, list<'a*'a>>


            for ((li,l), (ri, r)) in keys do
                match Body.intersectWithin dt l r with
                | Some dti ->
                    results <- results |> MapExt.alter dti (function Some o -> Some ((li, ri) :: o) | None -> Some [li,ri])
                | None ->
                    ()

            match MapExt.tryMin results with
            | Some dt0 ->
                let all = results |> MapExt.toSeq |> Seq.takeWhile (fun (t,_) -> t <= dt0 + eps) |> Seq.collect (fun (t, g) -> g |> Seq.map (fun v -> t, v))
                
                let mutable bodies = bodies
                let mutable t = 0.0
                for ti, (li, ri) in all do
                    let dt = ti - t
                    bodies <- integrate dt bodies

                    let mutable l = bodies.[li]
                    let mutable r = bodies.[ri]
                    for (pt, n) in Body.intersect eps dt l r |> Option.defaultValue [] do
                        match Body.performHit 1.0 pt n l r with
                        | Some (l1, r1) ->
                            l <- l1
                            r <- r1
                        | None ->
                            ()

                    bodies <-
                        bodies
                        |> HashMap.add li l
                        |> HashMap.add ri r



                    ()

                let missing = dt - t
                if missing > 0.0 then step missing bodies
                else bodies
            | None ->
                integrate dt bodies



        member x.KineticEnergy =
            (0.0, bodies) ||> HashMap.fold (fun s _ b -> s + b.KineticEnergy)
            

        member x.Update (id : Id, update : Body -> Body) =
            bodies
            |> HashMap.alter id (Option.map update)
            |> World

        member x.Add(id : Id, body : Body) =
            World(HashMap.add id body bodies)

        member x.Remove(id : Id) =
            World(HashMap.remove id bodies)

        member x.Step(dt : float) : World = 
            if dt > 0.01 then
                x.Step(dt / 2.0).Step(dt / 2.0)
            else
                bodies
                |> step dt
                //|> integrate dt
                //|> fixIntersections
                |> World

        member x.Bodies = HashMap.toSeq bodies

        
    module World =
        
        let empty = World HashMap.empty
        
        let inline add (id : Id) (body : Body) (world : World) =
            world.Add(id, body)

        let inline remove (id : Id) (world : World) =
            world.Remove id

        let inline step (dt : float) (world : World) =
            world.Step dt

        let inline update (id : Id) (update : Body -> Body) (world : World) =
            world.Update(id, update)



    let test() =


        let x = V3d.III.Normalized
        let y = V3d.PNN.Normalized
        let z = -V3d.PNP.Normalized
        let m = 
            M33d.FromCols(
                x,y,z
            )

        let e =
            match SVD.Decompose m with
            | Some (u,_,vt) -> M44d(u*vt) |> Euclidean3d.FromM44d
            | None -> Euclidean3d.Identity



        let e = Euclidean3d.Translation(V3d(-0.51, 0.3, 0.7) - e.TransformPos(V3d(0.5, 0.5, -0.5))) * e
        
        
        
        let b0 = OrientedBox3d(Box3d.FromCenterAndSize(V3d.Zero, V3d.III), e)
        let b1 = OrientedBox3d(Box3d.FromCenterAndSize(V3d.Zero, V3d(1,1,2)), Euclidean3d.Identity)


        let pts = b0.Intersects(b1, 1E-6)
        for (p, n) in pts do
            Log.line "%s / %s" (p.ToString "0.0000") (n.ToString "0.0000")
        
        let ia = Id()
        let getInitial() = 
            World.empty 
            |> World.add ( Id()) (Body(Box (V3d.III), 1.0, MotionState(e)))
            |> World.add ( Id()) (Body(Box (V3d(1,1,2)), 1.0, MotionState(V3d.Zero)))
            //|> World.add ib (Body(Point, 1.0, MotionState(V3d(3.0,0.7,0.7))))
            //|> World.add ic (Body(Point, 1.0, MotionState(V3d(3.0,-0.7,0.7))))

        let initial = cval(getInitial())

        let win = window { backend Backend.GL }

        let sw = System.Diagnostics.Stopwatch()
        let world =
            initial |> AVal.bind (fun w ->
                let mutable w = w
                let mutable iter = 0
                win.Time |> AVal.map (fun _ ->
                    let dt = sw.Elapsed.TotalSeconds
                    sw.Restart()
                    w <- World.step dt w
                    iter <- iter + 1
                    if iter >= 100 then
                        //Log.line "%.4f" w.KineticEnergy
                        iter <- 0

                    w
                )
            )
            

        let boxTrafos =
            world |> AVal.map (fun w -> 
                w.Bodies 
                |> Seq.choose (fun (_,b) -> 
                    match b.Shape with 
                    | Box s -> 
                        Some (
                            Trafo3d.Scale(s) *
                            Trafo3d b.Trafo
                        )
                    | _ ->
                        None
                )
                |> Seq.toArray
            )

        let sphereTrafos =
            world |> AVal.map (fun w -> 
                w.Bodies 
                |> Seq.choose (fun (_,b) -> 
                    match b.Shape with 
                    | Sphere s -> 
                        Some (
                            Trafo3d.Scale(s) *
                            Trafo3d b.Trafo
                        )
                    | Point ->
                        Some (
                            Trafo3d.Scale(0.02) *
                            Trafo3d b.Trafo
                        )

                    | _ ->
                        None
                )
                |> Seq.toArray
            )


        let objects =
            Sg.ofList [
                Sg.sphere' 3 C4b.Red 1.0
                |> Sg.instanced sphereTrafos

                Sg.box' C4b.Red (Box3d(-V3d.Half, V3d.Half))
                |> Sg.instanced boxTrafos
            ]
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.simpleLighting
            }



        let rand = RandomSystem()
        let b = Box3d.FromCenterAndSize(V3d(0.0, 0.0, 0.0), V3d(0.4, 0.4, 0.4))

        win.Keyboard.DownWithRepeats.Values.Add (fun k ->
            match k with
            | Keys.Space ->
                

                let vp = Array.map2 (*) (AVal.force win.View) (AVal.force win.Proj) |> Array.item 0
                
                //let p = win.Mouse.Position |> AVal.force

                let c = vp.Backward.TransformPosProj (-V3d.OOI)
                let dir = vp.Backward.TransformPosProj (V3d.OOI) - c |> Vec.normalize





                let mutable w = AVal.force world
                for i in 1 .. 1 do
                    let b = Body(Box (V3d.III * 0.1), 0.5, MotionState(Euclidean3d(Rot3d.FromAngleAxis(rand.UniformV3dDirection() * rand.UniformDouble()), c + rand.UniformV3d b), 5.0 * dir))
                    w <- World.add (Id()) b w

                transact (fun () ->
                    initial.Value <- w
                )
         
            | Keys.Escape ->
                transact (fun () ->
                    initial.Value <- getInitial()
                )
            | _ ->
                ()
        )

        win.Scene <- objects

        win.Run()



        ()











        




type Sphere =
    {
        Center : V3d
        Radius : float
        Mass : float
        Velocity : V3d
    }

type World =
    {
        Spheres : HashMap<int,Sphere>
    }



type Line3d with 
    member x.IsDgn =
        Fun.ApproximateEquals(x.P0, x.P1,1E-8)
    member x.MinDist (o : Line3d) =
        if x.IsDgn then 
            if o.IsDgn then
                Vec.distance x.P0 o.P0
            else
                o.GetMinimalDistanceTo(x.P0)
        else
            if o.IsDgn then
                x.GetMinimalDistanceTo(o.P0)
            else
                x.GetMinimalDistanceTo(o)         

    member x.Hugo (o : Line3d) =



        if x.IsDgn || o.IsDgn || Vec.dot x.Direction o.Direction < 0.0 then
            x.MinDist(o)
        else
            System.Double.PositiveInfinity        

let rec allPairs (l : list<'a>) =
    match l with
    | [] -> []
    | h :: t ->
        List.append 
            (t |> List.map (fun t -> (h, t)))
            (allPairs t)


let step (dt : float) (ow : World) : World = 
    let s = 
        ow.Spheres |> HashMap.map (fun _ s -> 
            let d = -Vec.normalize s.Center
            let a = 9.81 * d//V3d(-d.Y,d.X,d.Z)
            let v = s.Velocity + a * dt
            let v = v * pow 0.999 dt
            let v = if Fun.IsTiny(v,1E-6) then V3d.Zero else v
            { s with Center = s.Center + v * dt; Velocity = v }
        )
    { ow with Spheres = s }


let intersectsP (lp : V3d) (lv : V3d) (lr : float) (lm : float) (rp : V3d) (rv : V3d) (rr : float) (rm : float) =
    if Vec.distance lp rp <= lr + rr then 
        let dv = rv - lv  
        if Vec.dot dv (rp - lp) < 0.0 then true
        else false 
        
    else 
        false


let intersection (ow : World) =
    let os = ow.Spheres

    let cs = 
        HashMap.map (fun _ o -> 
            (o.Center, o.Velocity, o.Radius, o.Mass)
        ) os 
        |> HashMap.toList
        |> allPairs
        
    cs |> List.choose (fun ((li,(lc,lv,lr,lm)),(ri,(rc,rv,rr,rm))) -> 
        let lm = ow.Spheres.[li].Mass
        let rm = ow.Spheres.[ri].Mass

        if intersectsP lc lv lr lm rc rv rr rm then
            Some (li,ri)
        else    
            None        
    )    

      
let rec stepWorld (depth : int) (dt : float) (w : World) =
    if dt > 0.01 then 
        (stepWorld depth (dt/2.0) >> stepWorld depth (dt/2.0)) w
    elif dt > 1E-10 then
        let rec doIt (w : World) =
            match intersection w with
            | [] -> w
            | iss -> 
                let mutable mw = w
                for (li,ri) in iss do 
                    let ls = mw.Spheres.[li]
                    let rs = mw.Spheres.[ri]
                    let lm = ls.Mass
                    let rm = rs.Mass
                    let dlr = Vec.normalize (rs.Center - ls.Center)

                    let lv = Vec.dot ls.Velocity dlr
                    let rv = Vec.dot rs.Velocity dlr

                    let lv1 = ((lm-rm)/(lm+rm))*lv + (2.0*rm/(lm+rm))*rv
                    let rv1 = (2.0*lm/(lm+rm))*lv + ((rm-lm)/(lm+rm))*rv

                    let lv1 = ls.Velocity + dlr * ( -lv + lv1 )
                    let rv1 = rs.Velocity + dlr * ( -rv + rv1 )

                    mw <- 
                        {
                            Spheres =
                                mw.Spheres  
                                |> HashMap.add li { ls with Velocity = lv1 }
                                |> HashMap.add ri { rs with Velocity = rv1 }
                        }
                doIt mw
        let nw = step dt w
        doIt nw
    else
        w            

        




[<EntryPoint>]
let main argv =
    Aardvark.Init()

    //exit 0

    Projectile.test()
    exit 0

    let initial =
        {
            Spheres = 
                HashMap.ofList [
                    let mutable cc = 10         
                    let ccc = 2          
                    for i in -ccc..ccc do
                        for j in -ccc..ccc do
                            for k in -ccc..ccc do
                                cc <- cc+1
                                let p = V3d(float i * 4.00, float j * 4.0, float k * 4.0)    
                                let p0 = p.Normalized
                                let v = Vec.cross p V3d.OOI * 8.0                     
                                cc,{ Center = p; Velocity = v; Mass = 1.0; Radius = 1.0 }  
                    // 1, { Center = V3d(20.0,0.0,0.0); Velocity = V3d(-20.0, 0.0, 0.0); Mass = 1.0; Radius = 1.0 }                    
                    // 2, { Center = V3d(80.0,0.0,0.0); Velocity = V3d(-30.0, 0.0, 0.0); Mass = 5.0; Radius = 1.0 }            
                    // 3, { Center = V3d(120.0,0.0,0.0); Velocity = V3d(-40.0, 0.0, 0.0); Mass = 1.0; Radius = 1.0 }                    
                    // 4, { Center = V3d(280.0,0.0,0.0); Velocity = V3d(-80.0, 0.0, 0.0); Mass = 10.0; Radius = 1.0 }            
                ]              

        }

    let floor = 
        Sg.fullScreenQuad
        |> Sg.diffuseTexture DefaultTextures.checkerboard
        |> Sg.transform (Trafo3d.Translation(0.0,0.0,-1.0))
        |> Sg.transform (Trafo3d.Scale(100.0,100.0,1.0))
        |> Sg.shader {
            do! DefaultSurfaces.trafo
            do! DefaultSurfaces.diffuseTexture
        }

    let win =
        window {
            backend Backend.GL        
        }

    let world = cval((0,initial))
    let tw = 
        world |> AVal.bind (fun (_,initial) ->
            let mutable w = initial
            let sw = System.Diagnostics.Stopwatch()
            win.Time |> AVal.map (fun _ ->
                let dt = sw.Elapsed.TotalSeconds
                sw.Restart()
                if dt < 0.05 then
                    w <- stepWorld 500 dt w
                w
            )
        )

    let trafos =
        tw |> AVal.map (fun w ->
            w.Spheres |> Seq.map (fun (_,s) ->
                Trafo3d.Scale(s.Radius) *
                Trafo3d.Translation(s.Center)
            )
            |> Seq.toArray
        )

    let sg =
        Sg.sphere 3 (AVal.constant C4b.Red) (AVal.constant 1.0)
        |> Sg.instanced trafos
        |> Sg.shader {
            do! DefaultSurfaces.trafo
            do! DefaultSurfaces.simpleLighting       
        } //|> Sg.andAlso floor

    win.Keyboard.DownWithRepeats.Values.Add (fun k ->
        match k with
        | Keys.Space ->
            transact (fun () -> 
                let (v,_) = world.Value 
                world.Value <- (v+1, initial)
            )            
        | _ ->
            ()
    )

    win.Scene <- sg
    win.Run()

    0 // return an integer exit code
