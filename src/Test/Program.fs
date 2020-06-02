// Learn more about F# at http://fsharp.org

open System
open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.Application
open Aardvark.SceneGraph
open Aardvark.Base.Rendering

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
