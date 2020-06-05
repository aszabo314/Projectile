namespace Aardvark.Base

open System
open System.Runtime.CompilerServices


[<Struct>]
type Regression2d private(reference : V2d, sum : V2d, sumSq : V3d, count : int) =

    member x.Count = count
    member x.Centroid = 
        if count = 0 then V2d.Zero
        else reference + sum / float count

    member x.Variance = 
        let avg = sum / float count
        let avgSq = sumSq.XY / float count
        let n = float count
        (avgSq - sqr avg * n) / (n - 1.0)

    member x.Covariance =
        let avg = sum.Y / float count
        let avgSq = sumSq.Z / float count
        let n = float count
        (avgSq - sum.X*avg) / (n - 1.0)

    static member Empty = 
        Regression2d(V2d.Zero, V2d.Zero, V3d.Zero, 0)

    member x.Plane =
        if count < 2 then
            Plane2d.Invalid
        
        else
            let avg = sum / float count
            let avgSq = sumSq / float count

            let n = float count
            let sxx = (avgSq.X - sqr avg.X * n) / (n - 1.0)
            let syy = (avgSq.Y - sqr avg.Y * n) / (n - 1.0)
            let sxy = (avgSq.Z - sum.X*avg.Y) / (n - 1.0)

            
            if Fun.IsTiny sxx || Fun.IsTiny syy then
                if Fun.ApproximateEquals(avg, V2d.Zero) then
                    Plane2d.Invalid
                else
                    let d = Vec.normalize avg
                    let n = V2d(-d.Y, d.X)
                    let mutable plane = Plane2d(n, Vec.dot n (reference + avg))
                    if plane.Distance < 0.0 then plane.Reverse()
                    plane
            else
                let normal = 
                    // (sxx - l) * (syy - l) - sxy^2 = 0
                    // sxx*syy - l*(sxx + syy) + l^2  - sxy^2 = 0
                    // l^2 + (-sxx - syy)*l  + sxx*syy-sxy^2 = 0
                    let struct(l0, l1) = Polynomial.RealRootsOfNormed(-sxx - syy, sxx*syy - sqr sxy)

                    let lambda =
                        if Fun.IsFinite l1 then
                            if abs l0 < abs l1 then l0
                            else l1
                        else
                            l0

                    if Fun.IsNaN lambda then
                        V2d.Zero
                    else
                        // sxy*vx + (syy - l)*vy = 0

                        let a = lambda - sxx
                        let b = sxy

                        let ta = Fun.IsTiny a
                        let tb = Fun.IsTiny b

                        // a*vx + b*vy = 0
                        if ta && tb then
                            V2d.Zero
                        elif ta then 
                            V2d.IO
                        elif tb then
                            V2d.OI
                        elif abs a > abs b then
                            let vx = b / a
                            V2d(vx, 1.0) |> Vec.normalize
                        else
                            let vy = a / b
                            V2d(1.0, vy) |> Vec.normalize

                let mutable plane = Plane2d(normal, Vec.dot normal (reference + avg))
                if plane.Distance < 0.0 then plane.Reverse()
                plane

    member x.Add(pt : V2d) =
        if count <= 0 then
            Regression2d(pt, V2d.Zero, V3d.Zero, 1)
        else
            let pt = pt - reference
            Regression2d(
                reference,
                sum + pt,
                sumSq + V3d(sqr pt.X, sqr pt.Y, pt.X * pt.Y),
                count + 1
            )

    member x.Remove(pt : V2d) =
        if count <= 0 then
            x
        else
            let pt = pt - reference
            Regression2d(
                reference,
                sum - pt,
                sumSq - V3d(sqr pt.X, sqr pt.Y, pt.X*pt.Y) |> max V3d.Zero,
                count - 1
            )
           
    static member Zero = Regression2d.Empty
    static member (+) (l : Regression2d, r : V2d) = l.Add r
    static member (+) (l : V2d, r : Regression2d) = r.Add l

    static member FromSpan(points : Span<V2d>) =
        let mutable r = Regression2d.Empty
        for i in 0 .. points.Length - 1 do r <- r.Add points.[i]
        r
        
    static member inline FromMemory(points : Memory<V2d>) =
        Regression2d.FromSpan points.Span

    static member FromArray(points : V2d[]) =
        let mutable r = Regression2d.Empty
        for i in 0 .. points.Length - 1 do r <- r.Add points.[i]
        r
        
    static member FromList(points : list<V2d>) =
        let mutable r = Regression2d.Empty
        for p in points do r <- r.Add p
        r

    static member FromEnumerable(points : seq<V2d>) =
        let mutable r = Regression2d.Empty
        for p in points do r <- r.Add p
        r

module Regression2d =
    
    let empty = Regression2d.Empty
     
    let inline plane (r : Regression2d) = r.Plane
    let inline count (r : Regression2d) = r.Count
    let inline variance (r : Regression2d) = r.Variance
    let inline covariance (r : Regression2d) = r.Covariance
     
    let inline add (pt : V2d) (r : Regression2d) = r.Add pt
    let inline remove (pt : V2d) (r : Regression2d) = r.Remove pt

    let inline ofArray (pts : V2d[]) = Regression2d.FromArray pts
    let inline ofList (pts : list<V2d>) = Regression2d.FromList pts
    let inline ofSeq (pts : seq<V2d>) = Regression2d.FromEnumerable pts
    

[<Struct>]
type Regression3d private(reference : V3d, sum : V3d, sumSq : V3d, off : V3d, count : int) =
    
    // off.X = sum(yi*zi) 
    // off.Y = sum(xi*zi) 
    // off.Z = sum(xi*yi) 

    static member Empty =
        Regression3d(V3d.Zero, V3d.Zero, V3d.Zero, V3d.Zero, 0)

    member x.Count = count
    member x.Centroid = 
        if count = 0 then V3d.Zero
        else reference + sum / float count

    member x.Add(pt : V3d) =
        if count = 0 then
            Regression3d(pt, V3d.Zero, V3d.Zero, V3d.Zero, 1)
        else
            let pt = pt - reference
            Regression3d(
                reference,
                sum + pt,
                sumSq + sqr pt,
                off + V3d(pt.Y*pt.Z, pt.X*pt.Z, pt.X*pt.Y),
                count + 1
            )

    member x.Remove(pt : V3d) =
        if count <= 0 then
            x
        else
            let pt = pt - reference
            Regression3d(
                reference,
                sum - pt,
                sumSq - sqr pt |> max V3d.Zero,
                off - V3d(pt.Y*pt.Z, pt.X*pt.Z, pt.X*pt.Y),
                count - 1
            )
        
    /// Gets the variances as [Var(X), Var(Y), Var(Z)]
    member x.Variance =
        let avg = sum / float count
        let avgSq = sumSq / float count
        let n = float count
        let sxx = (avgSq.X - sqr avg.X * n) / (n - 1.0)
        let syy = (avgSq.Y - sqr avg.Y * n) / (n - 1.0)
        let szz = (avgSq.Z - sqr avg.Z * n) / (n - 1.0)
        V3d(sxx, syy, szz)

    /// Gets the covariance as [Cov(Y,Z), Cov(X,Z), Cov(X,Y)]
    member x.Covariance =
        let avg = sum / float count
        let avgSq = sumSq / float count
        let n = float count
        let sxy = (off.Z - avg.X*sum.Y * n) / (n - 1.0)
        let sxz = (off.Y - avg.X*sum.Z * n) / (n - 1.0)
        let syz = (off.X - avg.Y*sum.Z * n) / (n - 1.0)
        V3d(syz, sxz, sxy)

    member x.Plane =
        if count < 3 then
            Plane3d.Invalid
        else
            let avg = sum / float count
            let avgSq = sumSq / float count

            // (sum((xi - cx)*(yi - cy)) / (n-1)
            // (sum(xi*yi - cx*yi - cy*xi + cx*cy)) / (n-1)
            // (sum(xi*yi) - cx*sum(yi) - cy*sum(xi) + sum(cx*cy)) / (n-1)
            // (sum(xi*yi) - cx*cy*n - cy*cx*n + cx*cy*n) / (n-1)
            // (sum(xi*yi) - cx*cy*n) / (n-1)

            let n = float count
            let sxx = (avgSq.X - sqr avg.X * n) / (n - 1.0)
            let syy = (avgSq.Y - sqr avg.Y * n) / (n - 1.0)
            let szz = (avgSq.Z - sqr avg.Z * n) / (n - 1.0)

            let sxy = (off.Z - avg.X*sum.Y * n) / (n - 1.0)
            let sxz = (off.Y - avg.X*sum.Z * n) / (n - 1.0)
            let syz = (off.X - avg.Y*sum.Z * n) / (n - 1.0)

            let m = M33d(sxx, sxy, sxz, sxy, syy, syz, sxz, syz, szz)
            match SVD.Decompose m with
            | Some (_u, s, vt) ->
                if Fun.IsTiny s.M00 || Fun.IsTiny s.M11 then
                    Plane3d.Invalid
                else
                    let normal = vt.R2
                    let mutable plane = Plane3d(normal, Vec.dot normal (reference + avg))
                    if plane.Distance < 0.0 then plane.Reverse()
                    plane
            | None ->
                Plane3d.Invalid

  
    static member Zero = Regression3d.Empty
    static member (+) (l : Regression3d, r : V3d) = l.Add r
    static member (+) (l : V3d, r : Regression3d) = r.Add l

    static member FromSpan(points : Span<V3d>) =
        let mutable r = Regression3d.Empty
        for i in 0 .. points.Length - 1 do r <- r.Add points.[i]
        r
        
    static member inline FromMemory(points : Memory<V3d>) =
        Regression3d.FromSpan points.Span

    static member FromArray(points : V3d[]) =
        let mutable r = Regression3d.Empty
        for i in 0 .. points.Length - 1 do r <- r.Add points.[i]
        r
        
    static member FromList(points : list<V3d>) =
        let mutable r = Regression3d.Empty
        for p in points do r <- r.Add p
        r

    static member FromEnumerable(points : seq<V3d>) =
        let mutable r = Regression3d.Empty
        for p in points do r <- r.Add p
        r

module Regression3d =
    
    let empty = Regression3d.Empty
     
    let inline plane (r : Regression3d) = r.Plane
    let inline count (r : Regression3d) = r.Count
    let inline variance (r : Regression3d) = r.Variance
    let inline covariance (r : Regression3d) = r.Covariance
     
    let inline add (pt : V3d) (r : Regression3d) = r.Add pt
    let inline remove (pt : V3d) (r : Regression3d) = r.Remove pt

    let inline ofArray (pts : V3d[]) = Regression3d.FromArray pts
    let inline ofList (pts : list<V3d>) = Regression3d.FromList pts
    let inline ofSeq (pts : seq<V3d>) = Regression3d.FromEnumerable pts


type ErrorMetrics =
    {
        min         : float
        max         : float
        average     : float
        stddev      : float
        rmse        : float
    }

    static member Empty =
        {
            min = Double.PositiveInfinity
            max = Double.PositiveInfinity
            average = Double.PositiveInfinity
            stddev = Double.PositiveInfinity
            rmse = Double.PositiveInfinity
        }

[<AbstractClass; Sealed; Extension>]
type RegressionPlaneExtensions private() =

    [<Extension>]
    static member GetErrorMetrics(plane : Plane2d, points : seq<V2d>) =
        let n = plane.Normalized
        let mutable sum = 0.0
        let mutable sumSq = 0.0
        let mutable l = System.Double.PositiveInfinity
        let mutable h = System.Double.NegativeInfinity
        let mutable cnt = 0

        for p in points do
            let e = n.Height p |> abs
            sum <- sum + e
            sumSq <- sumSq + sqr e
            l <- min l e
            h <- max h e
            cnt <- cnt + 1
            
        if cnt <= 0 then
            ErrorMetrics.Empty
        elif cnt = 1 then
            {
                min = l
                max = h
                average = sum
                stddev = 0.0
                rmse = sum
            }
        else
            // sum((xi - x)^2) / (n-1) =
            // sum(xi^2 - 2*xi*x + x^2) / (n-1) =
            // (sum(xi^2) - 2*x*sum(xi) + sum(x^2)) / (n-1) =
            // (sum(xi^2) - 2*n*x^2 + n*x^2) / (n-1) =
            // (sum(xi^2) - n*x^2) / (n-1) =
            let avg = sum / float cnt
            let var = (sumSq - sum*avg) / (float (cnt - 1))
            let stddev = sqrt var
            {
                min = l
                max = h
                average = avg
                stddev = stddev
                rmse = sqrt (sumSq / float cnt)
            }
     
    [<Extension>]
    static member GetErrorMetrics(plane : Plane3d, points : seq<V3d>) =
        let n = plane.Normalized
        let mutable sum = 0.0
        let mutable sumSq = 0.0
        let mutable l = System.Double.PositiveInfinity
        let mutable h = System.Double.NegativeInfinity
        let mutable cnt = 0

        for p in points do
            let e = n.Height p |> abs
            sum <- sum + e
            sumSq <- sumSq + sqr e
            l <- min l e
            h <- max h e
            cnt <- cnt + 1
            
        if cnt <= 0 then
            ErrorMetrics.Empty
        elif cnt = 1 then
            {
                min = l
                max = h
                average = sum
                stddev = 0.0
                rmse = sum
            }
        else
            // sum((xi - x)^2) / (n-1) =
            // sum(xi^2 - 2*xi*x + x^2) / (n-1) =
            // (sum(xi^2) - 2*x*sum(xi) + sum(x^2)) / (n-1) =
            // (sum(xi^2) - 2*n*x^2 + n*x^2) / (n-1) =
            // (sum(xi^2) - n*x^2) / (n-1) =
            let avg = sum / float cnt
            let var = (sumSq - sum*avg) / (float (cnt - 1))
            let stddev = sqrt var
            {
                min = l
                max = h
                average = avg
                stddev = stddev
                rmse = sqrt (sumSq / float cnt)
            }

