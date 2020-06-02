namespace Deagle.Model

open System
open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.UI.Primitives
open Adaptify

[<ModelType>]
type Model =
    {
        ballTrafo       : Trafo3d
        boxTrafo        : Trafo3d

        cameraState     : CameraControllerState
    }