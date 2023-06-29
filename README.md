# raytracerfun
Demiurge Engineering Reading Group personal code, Ray Tracer, "Computer Graphics From Scratch" by Gabriel Gambetta.

# Overview
This is a ray tracer implemented as a full screen pixel shader.

This is a [SHADERed](https://shadered.org) project. You will likely want to download the Desktop version of SHADERed to play with this code, since the Lite [browser version](https://shadered.org/template) does not support importing a project.

If you *do* use the Lite version of SHADERed, or if you want to use these shaders in another environment (they are standard `.hlsl` shaders), you will need to:
- draw a full screen quad. 
- set the `InverseViewProjection` uniform to your inverse view project transform.

SHADERed defaults to an orbit style camera (right-click and drag) or you can toggle a first person camera in the options.
