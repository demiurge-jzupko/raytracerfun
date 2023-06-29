/**
 * \file RayTracerFun (VS)
 * \brief Vertex shader complement to the RayTracerFun pixel shader.
 */

///////////////////////////////////////////////////////////////////////////////
// Entry
///////////////////////////////////////////////////////////////////////////////
struct vsIn
{
	float2 Position  : POSITION;
	float2 TexCoords : TEXCOORD0;
};

struct vsOut
{
	float4 Position  : SV_POSITION;
	float2 TexCoords : TEXCOORD0;
};

vsOut main(vsIn input)
{
	vsOut ret;
	ret.Position = float4(input.Position, 0, 1);
	// Just using a TexCoords field
	// to mirror the position to
	// the pixel shader.
	ret.TexCoords = ret.Position.xy;
	return ret;
}
