/**
 * \file RayTracerFun (PS)
 * \brief Pixel shader implementation of a ray tracer, for
 * Demiurge Studios' Engineering Reading Group. This is a
 * full screen pixel shader.
 */
 
///////////////////////////////////////////////////////////////////////////////
// Types
///////////////////////////////////////////////////////////////////////////////
struct Directional
{
	float3 Direction;
	float3 Intensity;
};
struct Point
{
	float3 Position;
	float3 Intensity;
};
struct Ray
{
	float3 Position;
	float3 Direction;
};
struct Sphere
{
	float3 Center;
	float  Radius;
	float3 Color;
	float  Shininess;
};

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////
static const float3 AmbientIntensity = float3(0.1, 0.1, 0.1);
static const int DirectionalCount = 1;
static const Directional Directionals[DirectionalCount] =
{
	// Direction,                    Intensity
	{  normalize(float3(-1, 4, -4)), float3(0.4, 0.4, 0.4) },
};
static const int PointCount = 1;
static const Point Points[PointCount] =
{
	// Position,        Intensity
	{  float3(1, 1, 1), float3(1.5, 1.5, 1.5) },
};
static const int SphereCount = 4;
static const Sphere Spheres[SphereCount] =
{
	// Position,             Radius, Color (linear space), Shininess
	{  float3( 0, -1,    3), 1,      float3(1, 0, 0),      500       },
	{  float3( 2,  0,    4), 1,      float3(0, 0, 1),      500       },
	{  float3(-2,  0,    4), 1,      float3(0, 1, 0),      10        },
	{  float3( 0, -5001, 0), 5000,   float3(1, 1, 0),      1000      },
};

/**
 * Furthest distance from the camera in world space - could also be
 * passed in as a uniform.
 */
static const float FarPlane = 2000.0;

///////////////////////////////////////////////////////////////////////////////
// Uniforms
///////////////////////////////////////////////////////////////////////////////
/**
 * Uniform transform that transforms a UV on [-1, 1] into world space
 *
 * \remarks NOTE: To function as expected, the vertex shader *must* rescale
 * default UV coordinates from [0, 1] to [-1, 1] to match screen space
 * vertex coordinates. 
 */
float4x4 InverseViewProjection  = float4x4(1, 0, 0, 0,
                                           0, 1, 0, 0,
                                           1, 0, 1, 1,
                                           0, 0, 0, 1);

///////////////////////////////////////////////////////////////////////////////
// Ray Tracer Fun
///////////////////////////////////////////////////////////////////////////////
/**
 * Convenience, square a scalar value.
 */
float Sqr(float v)
{
	return (v * v);
}

/**
 * Given a distance along \a ray, derive the 3D point.
 */
float3 RayDerivePoint(Ray ray, float distance)
{
	return ray.Position + ray.Direction * distance;
}

/**
 * Check for intersection between a ray and a sphere, both in the same
 * 3D space.
 *
 * @param[in] ray       Ray to intersect, same 3D space as sphere.
 * @param[in] sphere    Sphere to intersect, same 3D space as ray.
 * @param[out] distance Distance along ray of the hit. Valid only on return
 *                      value of true.
 *
 * @return true on hit, false otherwise. If true, \a distance will be populated
 * with the distance of the hit. Use RayDerivePoint() to reconstruct the hit
 * point from the distance and \a ray.
 */
bool Intersect(Ray ray, Sphere sphere, out float distance)
{
	// Vector in the direction of the ray start from the sphere, with magnitude.
	const float3 dirMag = ray.Position - sphere.Center;

	// Signed magnitude of the ray-to-sphere and the ray direction - this must be negative
	// (indicating the center of the sphere is in front of the camera plane) or if it is positive,
	// then the sphere must be closer to the camera near plane than its radius (to project through
	// the plane to be in front of the camera).
	const float b = dot(dirMag, ray.Direction);

	// Distance comparison between the ray/sphere distance and the sphere radius.
	const float c = dot(dirMag, dirMag) - Sqr(sphere.Radius);

	// If both positive (behind the camera and further from the camera than its radius),
	// done. Sphere is behind the camera plane.
	if (b > 0 && c > 0) { return false; }

	// Front case check - signed projection onto the ray.
	const float discr = Sqr(b) - c;
	if (discr < 0) { return false; }

	// Done - distance is the ray projection subtracted from the direct distance.
	distance = max(-b - sqrt(discr), 0);
	return true;
}

/**
 * Uses the uniform UvToCameraWorld transform to derive a 3D world
 * position from \a uv and \a depth (a post projection depth on [0, 1]).
 */
float3 UvToWorldPos(float2 uv, float depth)
{
	float4 tmp = mul(float4(uv, depth, 1), InverseViewProjection);
	return tmp.xyz / tmp.w;
}

/**
 * Determining the shadowing factor from a surface hit point
 * on \a iSphere along a direction vector.
 *
 * @param[in] iSphere         index in the Spheres constant of the starting point.
 * @param[in] surfacePosition Hit point on \a iSphere from which to trace a new
 *                            ray.
 * @param[in] dir             Direction of the ray trace.
 *
 * @return A shadowing value - 1.0 for no shadow, 0.0 for full shadow.
 */
float Shadow(int iSphere, float3 surfacePosition, float3 dir)
{
	// Assemble ray.
	Ray ray;
	ray.Position = surfacePosition;
	ray.Direction = dir;

	// Check all other spheres than the source of the shadow check.
	for (int iOther = 0; iOther < SphereCount; ++iOther)
	{
		// Skip self.
		if (iOther == iSphere) { continue; }

		// Intersect - ignore fDistance. On hit, return the shadowing.
		float fUnusedDistance;
		if (Intersect(ray, Spheres[iOther], fUnusedDistance))
		{
			return 0.0;
		}
	}

	return 1.0;
}

/**
 * @return A 3D unit length normal on \a iSphere, given a point on its surface.
 */
float3 SphereNormal(int iSphere, float3 surfacePosition)
{
	return normalize(surfacePosition - Spheres[iSphere].Center);
}

/**
 * @return The diffuse light contribution from directional lights to \a iSphere
 * at a point on its surface \a surfacePosition.
 *
 * \remarks Factors in shadowing from other spheres.
 */
float3 LightDirectionals(int iSphere, float3 surfacePosition)
{
	// TODO: Specular maybe - don't really wanna but not much else
	// going on in this lighting model.

	const float3 normal = SphereNormal(iSphere, surfacePosition);
	float3 ret = float3(0, 0, 0);
	for (int i = 0; i < DirectionalCount; ++i)
	{
		// Compute the shadowing factor for the directional.
		const float shadow = Shadow(iSphere, surfacePosition, Directionals[i].Direction);
		// Accumulate the contribution of the direction - shadowed intensity (without attenuation)
		// scaled by the front-face dot-normal.
		ret += shadow * Directionals[i].Intensity * max(dot(normal, Directionals[i].Direction), 0);
	}
	return ret;
}

/**
 * @return The diffuse + specular light contribution from point lights to \a iSphere
 * at a point on its surface \a surfacePosition.
 *
 * \remarks Factors in shadowing from other spheres.
 */
float3 LightPoints(float3 vv, int iSphere, float3 surfacePosition)
{
	const float3 normal = SphereNormal(iSphere, surfacePosition);
	float3 ret = float3(0, 0, 0);
	for (int i = 0; i < PointCount; ++i)
	{
		// Light vector for the point light (magnitude)
		const float3 lvm = (Points[i].Position - surfacePosition);
		// Distance and normalized light vector.
		const float distance = length(lvm);
		const float3 lv = normalize(lvm);
		// Quadratic attenuation - minimum attenuation at 1 meter is effectively
		// arbitrary, but must do something there to avoid / 0.
		const float attenuation = 1.0 / (1.0 + Sqr(distance));

		// Reflection vector.
		const float3 rv = 2.0 * normal * max(dot(normal, lv), 0) - lv;
		// ndotl
		const float ndotl = max(dot(normal, lv), 0);
		// rdotv
		const float rdotv = max(dot(rv, vv), 0);

		// Compute the shadowing factor for the point.
		const float shadow = Shadow(iSphere, surfacePosition, lv);
		// Accumulate the contribution of the point - shadowed and attenuated
		// intensity of diffuse (ndol) and specular (rdotv power).
		//
		// NOTE: To factor this out if the book ever has different specular materials,
		// want to accumulate two factors here (diffuse and specular/view dependant)
		// and then return both, which will then be applied against a diffuse material
		// color and a specular/view dependent color.
		ret += shadow * Points[i].Intensity * attenuation * (ndotl + pow(rdotv, Spheres[iSphere].Shininess));
	}
	return ret;
}

/**
 * @return The full diffuse + specular lighting contribution.
 */
float3 LightIntensity(float3 vv, int iSphere, float3 surfacePosition)
{
	return AmbientIntensity
		+ LightDirectionals(iSphere, surfacePosition)
		+ LightPoints(vv, iSphere, surfacePosition);
}

/**
 * @return The fully lit pixel at a surface point \a surfacePosition
 * on sphere \a iSphere. This combines light intensity with material.
 *
 * @param[in] vv              View vector for \a surfacePosition.
 * @param[in] iSphere         Sphere that owns the pixel being lit.
 * @param[in] surfacePosition 3D position on iSphere of the pixel being lit.
 *
 * @return Lit color of the sphere. Color value is in gamma 2.0 and clamped
 * to [0, 1].
 */
half3 Light(float3 vv, int iSphere, float3 surfacePosition)
{
	// Compute light intensity in linear space.
	const float3 lightLinear = LightIntensity(vv, iSphere, surfacePosition);
	// Acquire material color in linear space.
	const float3 matLinear = Spheres[iSphere].Color;

	// Apply light to material, convert to gamma 2, clamp and quantize.
	return half3(saturate(sqrt(matLinear * lightLinear) /* quirk and dirty gamma 2 */));
}

/**
 * Main ray trace function - given a screen space UV, return the fully
 */
bool RayTraceMain(float2 uv, out half3 smpl)
{
	// Screen into 3D world ray - from depth 0 (near plane) to depth 1
	// (far plane) at the given screen UV position.
	Ray ray;
	ray.Position = UvToWorldPos(uv, 0);
	ray.Direction = normalize(UvToWorldPos(uv, 1) - ray.Position);

	// Intersect the ray against all spheres, keep the closest.
	bool ret = false;
	float bestDistance = FarPlane;
	int bestSphere = 0;
	for (int i = 0; i < SphereCount; ++i)
	{
		// On intersection, keep if closer than current best.
		float distance;
		if (Intersect(ray, Spheres[i], distance) && distance < bestDistance)
		{
			ret = true;
			bestDistance = distance;
			bestSphere = i;
		}
	}

	// If we have a hit, light the pixel.
	if (ret)
	{
		smpl = Light(-ray.Direction, bestSphere, RayDerivePoint(ray, bestDistance));
	}

	return ret;
}

///////////////////////////////////////////////////////////////////////////////
// Entry
///////////////////////////////////////////////////////////////////////////////
struct vsOut
{
	float4 Position  : SV_POSITION;
	float2 TexCoords : TEXCOORD0;
};

half4 main(vsOut input) : SV_TARGET
{
	half4 ret;
	half3 smpl;
	if (RayTraceMain(input.TexCoords.xy, smpl))
	{
		ret.rgb = smpl;
	}
	else
	{
		ret.rgb = float3(0.0, 0.3, 0.4); // Sky color.
	}
	ret.a = 1.0;
	return ret;
}
