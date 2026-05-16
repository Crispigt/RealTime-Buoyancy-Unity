// Gerstner wave function adapted from Catlike Coding's "Waves" tutorial
// by Jasper Flick (MIT-0): https://catlikecoding.com/unity/tutorials/flow/waves/
// Ported to URP with alpha transparency and depth-fade.
Shader "Custom/Waves"
{
    Properties
    {
        _BaseColor  ("Color",      Color)         = (0.1, 0.4, 0.6, 1)
        _Smoothness ("Smoothness", Range(0,1))    = 0.85
        _Metallic   ("Metallic",   Range(0,1))    = 0.0
        [Header(Transparency)]
        _Opacity       ("Surface opacity",    Range(0,1)) = 0.45
        _DepthFadeNear ("Depth fade near (m)", Float)     = 0.5
        _DepthFadeFar  ("Depth fade far  (m)", Float)     = 4.0
        _GrazingOpacityBoost ("Grazing opacity boost", Range(0,1)) = 1.0
        _GrazingPower        ("Grazing opacity power", Float)      = 3.0
        [Header(Waves)]
        _WaveA ("Wave A (dir, steepness, wavelength)", Vector) = (1, 0, 0.25, 60)
        _WaveB ("Wave B", Vector) = (1, 0.6, 0.25, 31)
        _WaveC ("Wave C", Vector) = (1, 1.3, 0.25, 18)
        _WaveTime ("Wave time", Float) = 0
    }

    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent" "RenderPipeline"="UniversalPipeline" }
        LOD 200

        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode"="UniversalForward" }

            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite On
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS_CASCADE
            #pragma multi_compile_fragment _ _SHADOWS_SOFT

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/DeclareDepthTexture.hlsl"

            CBUFFER_START(UnityPerMaterial)
                float4 _BaseColor;
                float  _Smoothness;
                float  _Metallic;
                float  _Opacity;
                float  _DepthFadeNear;
                float  _DepthFadeFar;
                float  _GrazingOpacityBoost;
                float  _GrazingPower;
                float4 _WaveA;
                float4 _WaveB;
                float4 _WaveC;
                float  _WaveTime;
            CBUFFER_END

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS   : NORMAL;
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                float3 positionWS  : TEXCOORD0;
                float3 normalWS    : TEXCOORD1;
                float4 screenPos   : TEXCOORD2;  // for depth-fade
            };

            float3 GerstnerWave(float4 wave, float3 p, inout float3 tangent, inout float3 binormal)
            {
                float steepness  = wave.z;
                float wavelength = wave.w;
                float k = 2.0 * PI / wavelength;
                float c = sqrt(9.8 / k);
                float2 d = normalize(wave.xy);
                float  f = k * (dot(d, p.xz) - c * _WaveTime);
                float  a = steepness / k;

                tangent  += float3(-d.x * d.x * (steepness * sin(f)),
                                    d.x       * (steepness * cos(f)),
                                   -d.x * d.y * (steepness * sin(f)));
                binormal += float3(-d.x * d.y * (steepness * sin(f)),
                                    d.y       * (steepness * cos(f)),
                                   -d.y * d.y * (steepness * sin(f)));
                return float3(d.x * (a * cos(f)),
                                     a * sin(f),
                              d.y * (a * cos(f)));
            }

            Varyings vert(Attributes IN)
            {
                // Compute waves in WORLD space so they don't follow the plane's transform.
                float3 gridPointWS = TransformObjectToWorld(IN.positionOS.xyz);
                float3 tangent = float3(1, 0, 0);
                float3 binormal = float3(0, 0, 1);
                float3 p = gridPointWS;
                p += GerstnerWave(_WaveA, gridPointWS, tangent, binormal);
                p += GerstnerWave(_WaveB, gridPointWS, tangent, binormal);
                p += GerstnerWave(_WaveC, gridPointWS, tangent, binormal);
                float3 normalWS = normalize(cross(binormal, tangent));

                Varyings OUT;
                OUT.positionWS  = p;
                OUT.positionHCS = TransformWorldToHClip(p);
                OUT.normalWS    = normalWS;
                OUT.screenPos   = ComputeScreenPos(OUT.positionHCS);
                return OUT;
            }

            half4 frag(Varyings IN, FRONT_FACE_TYPE frontFace : FRONT_FACE_SEMANTIC) : SV_Target
            {
                float faceSign = IS_FRONT_VFACE(frontFace, 1.0, -1.0);
                float3 normalWS = normalize(IN.normalWS) * faceSign;

                InputData inputData = (InputData)0;
                inputData.positionWS         = IN.positionWS;
                inputData.normalWS           = normalWS;
                inputData.viewDirectionWS    = GetWorldSpaceNormalizeViewDir(IN.positionWS);
                inputData.shadowCoord        = TransformWorldToShadowCoord(IN.positionWS);
                inputData.fogCoord           = 0;
                inputData.vertexLighting     = 0;
                inputData.bakedGI            = SampleSH(normalWS);
                inputData.normalizedScreenSpaceUV = 0;
                inputData.shadowMask         = half4(1,1,1,1);

                // Depth-fade: sample the opaque scene depth behind the water surface.
                float2 uv = IN.screenPos.xy / IN.screenPos.w;
                float rawDepth   = SampleSceneDepth(uv);
                float sceneDepth = LinearEyeDepth(rawDepth, _ZBufferParams);
                float surfDepth  = IN.screenPos.w;   // eye-space distance to this fragment
                float depthDiff  = sceneDepth - surfDepth;
                float depthAlpha = saturate((depthDiff - _DepthFadeNear) / (_DepthFadeFar - _DepthFadeNear));
                float alpha = _Opacity + (1.0 - _Opacity) * depthAlpha;
                float ndotv = saturate(abs(dot(normalWS, inputData.viewDirectionWS)));
                float grazing = pow(1.0 - ndotv, max(_GrazingPower, 0.001));
                alpha = saturate(alpha + (1.0 - alpha) * grazing * _GrazingOpacityBoost);

                SurfaceData surfaceData = (SurfaceData)0;
                surfaceData.albedo     = _BaseColor.rgb;
                surfaceData.metallic   = _Metallic;
                surfaceData.smoothness = _Smoothness;
                surfaceData.alpha      = alpha;
                surfaceData.occlusion  = 1;
                surfaceData.normalTS   = float3(0,0,1);

                half4 color = UniversalFragmentPBR(inputData, surfaceData);
                color.a = alpha;
                return color;
            }
            ENDHLSL
        }
    }
    FallBack "Hidden/Universal Render Pipeline/FallbackError"
}