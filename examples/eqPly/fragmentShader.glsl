/*  
 *  fragmentShader.glsl
 *  Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as published
 * by the Free Software Foundation.
 *  
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */ 
    
// Fragment shader for Phong/Blinn-Phong Shading with one light source.


varying vec3 normalEye;
varying vec4 positionEye;


void main()
{
    // normalize interpolated normal, compute view vector from position
    vec3 normal = normalize( normalEye );
    vec3 view = normalize( -positionEye ).xyz;
    
    // compute light vector
    vec3 light;
    if( gl_LightSource[0].position.w == 0.0 )
        // directional light
        light = normalize( gl_LightSource[0].position ).xyz;
    else
        // point light
        light = normalize( gl_LightSource[0].position - positionEye ).xyz;
    
    // compute the ambient component
    //vec4 ambient = gl_FrontLightProduct[0].ambient;
    vec4 ambient = gl_LightSource[0].ambient * gl_Color;
    
    // compute the diffuse component
    float dotLN = dot( light, normal );
    //vec4 diffuse = gl_FrontLightProduct[0].diffuse * max( dotLN, 0.0 );
    vec4 diffuse = gl_LightSource[0].diffuse * gl_Color * max( dotLN, 0.0 );
    
    // compute the specular component
    float factor;
    if( dotLN > 0.0 )
        factor = 1.0;
    else
        factor = 0.0;
    
    // pure Phong
    //vec3 reflect = normalize( reflect( -light, normal ) );
    //vec4 specular = 
    //    gl_FrontLightProduct[0].specular * factor *
    //    max( pow( dot( reflect, view ), gl_FrontMaterial.shininess ), 0.0 );
    
    // modified Blinn-Phong
    vec3 halfway = normalize( light + view );
    vec4 specular = 
        gl_FrontLightProduct[0].specular * factor *
        max( pow( dot( normal, halfway ), gl_FrontMaterial.shininess ), 0.0 );
    
    // sum the components up, defaulting alpha to 1.0
    gl_FragColor = 
        vec4( vec3( gl_FrontLightModelProduct.sceneColor + 
                    ambient + diffuse + specular ), 1.0 );
}
