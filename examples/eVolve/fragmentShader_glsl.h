//Generated file - Edit fragmentShader.glsl!
static const std::string fragmentShader_glsl = "/* Copyright (c) 2007       Maxim Makhinya\n  *\n  * This library is free software; you can redistribute it and/or modify it under\n  * the terms of the GNU Lesser General Public License version 2.1 as published\n  * by the Free Software Foundation.\n  *  \n  * This library is distributed in the hope that it will be useful, but WITHOUT\n  * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS\n  * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more\n  * details.\n  * \n  * You should have received a copy of the GNU Lesser General Public License\n  * along with this library; if not, write to the Free Software Foundation, Inc.,\n  * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.\n  */\n \n \n // input variables to function\n \n uniform sampler3D volume; //gx, gy, gz, v\n uniform sampler2D preInt; // r,  g,  b, a\n \n uniform float shininess;\n uniform vec3  viewVec;\n \n void main (void)\n {\n     vec4 lookupSF;\n     vec4 lookupSB;\n \n     lookupSF = texture3D(volume, gl_TexCoord[0].xyz);\n     lookupSB = texture3D(volume, gl_TexCoord[1].xyz);\n \n     vec4 preInt_ =  texture2D(preInt, vec2(lookupSF.a, lookupSB.a));\n \n     // lighting\n     vec3 normalSF = lookupSF.rgb-0.5;\n     vec3 normalSB = lookupSB.rgb-0.5;\n     vec3 tnorm   = -normalize(normalSF+normalSB);\n \n     vec3 lightVec = normalize( gl_LightSource[0].position.xyz );\n     vec3 reflect  = reflect( -lightVec, tnorm );\n \n     float diffuse = max( dot(lightVec, tnorm), 0.0 );\n \n     float specular = pow(max(dot(reflect, viewVec), 0.0), shininess);\n \n     vec4 color = vec4(gl_LightSource[0].ambient.rgb  * preInt_.rgb +\n                       gl_LightSource[0].diffuse.rgb  * preInt_.rgb * diffuse +\n                       gl_LightSource[0].specular.rgb * preInt_.rgb * specular,\n                       preInt_.a);\n \n     gl_FragColor = color;\n }\n \n ";
