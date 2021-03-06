/*******************************************************************************
 * Copyright 2011 See AUTHORS file.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *   http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/
/** @author Xoppa */
#ifdef _MSC_VER 
#pragma once
#endif
#ifndef MODELDATA_ATTRIBUTES_H
#define MODELDATA_ATTRIBUTES_H

#include "../json/BaseJSONWriter.h"
#include <map>
#include "../Settings.h"

#define ATTRIBUTE_UNKNOWN		0
#define ATTRIBUTE_POSITION		1
#define ATTRIBUTE_NORMAL		2
#define ATTRIBUTE_COLOR			3
#define ATTRIBUTE_COLORPACKED	4
#define ATTRIBUTE_TANGENT		5
#define ATTRIBUTE_BINORMAL		6
#define ATTRIBUTE_TEXCOORD0		7
#define ATTRIBUTE_TEXCOORD1		8
#define ATTRIBUTE_TEXCOORD2		9
#define ATTRIBUTE_TEXCOORD3		10
#define ATTRIBUTE_TEXCOORD4		11
#define ATTRIBUTE_TEXCOORD5		12
#define ATTRIBUTE_TEXCOORD6		13
#define ATTRIBUTE_TEXCOORD7		14
#define ATTRIBUTE_BLENDWEIGHT0	15
#define ATTRIBUTE_BLENDWEIGHT1	16
#define ATTRIBUTE_BLENDWEIGHT2	17
#define ATTRIBUTE_BLENDWEIGHT3	18
#define ATTRIBUTE_BLENDWEIGHT4	19
#define ATTRIBUTE_BLENDWEIGHT5	20
#define ATTRIBUTE_BLENDWEIGHT6	21
#define ATTRIBUTE_BLENDWEIGHT7	22
#define ATTRIBUTE_COUNT			23

#define ATTRIBUTE_TYPE_SIGNED	0x00
#define ATTRIBUTE_TYPE_UNSIGNED	0x80
#define ATTRIBUTE_TYPE_HEX		0x40
#define ATTRIBUTE_TYPE_FLOAT	0
#define ATTRIBUTE_TYPE_INT		1
#define ATTRIBUTE_TYPE_UINT		(ATTRIBUTE_TYPE_INT | ATTRIBUTE_TYPE_UNSIGNED)
#define ATTRIBUTE_TYPE_UINT_HEX	(ATTRIBUTE_TYPE_UINT | ATTRIBUTE_TYPE_HEX)

#define INIT_VECTOR(T, A) std::vector<T>(A, A + sizeof(A) / sizeof(*A))

#define DATA_TYPE_GL_BYTE                                          0x0
#define DATA_TYPE_GL_UNSIGNED_BYTE                                 0x1
#define DATA_TYPE_GL_SHORT                                         0x2
#define DATA_TYPE_GL_UNSIGNED_SHORT                                0x3
#define DATA_TYPE_GL_INT                                           0x4
#define DATA_TYPE_GL_UNSIGNED_INT                                  0x5
#define DATA_TYPE_GL_FLOAT                                         0x6
#define DATA_TYPE_GL_FIXED                                         0x7

namespace fbxconv {
namespace modeldata {
	static const char * AttributeNames[] = {
		"UNKNOWN", "VERTEX_ATTRIB_POSITION", "VERTEX_ATTRIB_NORMAL", "VERTEX_ATTRIB_COLOR", "COLORPACKED", "VERTEX_ATTRIB_TANGENT", "VERTEX_ATTRIB_BINORMAL",
		"VERTEX_ATTRIB_TEX_COORD", "VERTEX_ATTRIB_TEX_COORD1", "VERTEX_ATTRIB_TEX_COORD2", "VERTEX_ATTRIB_TEX_COORD3", "VERTEX_ATTRIB_TEX_COORD4", "VERTEX_ATTRIB_TEX_COORD5", "VERTEX_ATTRIB_TEX_COORD6", "VERTEX_ATTRIB_TEX_COORD7",
		"VERTEX_ATTRIB_BLEND_WEIGHT", "VERTEX_ATTRIB_BLEND_INDEX", "BLENDWEIGHT2", "BLENDWEIGHT3", "BLENDWEIGHT4", "BLENDWEIGHT5", "BLENDWEIGHT6", "BLENDWEIGHT7"
	};

	static const unsigned short AttributeTypeV2[]		= {ATTRIBUTE_TYPE_FLOAT, ATTRIBUTE_TYPE_FLOAT};
	static const unsigned short AttributeTypeV4[]		= {ATTRIBUTE_TYPE_FLOAT, ATTRIBUTE_TYPE_FLOAT, ATTRIBUTE_TYPE_FLOAT, ATTRIBUTE_TYPE_FLOAT};
	static const unsigned short AttributeTypeV3[]		= {ATTRIBUTE_TYPE_FLOAT, ATTRIBUTE_TYPE_FLOAT, ATTRIBUTE_TYPE_FLOAT};
	static const unsigned short AttributeTypeUIntHex[]	= {ATTRIBUTE_TYPE_UINT_HEX};
	static const unsigned short AttributeTypeBlend[]	= {ATTRIBUTE_TYPE_INT, ATTRIBUTE_TYPE_FLOAT};

	static const std::vector<unsigned short> AttributeTypes[] = {
		INIT_VECTOR(unsigned short, AttributeTypeV4),		// Unknown
		INIT_VECTOR(unsigned short, AttributeTypeV3),		// Position
		INIT_VECTOR(unsigned short, AttributeTypeV3),		// Normal
		INIT_VECTOR(unsigned short, AttributeTypeV4),		// Color
		INIT_VECTOR(unsigned short, AttributeTypeUIntHex),// ColorPacked
		INIT_VECTOR(unsigned short, AttributeTypeV3),		// Tangent
		INIT_VECTOR(unsigned short, AttributeTypeV3),		// Binormal
		INIT_VECTOR(unsigned short, AttributeTypeV2),		// Texcoord0
		INIT_VECTOR(unsigned short, AttributeTypeV2),		// Texcoord1
		INIT_VECTOR(unsigned short, AttributeTypeV2),		// Texcoord2
		INIT_VECTOR(unsigned short, AttributeTypeV2),		// Texcoord3
		INIT_VECTOR(unsigned short, AttributeTypeV2),		// Texcoord4
		INIT_VECTOR(unsigned short, AttributeTypeV2),		// Texcoord5
		INIT_VECTOR(unsigned short, AttributeTypeV2),		// Texcoord6
		INIT_VECTOR(unsigned short, AttributeTypeV2),		// Texcoord7
		INIT_VECTOR(unsigned short, AttributeTypeBlend),	// Blendweight0
		INIT_VECTOR(unsigned short, AttributeTypeBlend),	// Blendweight1
		INIT_VECTOR(unsigned short, AttributeTypeBlend),	// Blendweight2
		INIT_VECTOR(unsigned short, AttributeTypeBlend),	// Blendweight3
		INIT_VECTOR(unsigned short, AttributeTypeBlend),	// Blendweight4
		INIT_VECTOR(unsigned short, AttributeTypeBlend),	// Blendweight5
		INIT_VECTOR(unsigned short, AttributeTypeBlend),	// Blendweight6
		INIT_VECTOR(unsigned short, AttributeTypeBlend)	// Blendweight7
	};
     struct MeshVertexAttrib
	{
		//attribute size
		unsigned int size;
		//GL_FLOAT
		unsigned int type;
		//VERTEX_ATTRIB_POSITION,VERTEX_ATTRIB_COLOR,VERTEX_ATTRIB_TEX_COORD,VERTEX_ATTRIB_NORMAL, VERTEX_ATTRIB_BLEND_WEIGHT, VERTEX_ATTRIB_BLEND_INDEX, GLProgram for detail
		unsigned int name;
		//size in bytes
		//unsigned int attribSizeBytes;
		bool normalize;
		//unsigned int usage;
	};
	#define ATTRIBUTE_SIZE(idx) (AttributeTypes[idx].size())

	struct Attributes : public json::ConstSerializable {
		unsigned long value;
        std::map<std::string, MeshVertexAttrib> attributemap;
        std::map<std::string, MeshVertexAttrib> packedAttributemap;
        Attributes() : value(0) 
        {
            MeshVertexAttrib v1;
            v1.name = ATTRIBUTE_POSITION;
            v1.size = 3;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_POSITION"]= v1;

            v1.name = ATTRIBUTE_NORMAL;
            v1.size = 3;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_NORMAL"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD0;
            v1.size = 2;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_TEX_COORD"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD1;
            v1.size = 2;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_TEX_COORD1"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD2;
            v1.size = 2;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_TEX_COORD2"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD3;
            v1.size = 2;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_TEX_COORD3"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD4;
            v1.size = 2;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_TEX_COORD4"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD5;
            v1.size = 2;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_TEX_COORD5"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD6;
            v1.size = 2;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_TEX_COORD6"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD7;
            v1.size = 2;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_TEX_COORD7"]= v1;


            v1.name = ATTRIBUTE_BLENDWEIGHT0;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_BLEND_WEIGHT"]= v1;

            v1.name = ATTRIBUTE_BLENDWEIGHT1; // a hack
            v1.size = 4;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_BLEND_INDEX"]= v1;

            v1.name = ATTRIBUTE_COLOR;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_COLOR"]= v1;


            v1.name = ATTRIBUTE_TANGENT;
            v1.size = 3;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_TANGENT"]= v1;

            v1.name = ATTRIBUTE_BINORMAL;
            v1.size = 3;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            attributemap["VERTEX_ATTRIB_BINORMAL"]= v1;

            v1.name = ATTRIBUTE_POSITION;
            v1.size = 3 ;
            v1.type = DATA_TYPE_GL_FLOAT;
			v1.normalize = false;
            packedAttributemap["VERTEX_ATTRIB_POSITION"]= v1;

            v1.name = ATTRIBUTE_NORMAL;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_NORMAL"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD0;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_TEX_COORD"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD1;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_TEX_COORD1"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD2;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_TEX_COORD2"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD3;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_TEX_COORD3"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD4;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_TEX_COORD4"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD5;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_TEX_COORD5"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD6;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_TEX_COORD6"]= v1;

            v1.name = ATTRIBUTE_TEXCOORD7;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_TEX_COORD7"]= v1;


            v1.name = ATTRIBUTE_BLENDWEIGHT0;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_BLEND_WEIGHT"]= v1;

            v1.name = ATTRIBUTE_BLENDWEIGHT1; // a hack
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = false;
            packedAttributemap["VERTEX_ATTRIB_BLEND_INDEX"]= v1;

            v1.name = ATTRIBUTE_COLOR;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_UNSIGNED_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_COLOR"]= v1;


            v1.name = ATTRIBUTE_TANGENT;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_TANGENT"]= v1;

            v1.name = ATTRIBUTE_BINORMAL;
            v1.size = 4;
            v1.type = DATA_TYPE_GL_BYTE;
			v1.normalize = true;
            packedAttributemap["VERTEX_ATTRIB_BINORMAL"]= v1;
        }

		Attributes(const unsigned long &v) : value(v) {}

		Attributes(const Attributes &copyFrom) : value(copyFrom.value) {}

		inline bool operator==(const Attributes& rhs) const {
			return value == rhs.value;
		}

		unsigned int size() const {
			unsigned int result = 0;
			for (unsigned int i = 0; i < ATTRIBUTE_COUNT; i++)
				if (has(i))
					result += (unsigned int)ATTRIBUTE_SIZE(i);
			return result;
		}

		unsigned int length() const {
			unsigned int result = 0;
			for (unsigned int i = 0; i < ATTRIBUTE_COUNT; i++)
				if (has(i))
					result++;
			return result;
		}

		/** 0 <= index < length() */
		const int get(unsigned int index) const {
			for (unsigned int i = 0; i < ATTRIBUTE_COUNT; i++)
				if (has(i) && index-- <= 0)
					return i;
			return -1;
		}

		/** 0 <= index < length() */
		const char *name(const unsigned int &index) const {
			const int a = get(index);
			return a < 0 ? 0 : AttributeNames[a];
		}

		/** 0 <= v < size() */
		const unsigned short getType(const unsigned int &v) const {
			unsigned int s = 0;
			for (unsigned int i = 0; i < ATTRIBUTE_COUNT; i++) {
				if (!has(i))
					continue;
				const unsigned short is = (unsigned short)ATTRIBUTE_SIZE(i);
				if ((s + is) > v)
					return AttributeTypes[i][v-s];
				s+=is;
			}
			return 0;
		}

		void set(const unsigned int &attribute, const bool &v) {
			if (v)
				add(attribute);
			else
				remove(attribute);
		}

		void add(const unsigned int &attribute) {
			value |= (1 << attribute);
		}

		void remove(const unsigned int &attribute) {
			value &= -1 ^ (1 << attribute);
		}

		inline bool has(const unsigned int &attribute) const {
			return (value & (1 << attribute)) != 0;
		}

		inline bool hasPosition() const {
			return has(ATTRIBUTE_POSITION);
		}

		void hasPosition(const bool &v) {
			set(ATTRIBUTE_POSITION, v);
		}

		inline bool hasNormal() const {
			return has(ATTRIBUTE_NORMAL);
		}

		void hasNormal(const bool &v) {
			if (v) {
				return; 
			}
			set(ATTRIBUTE_NORMAL, v);
		}

		inline bool hasColor() const {
			return has(ATTRIBUTE_COLOR);
		}

		void hasColor(const bool &v) {
			if (v) {
				return; 
			}
			set(ATTRIBUTE_COLOR, v);
		}

		inline bool hasColorPacked() const {
			return has(ATTRIBUTE_COLORPACKED);
		}

		void hasColorPacked(const bool &v) {
			if (v) {
				return; 
			}
			set(ATTRIBUTE_COLORPACKED, v);
		}

		inline bool hasTangent() const {
			return has(ATTRIBUTE_TANGENT);
		}

		void hasTangent(const bool &v) {
			if (v) {
				return; 
			}
			set(ATTRIBUTE_TANGENT, v);
		}

		inline bool hasBinormal() const {
			return has(ATTRIBUTE_BINORMAL);
		}

		void hasBinormal(const bool &v) {
			if (v) {
				return; 
			}
			set(ATTRIBUTE_BINORMAL, v);
		}

		inline bool hasUV(const unsigned short &uv) const {
			return has(ATTRIBUTE_TEXCOORD0 + uv);
		}

		void hasUV(const unsigned short &uv, const bool &v) {
			set(ATTRIBUTE_TEXCOORD0 + uv, v);
		}

		inline bool hasBlendWeight(const unsigned short &index) const {
			return has(ATTRIBUTE_BLENDWEIGHT0 + index);
		}

		void hasBlendWeight(const unsigned short &index, const bool &v) {
			set(ATTRIBUTE_BLENDWEIGHT0 + index, v);
		}

		virtual void serialize(json::BaseJSONWriter &writer) const;
        	void writeBinary(FILE* file, const struct fbxconv::Settings* settings);
	};
} }

#endif //MODELDATA_ATTRIBUTES_H
