#ifndef OLC_PGEX_COLLIDE3D
#define OLC_PGEX_COLLIDE3D

#include <vector>
#include <algorithm>
#undef min
#undef max

#include "olcPGEX_Graphics3D.h"

namespace olc {
	class COLLIDE3D : public olc::PGEX
	{
	public:
		COLLIDE3D() {

		}

		struct polygon {
			olc::GFX3D::mesh m;
			olc::GFX3D::vec3d pos;
			float fTheta = 0.0f;
			bool overlap = false;
			std::vector<olc::GFX3D::vec3d> objectSpace;

			void UpdatePositions() {
				objectSpace.clear();

				olc::GFX3D::mat4x4 poly1RotX = olc::GFX3D::Math::Mat_MakeRotationX(fTheta);
				olc::GFX3D::mat4x4 poly1RotZ = olc::GFX3D::Math::Mat_MakeRotationZ(fTheta);

				olc::GFX3D::mat4x4 poly1Translate = olc::GFX3D::Math::Mat_MakeTranslation(pos.x, pos.y, pos.z);

				olc::GFX3D::mat4x4 poly1World = olc::GFX3D::Math::Mat_MultiplyMatrix(poly1RotX, poly1RotZ);

				poly1World = olc::GFX3D::Math::Mat_MultiplyMatrix(poly1World, poly1Translate);

				for (int i = 0; i < m.verts.size(); i++) {
					objectSpace.push_back(olc::GFX3D::Math::Mat_MultiplyVector(poly1World, m.verts[i]));
				}
			}
		};

	static bool ShapeOverlap_SAT(polygon &r1, polygon &r2) {
			polygon *poly1 = &r1;
			polygon *poly2 = &r2;

			for (int shape = 0; shape < 2; shape++) {
				if (shape == 1) {
					poly1 = &r2;
					poly2 = &r1;
				}

				for (int a = 0; a < poly1->objectSpace.size(); a++) {
					int b = (a + 1) % poly1->objectSpace.size();
					olc::GFX3D::vec3d axisProj = {
						-(poly1->objectSpace[b].y - poly1->objectSpace[a].y),
						poly1->objectSpace[b].x - poly1->objectSpace[a].x,
						poly1->objectSpace[b].z - poly1->objectSpace[a].z
					};

					float d = sqrtf(axisProj.x * axisProj.x + axisProj.y * axisProj.y + axisProj.z * axisProj.z);
					axisProj = {
						axisProj.x / d,
						axisProj.y / d,
						axisProj.z / d
					};

					float min_r1 = INFINITY, max_r1 = -INFINITY;
					for (int p = 0; p < poly1->objectSpace.size(); p++) {
						float q = (poly1->objectSpace[p].x * axisProj.x + poly1->objectSpace[p].y * axisProj.y + poly1->objectSpace[p].z * axisProj.z);
						min_r1 = std::min(min_r1, q);
						max_r1 = std::max(max_r1, q);
					}

					float min_r2 = INFINITY, max_r2 = -INFINITY;
					for (int p = 0; p < poly2->m.verts.size(); p++) {
						float q = (poly2->m.verts[p].x * axisProj.x + poly2->m.verts[p].y * axisProj.y + poly2->m.verts[p].z * axisProj.z);
						min_r2 = std::min(min_r2, q);
						max_r2 = std::max(max_r2, q);
					}

					if (!(max_r2 >= min_r1 && max_r1 >= min_r2))
						return false;
				}
			}

			return true;
		}
	};
}


#endif
