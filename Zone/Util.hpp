#include "glm/glm.hpp"
#include "proto/mmo.pb.h"

class Util
{
public:
	static void SetVec3(glm::vec3& gvec, const Vector3D& vec)
	{
		gvec.x = vec.x();
		gvec.y = vec.y();
		gvec.z = vec.z();
	}

	// 工具函数
	static glm::vec3 ProjectOnPlane(const glm::vec3& vector, const glm::vec3& planeNormal)
	{
		// 确保法向量是单位向量
		glm::vec3 normal = glm::normalize(planeNormal);
		return vector - glm::dot(vector, normal) * normal;
	}
};

