#include "Body.h"

// 构造函数：初始化物理体的基本属性
Body::Body() :
    m_position(0.0f), // 初始化位置为原点(0,0,0)
    m_orientation(0.0f, 0.0f, 0.0f, 1.0f), // 初始化方向为单位四元数（无旋转）
    m_shape(nullptr) // 初始化形状指针为空
{
}

// 获取质心在世界坐标系中的位置
// 将模型空间的质心转换到世界坐标系
Vec3 Body::GetCenterOfMassWorldSpace() const
{
    // 获取形状在模型空间中的质心
    const Vec3 centerOfMass = m_shape->GetCenterOfMass();
    // 将模型空间质心转换到世界坐标系
    // 公式：世界坐标 = 位置 + 方向旋转(模型空间质心)
    const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
    return pos;
}

// 获取质心在模型坐标系中的位置
// 直接返回形状的质心，因为形状本身就是以模型空间定义的
Vec3 Body::GetCenterOfMassModelSpace() const
{
    // 获取形状在模型空间中的质心
    const Vec3 centerOfMass = m_shape->GetCenterOfMass();
    return centerOfMass;
}

// 将世界坐标系中的点转换到物体本地坐标系
Vec3 Body::WorldSpaceToBodySpace(const Vec3& pt) const
{
    // 计算相对于世界坐标系中质心的位置向量
    Vec3 tmp = pt - GetCenterOfMassWorldSpace();
    // 获取当前方向的逆向旋转
    Quat inverseOrient = m_orientation.Inverse();
    // 使用逆向旋转将世界坐标点转换为物体本地坐标
    Vec3 bodySpace = inverseOrient.RotatePoint(tmp);
    return bodySpace;
}

// 将物体本地坐标系中的点转换到世界坐标系
Vec3 Body::BodySpaceToWorldSpace(const Vec3& pt) const
{
    // 使用当前方向和位置将物体本地坐标转换为世界坐标
    // 公式：世界坐标 = 质心世界坐标 + 方向旋转(本地坐标)
    Vec3 worldSpace = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(pt);
    return worldSpace;
}

// 应用线性冲量
// 根据冲量和质量计算速度变化
void Body::ApplyImpulseLinear(const Vec3& impulse)
{
    // 如果质量倒数为0，表示该物体是静态的，无法施加冲量
    if (0.0f == m_invMass)
    {
        return;
    }
    // 物理公式：
    // 动量 p = m * v 
    // 动量变化 dp = m * dv = J (冲量)
    // => 速度变化 dv = J / m
    m_linearVelocity += impulse * m_invMass;
}
