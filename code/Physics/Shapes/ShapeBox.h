//
//	ShapeBox.h
//
#pragma once
#include "ShapeBase.h"

/*
====================================================
ShapeBox
====================================================
*/
class ShapeBox : public Shape
{
public:
    explicit ShapeBox(const Vec3* pts, const int num)
    {
        Build(pts, num);
    }

    void Build(const Vec3* pts, int num);

    Vec3 Support(const Vec3& dir, const Vec3& pos, const Quat& orient, float bias) const override;

    Mat3 InertiaTensor() const override;

    Bounds GetBounds(const Vec3& pos, const Quat& orient) const override;
    Bounds GetBounds() const override { return m_bounds; }

    float FastestLinearSpeed(const Vec3& angularVelocity, const Vec3& dir) const override;

    shapeType_t GetType() const override { return SHAPE_BOX; }

    std::vector<Vec3> m_points;
    Bounds m_bounds;
};
