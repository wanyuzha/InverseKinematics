#pragma once
#include "quaternion.h"
#include "vec3d.h"
#include "mat3d.h"

#ifndef _DUAL_QUATERNION_H_
#define _DUAL_QUATERNION_H_

class Dual_quaternion
{
public:

	Dual_quaternion()
	{
		_quat_0 = Quaternion<double>();
		_quat_e = Quaternion<double>();
	}

	// Set Vec3d to _quat_e_
	Dual_quaternion(Mat3d rotation, Vec3d t)
	{
		_quat_0 = Quaternion<double>::Matrix2Quaternion(rotation.data());
		double s = -0.5f * (t[0] * _quat_0.Getx() + t[1] * _quat_0.Gety() + t[2] * _quat_0.Getz());
		double x = 0.5f * (t[0] * _quat_0.Gets() + t[1] * _quat_0.Getz() - t[2] * _quat_0.Gety());
		double y = 0.5f * (-t[0] * _quat_0.Getz() + t[1] * _quat_0.Gets() + t[2] * _quat_0.Getx());
		double z = 0.5f * (t[0] * _quat_0.Gety() - t[1] * _quat_0.Getx() + t[2] * _quat_0.Gets());
		_quat_e = Quaternion<double>(s, x, y, z);
	}

	Dual_quaternion(const Quaternion<double>& q, Vec3d t)
	{
		_quat_0 = q;
		double s = -0.5f * (t[0] * _quat_0.Getx() + t[1] * _quat_0.Gety() + t[2] * _quat_0.Getz());
		double x = 0.5f * (t[0] * _quat_0.Gets() + t[1] * _quat_0.Getz() - t[2] * _quat_0.Gety());
		double y = 0.5f * (-t[0] * _quat_0.Getz() + t[1] * _quat_0.Gets() + t[2] * _quat_0.Getx());
		double z = 0.5f * (t[0] * _quat_0.Gety() - t[1] * _quat_0.Getx() + t[2] * _quat_0.Gets());
		_quat_e = Quaternion<double>(s, x, y, z);
	}

	Dual_quaternion(const Quaternion<double>& q0, const Quaternion<double>& q1)
	{
		_quat_0 = q0;
		_quat_e = q1;
	}

	void to_transformation(Mat3d& rotation, Vec3d& t)
	{
		// need normalize before transform it back to Mat4d
		double R[9];
		double norm = _quat_0.Norm();
		_quat_0.Normalize();
		_quat_0.Quaternion2Matrix(R);
		rotation.set(R);

		// translation vector from dual quaternion part:
		t[0] = 2.f * (-_quat_e.Gets() * _quat_0.Getx() + _quat_e.Getx() * _quat_0.Gets() - _quat_e.Gety() * _quat_0.Getz() + _quat_e.Getz() * _quat_0.Gety()) / norm;
		t[1] = 2.f * (-_quat_e.Gets() * _quat_0.Gety() + _quat_e.Getx() * _quat_0.Getz() + _quat_e.Gety() * _quat_0.Gets() - _quat_e.Getz() * _quat_0.Getx()) / norm;
		t[2] = 2.f * (-_quat_e.Gets() * _quat_0.Getz() - _quat_e.Getx() * _quat_0.Gety() + _quat_e.Gety() * _quat_0.Getx() + _quat_e.Getz() * _quat_0.Gets()) / norm;
	}

	static Dual_quaternion Identity()
	{
		return Dual_quaternion(Quaternion<double>(1,0,0,0), Vec3d(0,0,0));
	}

	Quaternion<double> rotation() { return _quat_0; }
	Quaternion<double> translation() { return _quat_e; }
	// for convenience when performs q = q + w * q0
	Dual_quaternion operator*(double scalar)
	{
		return Dual_quaternion(scalar * _quat_0, scalar * _quat_e);
	}
	// for convenience when performs q = q + w * q0
	Dual_quaternion operator+(Dual_quaternion q)
	{
		return Dual_quaternion(_quat_0 + q._quat_0, _quat_e + q._quat_e);
	}

protected:
	// basicaly using everything from quaternion.h
	Quaternion<double> _quat_0, _quat_e;
};



#endif // !_DUAL_QUATERNION_H_
