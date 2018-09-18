using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public static class MathNetUnityConversions
{
	// Raw conversion between vectors (no rotation)

	public static Vector3 ToVector3(this Vector<double> v) {
		return new Vector3((float)v[0], (float)v[1], (float)v[2]);
	}

	public static Vector<double> ToVector(this Vector3 v) {
		return Vector<double>.Build.DenseOfArray(new double[] { v.x, v.y, v.z });
	}

	// Conversion and rotation between vectors

	public static Vector3 ToUnity(this Vector<double> v) {
		return new Vector3((float)v[1], (float)-v[2], (float)v[0]);
	}

	public static Vector<double> FromUnity(this Vector3 v) {
		return Vector<double>.Build.DenseOfArray(new double[] { v.z, v.x, -v.y });
	}

	public static Matrix<double> FromUnity(this Quaternion q) {
		double sqw = q.w*q.w;
		double sqx = q.x*q.x;
		double sqy = q.y*q.y;
		double sqz = q.z*q.z;

		// invs (inverse square length) is only required if quaternion is not already normalised
		double invs = 1 / (sqx + sqy + sqz + sqw);
		double m00 = ( sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
		double m11 = (-sqx + sqy - sqz + sqw)*invs;
		double m22 = (-sqx - sqy + sqz + sqw)*invs;

		double tmp1 = q.x*q.y;
		double tmp2 = q.z*q.w;
		double m10 = 2.0 * (tmp1 + tmp2)*invs;
		double m01 = 2.0 * (tmp1 - tmp2)*invs;

		tmp1 = q.x*q.z;
		tmp2 = q.y*q.w;
		double m20 = 2.0 * (tmp1 - tmp2)*invs;
		double m02 = 2.0 * (tmp1 + tmp2)*invs;
		tmp1 = q.y*q.z;
		tmp2 = q.x*q.w;
		double m21 = 2.0 * (tmp1 + tmp2)*invs;
		double m12 = 2.0 * (tmp1 - tmp2)*invs;

		return Matrix<double>.Build.DenseOfArray(new double[,] {
			{ m22, m20, -m21 },  //z  =  zz   zx  -zy     z
			{ m02, m00, -m01 },  //x  =  xz   xx  -xy  *  x
			{ -m12, -m10, m11 }  //-y = -yz  -yx   yy    -y
		});
	}

	public static Quaternion ToUnity(this Matrix<double> m) {
		return Quaternion.LookRotation(m.Column(0).ToUnity(), -m.Column(2).ToUnity());

	}


}
