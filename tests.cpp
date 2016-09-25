#include <cstdio>
#include "cg_utils.hpp"

using namespace CGutils;

int main()
{
    printf("This is the CG_utils testing app. \n");

    CGVector3D Vs = CGVector3D(2.0f, 3.0f, 4.0f);
    CGVector3D *Vh = new CGVector3D(5.0f, 6.0f, 7.0f);

    printf("Initializing Vs(%f, %f, %f) to stack \n", Vs.GetX(), Vs.GetY(), Vs.GetZ());
    printf("Initializing Vh(%f, %f, %f) to heap \n", Vh->GetX(), Vh->GetY(), Vh->GetZ());

    CGVector3D Vn = CGVector3D::Normalize(Vs);
    printf("Normalizing Normalize(Vs) => Vn(%f, %f, %f) \n", Vn.GetX(), Vn.GetY(), Vn.GetZ());
    printf("Magnitude of Vs(%f, %f, %f) = %f\n", Vs.GetX(), Vs.GetY(), Vs.GetZ(), Vs.Magnitude());
    printf("Magnitude of Vh(%f, %f, %f) = %f\n", Vh->GetX(), Vh->GetY(), Vh->GetZ(), Vh->Magnitude());

    CGVector3D ZeroCGVector3D(0, 0, 0);

    printf("ZeroCGVector3D(%f, %f, %f) \n", 
        ZeroCGVector3D.GetX(), 
        ZeroCGVector3D.GetY(), 
        ZeroCGVector3D.GetZ());

    Matrix m;
    Quaternion q;
    return 0;
}