#include <cstdio>
#include "cg_utils.hpp"

using namespace CGutils;

int main()
{
    printf("This is the CG_utils testing app. \n");

    Vector Vs = Vector(2.0f, 3.0f, 4.0f);
    Vector *Vh = new Vector(5.0f, 6.0f, 7.0f);

    printf("Initializing Vs(%f, %f, %f) to stack \n", Vs.GetX(), Vs.GetY(), Vs.GetZ());
    printf("Initializing Vh(%f, %f, %f) to heap \n", Vh->GetX(), Vh->GetY(), Vh->GetZ());

    Vector Vn = Vs.Normalize();
    printf("Normalizing Normalize(Vs) => Vn(%f, %f, %f) \n", Vn.GetX(), Vn.GetY(), Vn.GetZ());
    printf("Magnitude of Vs(%f, %f, %f) = %f\n", Vs.GetX(), Vs.GetY(), Vs.GetZ(), Vs.Magnitude());
    printf("Magnitude of Vh(%f, %f, %f) = %f\n", Vh->GetX(), Vh->GetY(), Vh->GetZ(), Vh->Magnitude());

    Vector  ZeroVector(0, 0, 0);

    printf("ZeroVector(%f, %f, %f) \n", 
        ZeroVector.GetX(), 
        ZeroVector.GetY(), 
        ZeroVector.GetZ());


    return 0;
}