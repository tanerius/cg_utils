#include <cstdio>
#include "cg_utils.hpp"

int main()
{
    printf("This is the CG_utils testing app. \n");

    CGutils::Vector Vs = CGutils::Vector(2.0f, 3.0f, 4.0f);
    CGutils::Vector *Vh = new CGutils::Vector(5.0f, 6.0f, 7.0f);

    printf("Initializing Vs(%f, %f, %f) to stack \n", Vs.GetX(), Vs.GetY(), Vs.GetZ());
    printf("Initializing Vh(%f, %f, %f) to heap \n", Vh->GetX(), Vh->GetY(), Vh->GetZ());

    CGutils::Vector Vn = Vs.Normalize();
    printf("Normalizing Normalize(Vs) => Vn(%f, %f, %f) \n", Vn.GetX(), Vn.GetY(), Vn.GetZ());

    return 0;
}