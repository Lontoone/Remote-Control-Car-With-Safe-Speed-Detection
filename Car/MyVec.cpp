#include "MyVec.h"
#include <math.h>
float Vec3D::GetNorm()const{
    return pow( x*x+y*y + z*z ,0.5 );;
}

Vec3D Vec3D::Normalize(){
    if(norm!=0)
        return Vec3D(x,y,z)/norm;
    else
        return Vec3D();
}

bool Vec3D::IsZero(){
    if(this->x ==0 and this->y==0 and this->z==0){
        return true;
      }
      else{
        return false;
        }
  }
