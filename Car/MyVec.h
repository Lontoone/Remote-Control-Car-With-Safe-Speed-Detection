class Vec3D
{

public:
    float x, y, z;
    float norm;
    Vec3D(){
        x=0;
        y=0;
        z=0;
        norm=0;
    }
    Vec3D(float _x, float _y, float _z)
    {
        x = _x;
        y = _y;
        z = _z;
        norm = 0;
    }
    float GetNorm()const;
    Vec3D Normalize();
    bool IsZero();
    /*
    Vec3D operator+(const Vec3D &);
    Vec3D operator-(const Vec3D &);
    Vec3D operator*(const float &);
    Vec3D operator*(const Vec3D &);
    Vec3D operator/(const float &);
    */
    Vec3D operator+(const Vec3D &that)
    {
        return Vec3D(this->x + that.x, this->y + that.y, this->z + that.z);
    }
    Vec3D operator-(const Vec3D &that)
    {
        return Vec3D(this->x - that.x, this->y - that.y, this->z - that.z);
    }
    Vec3D operator*(const float &that)
    {
        return Vec3D(this->x * that, this->y * that, this->z * that);
    }
    Vec3D operator*(const Vec3D &that)
    {
        return Vec3D(this->x * that.x, this->y * that.y, this->z * that.z);
    }
    Vec3D operator/(const float &that)
    {
        return Vec3D(this->x / that, this->y / that, this->z / that);
    }
    bool operator>(const Vec3D &that)const{
        return this->GetNorm() > that.GetNorm();
    }
    bool operator<(const Vec3D &that)const{
        return this->GetNorm() < that.GetNorm();
    }
    
};
