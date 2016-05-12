struct Coefficients{
	float kr, kg, kb;

	Coefficients(){}
	Coefficients(float kr, float kg, float kb){
		this->kr = kr;
		this->kg = kg;
		this->kb = kb;
	}
	Coefficients(const Coefficients& c){
		kr = c.kr;
		kg = c.kg;
		kb = c.kb;
	}

	Coefficients& operator=(const Coefficients& c){
		kr = c.kr;
		kg = c.kg;
		kb = c.kb;

		return *this;
	}

	bool operator==(const Coefficients& c){ return kr == c.kr && kg == c.kg && kb == c.kb; }
};

struct Color {
	float r, g, b;

	Color(){
		r = 0.0;
		g = 0.0;
		b = 0.0;
	}

	Color(const Color& c){
		r = c.r;
		g = c.g;
		b = c.b;
	}

	Color(float r, float g, float b){
		this->r = r;
		this->g = g;
		this->b = b;
	}

	Color& operator=(const Color& c){
		r = c.r;
		g = c.g;
		b = c.b;

		return *this;
	}

	Color operator+(const Color& c){
		Color result;
		result.r = r + c.r > 1 ? 1 : r + c.r;
		result.g = g + c.g > 1 ? 1 : g + c.g;
		result.b = b + c.b > 1 ? 1 : b + c.b;

		return result;
	}
};

Color operator*(float coefficient, const Color& c){
	Color result;
	result.r = coefficient*c.r;
	result.g = coefficient*c.g;
	result.b = coefficient*c.b;

	return result;
}

Color operator*(const Coefficients& k, const Color& c){
	Color result;
	result.r = k.kr * c.r;
	result.g = k.kg * c.g;
	result.b = k.kb * c.b;

	return result;
}

struct Point{
	float x, y, z;

	Point(){
		x = 0;
		y = 0;
		z = 0;
	}

	Point(float x, float y, float z){
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Point(const Point& p){
		x = p.x;
		y = p.y;
		z = p.z;
	}

	Point& operator=(const Point& p){
		x = p.x;
		y = p.y;
		z = p.z;

		return *this;
	}

	bool operator==(const Point& p){
		return x == p.x && y == p.y && z == p.z;
	}
};

struct Vector{
	float x, y, z;

	Vector(){
		x = 0;
		y = 0;
		z = 0;
	}

	Vector(float x, float y, float z){
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector(const Vector& v){
		x = v.x;
		y = v.y;
		z = v.z;
	}

	Vector& operator=(const Vector& v){
		x = v.x;
		y = v.y;
		z = v.z;

		return *this;
	}

	Vector operator+(const Vector& v){
		Vector result;
		result.x = x + v.x;
		result.y = y + v.y;
		result.z = z + v.z;

		return result;
	}

	Vector operator-(const Vector& v){
		Vector result;
		result.x = x - v.x;
		result.y = y - v.y;
		result.z = z - v.z;

		return result;
	}
};

Vector operator*(float scalar, const Vector& v){
	Vector result;
	result.x = scalar*v.x;
	result.y = scalar*v.y;
	result.z = scalar*v.z;

	return result;
}

struct MaterialProperties{
	Coefficients ambientCoeff;
	Coefficients diffuseCoeff;
	Coefficients specularCoeff;
	float specularExpo;
	float reflectance;

	bool operator==(const MaterialProperties& m){
		return ambientCoeff == m.ambientCoeff && diffuseCoeff == m.diffuseCoeff && 
			specularCoeff == m.specularCoeff && specularExpo == m.specularExpo && reflectance == m.reflectance;
	}
};

struct Intersection{
	int numOfPoints;
	float* t;

	Intersection(){ t = NULL; }
	~Intersection(){ if (t != NULL) delete[] t; }
};

class Sphere{
	Point center;
	float radius;
	MaterialProperties properties;
public:
	Sphere(const Point& c, float r) : center(c), radius(r){}

	void setProperties(const MaterialProperties& p){
		properties.ambientCoeff = p.ambientCoeff;
		properties.diffuseCoeff = p.diffuseCoeff;
		properties.specularCoeff = p.specularCoeff;
		properties.specularExpo = p.specularExpo;
		properties.reflectance = p.reflectance;
	}

	const MaterialProperties& getProperties() const { return properties; }
	const Point& getCenter() const { return center; }
	float getRadius() const { return radius; }

	bool operator==(const Sphere& s){ return center == s.center && radius == s.radius && properties == s.properties; }
	bool operator!=(const Sphere& s){ return !(*this == s); }
};

class Plane{
	Point point;
	Vector normal;
	MaterialProperties properties;
public:
	Plane(const Point& p, const Vector& n) : point(p), normal(n){}

	void setProperties(const MaterialProperties& p){
		properties.ambientCoeff = p.ambientCoeff;
		properties.diffuseCoeff = p.diffuseCoeff;
		properties.specularCoeff = p.specularCoeff;
		properties.specularExpo = p.specularExpo;
		properties.reflectance = p.reflectance;
	}

	const Point& getPoint() const { return point; }
	const Vector& getNormal() const { return normal; }
	const MaterialProperties& getProperties() const{ return properties; }
};

class Light{
	Point position;
	Color intensity;
public:
	Light(const Point& p, const Color& i) :position(p), intensity(i){}

	const Point& getPosition() const{ return position; }
	const Color& getIntensity() const { return intensity; }
};

class Camera{
	Point eye;
	Point center;
	Vector up;
public:
	Camera(const Point& e, const Point& c, const Vector& u) :eye(e), center(c), up(u){}

	const Point& getEye() const{ return eye; }
	const Point& getCenter() const{ return center; }
	const Vector& getUp() const{ return up; }
};